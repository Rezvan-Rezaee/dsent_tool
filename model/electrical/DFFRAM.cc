#include "model/electrical/DFFRAM.h"

#include <cmath>
#include <algorithm>

#include "model/PortInfo.h"
#include "model/EventInfo.h"
#include "model/TransitionInfo.h"
#include "model/timing_graph/ElectricalDriverMultiplier.h"
#include "model/timing_graph/ElectricalNet.h"
#include "model/std_cells/StdCell.h"
#include "model/std_cells/StdCellLib.h"
#include "model/electrical/Decoder.h"
#include "model/electrical/Multiplexer.h"
#include "model/ModelGen.h"
#include "model/timing_graph/ElectricalLoad.h"

namespace DSENT
{
    using std::ceil;

    namespace
    {
        // Phase 4.2: taper a BUF chain (EnBuf or RowBuf) toward its real
        // downstream load, working backward from the last stage. Before
        // this existed, EnableFanout/RowClockFanout only set STAGE COUNT
        // (n_stages = ceil(log(count)/log(fanout)), still computed the same
        // way in constructModel()/updateModel() below) -- every stage
        // stayed at construct()'s minimum drive strength, so the parameter
        // controlled how many minimum-size hops the signal took and
        // nothing else. That's why the earlier sweep showed MORE stages
        // (smaller EnableFanout) always winning: each extra hop of a
        // correctly-sized buffer should eventually lose to fewer, bigger
        // hops once intrinsic per-stage delay dominates, but an untapered
        // chain of identical minimum-size buffers never reaches that
        // regime -- adding stages only ever reduces each stage's own
        // fanout load, with no offsetting cost from being undersized.
        //
        // "final_load" is the real capacitance the chain's last stage
        // drives (enable_load for EnBuf; the row's real B-pin load for
        // RowBuf). Per stage, working from the load backward to the
        // source: target = (the load this stage must drive) / fanout, then
        // pick the LARGEST available drive strength whose OWN input
        // capacitance does not exceed that target -- NOT the smallest
        // that's <= target, since the smallest available size trivially
        // satisfies "<= target" for almost any positive target, which
        // would leave every stage at minimum size and produce no taper at
        // all. Taking the largest-that-fits is what actually produces a
        // chain that gets smaller moving toward the source, which is the
        // property this fix's own validation checks for.
        void sizeBufChain(const vector<ElectricalModel*>& stages_, double final_load_,
            double fanout_, Map<double>* cache_, const vector<double>& sizes_)
        {
            if(stages_.empty() || (fanout_ <= 0.0) || sizes_.empty())
            {
                return;
            }

            double target = final_load_;
            // Part 4.3: post-hoc invariant check -- a chain sized by
            // computeChainStages()'s achievable-stage cap (below) should
            // never contain two ADJACENT stages both clamped to the
            // smallest available size (drive ratio 1.0, pure padding
            // delay with no sizing benefit). Track the previously-chosen
            // index (processing runs load-to-source, s = size-1 downto 0)
            // and fail loudly the first time this happens: it means the
            // stage count handed to sizeBufChain() was NOT actually
            // capped at what this same backward walk can support, i.e.
            // computeChainStages()'s achievable-stage computation and
            // this function have gone out of sync.
            int prev_chosen_idx = -1;
            for(int s = (int)stages_.size() - 1; s >= 0; --s)
            {
                target /= fanout_;

                // Smallest available size is the fallback if even that
                // doesn't fit the target -- never leave a stage unsized.
                int chosen_idx = 0;
                double chosen_cap = cache_->get("BUF_X" + (String)sizes_[0] + "->Cap->A");
                for(unsigned int k = 0; k < sizes_.size(); ++k)
                {
                    double a_cap = cache_->get("BUF_X" + (String)sizes_[k] + "->Cap->A");
                    if(a_cap <= target)
                    {
                        chosen_idx = (int)k;
                        chosen_cap = a_cap;
                    }
                }
                ASSERT(!((chosen_idx == 0) && (prev_chosen_idx == 0)),
                    "[Error] sizeBufChain -> chain contains a ratio-1.0 padding "
                    "stage (two adjacent stages both clamped to the minimum "
                    "available size). computeChainStages() should have capped "
                    "the stage count before this point -- its achievable-stage "
                    "bound is out of sync with this function.");
                prev_chosen_idx = chosen_idx;
                stages_[s]->setDrivingStrengthIdx(chosen_idx);

                // Next stage back targets a fraction of what THIS stage
                // actually achieved, not of the idealized (possibly
                // unachievable) target.
                target = chosen_cap;
            }
            return;
        }

        // Part 4.2: the maximum number of stages a chain from `final_load_`,
        // through discrete `sizes_`, tapering by `taper_` each step, can
        // support before every further stage is forced to repeat the
        // smallest available size. Simulates the exact same backward walk
        // as sizeBufChain() above (same `target /= taper_` step, same
        // largest-available-size-that-fits-under-target rule) and stops
        // counting the instant a stage would land on index 0 (the smallest
        // size): one more stage past that point can only ever land on
        // index 0 again too, since dividing an already-floored cap by
        // taper_ again still can't clear anything bigger -- that repeat is
        // exactly the ratio-1.0 padding condition this function exists to
        // rule out. `guard` bounds the loop at sizes_.size()+1 iterations;
        // the loop is mathematically guaranteed to hit index 0 well before
        // that (each non-floor stage strictly decreases the chosen index),
        // so the bound is a defensive cap, not an expected exit path.
        unsigned int computeAchievableStages(double final_load_, double taper_,
            Map<double>* cache_, const vector<double>& sizes_)
        {
            if((taper_ <= 0.0) || sizes_.empty())
            {
                return 0;
            }

            double target = final_load_;
            unsigned int n_stages = 0;
            for(unsigned int guard = 0; guard <= sizes_.size(); ++guard)
            {
                target /= taper_;

                int chosen_idx = 0;
                double chosen_cap = cache_->get("BUF_X" + (String)sizes_[0] + "->Cap->A");
                for(unsigned int k = 0; k < sizes_.size(); ++k)
                {
                    double a_cap = cache_->get("BUF_X" + (String)sizes_[k] + "->Cap->A");
                    if(a_cap <= target)
                    {
                        chosen_idx = (int)k;
                        chosen_cap = a_cap;
                    }
                }
                ++n_stages;
                if(chosen_idx == 0)
                {
                    break;
                }
                target = chosen_cap;
            }
            return std::max((unsigned int)1, n_stages);
        }

        // Part 4.2: loads the discrete drive-strength list shared by every
        // chain (EnBuf and RowBuf both draw from StdCell->AvailableSizes),
        // used both to size chains (sizeBufChain()) and to bound their
        // stage count (computeAchievableStages()).
        void loadStdCellSizes(const TechModel* tech_model_, vector<double>& sizes_)
        {
            const vector<String>& size_strs = tech_model_->get("StdCell->AvailableSizes").split("[,]");
            for(unsigned int k = 0; k < size_strs.size(); ++k)
            {
                sizes_.push_back(size_strs[k].toDouble());
            }
            return;
        }

        // Part 4.2: the enable chain's real downstream load -- see the
        // updateModel() comment this logic was originally inline in for
        // why NAND2_CKGate1_i's cached, never-resized minimum-size A-pin
        // cap times N is the right quantity.
        double computeEnableLoad(unsigned int number_entries_, Map<double>* cache_, const vector<double>& sizes_)
        {
            const String& min_size_str = (String)sizes_[0];
            return (double)number_entries_ * cache_->get("NAND2_X" + min_size_str + "->Cap->A");
        }

        // Part 4.2: the per-row RowBuf chain's real downstream load -- B
        // copies of DFFQ's cached, never-resized minimum-size CK-pin cap.
        double computeRowLoad(unsigned int number_bits_, Map<double>* cache_, const vector<double>& sizes_)
        {
            const String& min_size_str = (String)sizes_[0];
            return (double)number_bits_ * cache_->get("DFFQ_X" + min_size_str + "->Cap->CK");
        }

        // Phase 4.3: EnableFanout/RowClockFanout used to conflate two
        // independent decisions -- how many stages a chain has, and how
        // aggressively each stage tapers -- into one parameter, which is
        // why the earlier fanout sweep was non-monotonic (changing the one
        // knob changed BOTH the stage count formula's log-base AND the
        // sizing target's shrink ratio at once). Split into a *Stages
        // parameter (0 = auto: derive from `count` and the taper ratio,
        // exactly the old formula) and a *Taper parameter (the shrink
        // ratio sizeBufChain() uses, and also the log-base when stage
        // count is auto). taper<=0 disables the chain entirely (direct
        // connection) -- the natural place to put that, since taper<=0 was
        // already meaningless as a log-base (this replaces the old
        // single-parameter's "0 disables" behavior, just moved to the
        // taper knob since the stage-count knob's 0 now means "auto").
        //
        // Part 4.2 update: the count produced above (from the auto formula
        // or from an explicit *Stages override) is now additionally capped
        // at computeAchievableStages()'s bound. Before this cap, a stage
        // count in excess of what StdCell->AvailableSizes can discretely
        // support (whether from the log-based auto formula overshooting a
        // coarse size list, or from a user explicitly setting *Stages too
        // high) still got BUILT -- every excess stage got wired into the
        // graph in constructModel(), then sizeBufChain() clamped it to the
        // smallest available size in updateModel(), since there was
        // nothing smaller left to taper down to. That's the ratio-1.0
        // padding stage this whole fix removes: pure added delay (one more
        // stage of intrinsic gate delay) with zero sizing benefit (the
        // stage before it is the same size, so nothing actually got
        // smaller). Capping here, at construct time, means the padding
        // stage is never built at all -- not built-then-detected.
        unsigned int computeChainStages(unsigned int count_, unsigned int stages_param_, double taper_,
            double final_load_, Map<double>* cache_, const vector<double>& sizes_)
        {
            if(taper_ <= 0.0)
            {
                return 0;
            }
            unsigned int requested = (stages_param_ > 0) ? stages_param_
                : std::max((unsigned int)1, (unsigned int)ceil(log((double)count_) / log(taper_)));
            unsigned int achievable = computeAchievableStages(final_load_, taper_, cache_, sizes_);
            return std::min(requested, achievable);
        }
    } // anonymous namespace

    DFFRAM::DFFRAM(const String& instance_name_, const TechModel* tech_model_)
        : ElectricalModel(instance_name_, tech_model_)
    {
        initParameters();
        initProperties();
    }

    DFFRAM::~DFFRAM()
    {}

    void DFFRAM::initParameters()
    {
        addParameterName("NumberEntries");
        addParameterName("NumberBits");
        // Phase 4.1: replaces the row-enable BroadcastHTree ("EnableTree")
        // with a plain fanout-sized BUF chain, the same kind of structure
        // already used for the per-row CK chain below. The investigation
        // (docs/timing-and-clock-distribution.md, Part 2.2/2.5) found the
        // wire-distribution H-tree model to be a category error at this
        // scale: EnableTree's NumberLevels was tied to ceil(log2(NumberEntries))
        // -- a topological, endpoint-count-driven choice appropriate for a
        // real multi-branch wire network -- while the load it actually
        // drives (enable_load, below) is a fixed, small, N-independent gate
        // fanout with no meaningful wire component at DFFRAM's physical
        // scale. ClockTree->NumberLevels and ClockTree->WireLayer no longer
        // apply (nothing in this file reads them any more) and have been
        // removed; EnableFanout replaces them.
        // Phase 4.3: EnableFanout split into EnableStages (how many BUF
        // stages, 0 = auto from EnableTaper) and EnableTaper (the
        // per-stage size ratio; also EnableStages=0's log-base). Taper<=0
        // disables the chain (direct connection, the pre-Phase-4
        // unbuffered behaviour) so its effect on timing/area can still be
        // isolated. See computeChainStages()'s own comment for why this
        // was split.
        addParameterName("EnableStages", "0");
        addParameterName("EnableTaper", "4");
        // Same split for the per-row CK chain.
        addParameterName("RowClockStages", "0");
        addParameterName("RowClockTaper", "4");
        return;
    }

    void DFFRAM::initProperties()
    {
        return;
    }

    DFFRAM* DFFRAM::clone() const
    {
        // TODO
        ASSERT(false, "[Error] " + getInstanceName() + " -> Clone not implemented for DFFRAM!");
        return NULL;
    }

    void DFFRAM::constructModel()
    {
        // Get parameters
        unsigned int number_bits = getParameter("NumberBits").toUInt();
        unsigned int number_entries = getParameter("NumberEntries").toUInt();

        ASSERT(number_bits > 0, "[Error] " + getInstanceName() + 
            " -> Number of bits must be > 0!");
        ASSERT(number_entries > 0, "[Error] " + getInstanceName() + 
            " -> Number of entries must be > 0!");

        unsigned int number_addr_bits = (unsigned int)ceil(log2(number_entries));

        // Create ports
        createInputPort("In", makeNetIndex(0, number_bits-1));
        for(unsigned int i = 0; i < number_addr_bits; ++i)
        {
            createInputPort("WRAddr" + (String)i);
            createInputPort("RDAddr" + (String)i);
        }
        createInputPort("WE");
        createInputPort("CK");
        createOutputPort("Out", makeNetIndex(0, number_bits-1));

        // Create energy, power, and area results
        createElectricalResults();
        getEventInfo("Idle")->setStaticTransitionInfos();
        getEventInfo("Idle")->setTransitionInfo("CK", TransitionInfo(0.0, 1.0, 0.0)); // defines the percentage of the clock period each transition takes
        getEventInfo("Idle")->setTransitionInfo("WE", TransitionInfo(1.0, 0.0, 0.0));

        createElectricalEventResult("Read");
        getEventInfo("Read")->setTransitionInfo("CK", TransitionInfo(0.0, 1.0, 0.0));
        getEventInfo("Read")->setTransitionInfo("WE", TransitionInfo(1.0, 0.0, 0.0));
        for(unsigned int i = 0; i < number_addr_bits; ++i)
        {
            getEventInfo("Read")->setTransitionInfo("WRAddr" + (String)i, TransitionInfo(0.5, 0.0, 0.5));
        }
        createElectricalEventResult("Write");
        getEventInfo("Write")->setTransitionInfo("CK", TransitionInfo(0.0, 1.0, 0.0));
        getEventInfo("Write")->setTransitionInfo("WE", TransitionInfo(0.0, 0.0, 1.0));
        for(unsigned int i = 0; i < number_addr_bits; ++i)
        {
            getEventInfo("Write")->setTransitionInfo("RDAddr" + (String)i, TransitionInfo(0.5, 0.0, 0.5));
        }

        // Init components - DFF array, Dec, Mux
        vector<String> dff_names(number_entries, "");
        vector<StdCell*> dffs(number_entries, NULL);
        for(unsigned int i = 0; i < number_entries; ++i)
        {
            dff_names[i] = "DFF_" + (String)i;
            dffs[i] = getTechModel()->getStdCellLib()->createStdCell("DFFQ", dff_names[i]);
            dffs[i]->construct();
        }
                    
        const String& dec_name = "Dec";
        Decoder* dec = new Decoder(dec_name, getTechModel());
        dec->setParameter("NumberOutputs", number_entries);
        dec->construct();

        const String& mux_name = "Mux";
        Multiplexer* mux = new Multiplexer(mux_name, getTechModel());
        mux->setParameter("NumberInputs", number_entries);
        mux->setParameter("NumberBits", 1);
        mux->setParameter("BitDuplicate", "TRUE");
        mux->construct();

        // Init components - CK & WE
        const String& nand2cg0_name = "NAND2_CKGate0";
        StdCell* nand2cg0 = getTechModel()->getStdCellLib()->createStdCell("NAND2", nand2cg0_name);
        nand2cg0->construct();
        const String& invcg0_name = "INV_CKGate0";
        StdCell* invcg0 = getTechModel()->getStdCellLib()->createStdCell("INV", invcg0_name);
        invcg0->construct();

        // Init components - (CK & WE) & DecOut[i]
        vector<String> nand2cg1_names(number_entries, "");
        vector<StdCell*> nand2cg1s(number_entries, NULL);
        vector<String> invcg1_names(number_entries, "");
        vector<StdCell*> invcg1s(number_entries, NULL);
        for(unsigned int i = 0; i < number_entries; ++i)
        {
            nand2cg1_names[i] = "NAND2_CKGate1_" + (String)i;
            nand2cg1s[i] = getTechModel()->getStdCellLib()->createStdCell("NAND2", nand2cg1_names[i]);
            nand2cg1s[i]->construct();

            invcg1_names[i] = "INV_CKGate1_" + (String)i;
            invcg1s[i] = getTechModel()->getStdCellLib()->createStdCell("INV", invcg1_names[i]);
            invcg1s[i]->construct();
        }

        // Connect Decoder
        for(unsigned int i = 0; i < number_addr_bits; ++i)
        {
            portConnect(dec, "Addr" + (String)i, "WRAddr" + (String)i);
        }
        for(unsigned int i = 0; i < number_entries; ++i)
        {
            createNet("Dec_Out" + (String)i);
            portConnect(dec, "Out" + (String)i, "Dec_Out" + (String)i);
        }

        // Connect CKGate0 - CK, WE
        createNet("NAND2_CKGate0_Out");
        createNet("CKGate0_Out");
        portConnect(nand2cg0, "A", "CK");
        portConnect(nand2cg0, "B", "WE");
        portConnect(nand2cg0, "Y", "NAND2_CKGate0_Out");
        portConnect(invcg0, "A", "NAND2_CKGate0_Out");
        portConnect(invcg0, "Y", "CKGate0_Out");

        // ---- Phase 4.1, fix (a): row-enable distribution ------------------
        // CRITICAL PLACEMENT (unchanged from the tree this replaces): this
        // chain sits on the GATED ENABLE path, between INV_CKGate0 and the
        // NAND2_CKGate1_i array -- it does NOT touch the CK net itself.
        // Router.cc already instantiates a router-level ClockTree (sized by
        // sqrt(router_area)) that carries CK to this buffer; a second tree
        // here would double-count that. The unbuffered fanout from one
        // small inverter (INV_CKGate0) to all N NAND2_CKGate1_i row-enable
        // gates is a separate, downstream problem, addressed here.
        //
        // Structure: a plain, tapered BUF chain sized from the real
        // electrical fanout (N gates), NOT from a wire-distribution
        // topology -- mirrors the per-row RowBuf chain below exactly
        // (n_buf_stages = ceil(log(count)/log(fanout))), rather than the
        // BroadcastHTree this replaces, whose NumberLevels was tied to
        // ceil(log2(NumberEntries)) as if N rows were N physically
        // separated destinations needing that many series wire-branch
        // decisions. At DFFRAM's physical scale there is effectively no
        // wire term to distribute (see docs/timing-and-clock-distribution.md,
        // Part 2.2) -- what actually needs sizing is drive strength against
        // a lumped gate-capacitance fanout, which is exactly what a plain
        // buffer chain (no wire segments, no SitePitch/array-area coupling)
        // models directly.
        //
        // Unlike BroadcastHTree (Out_Ron created non-sizable, In_to_Out_delay
        // a single opaque ElectricalDelay node -- see the investigation
        // report, Part 3.2), every stage here is wired with portConnect, so
        // each stage's real driver/load nodes ARE part of the parent timing
        // graph and ARE individually reachable/sizable by a top-level
        // ElectricalTimingOptimizer walk. This is a deliberate improvement:
        // the enable chain is no longer a black box to global timing
        // optimization the way the tree it replaces was.
        unsigned int enable_stages_param = getParameter("EnableStages").toUInt();
        double enable_taper = getParameter("EnableTaper").toDouble();
        // Part 4.2: stage count must be bounded by the achievable-size cap
        // BEFORE any stage is built, or an excess stage gets wired into
        // the graph here and only clamped-to-minimum later (the padding
        // bug). The cache is safe to read at construct time: StdCellLib's
        // constructor populates it synchronously (DSENT.cc's
        // constructTechModel(), called before any model is built), and
        // enable_load only depends on NAND2_CKGate1_i's fixed, never-resized
        // minimum-size cap -- no sub-instance needs to be update()'d first.
        Map<double>* construct_cache = getTechModel()->getStdCellLib()->getStdCellCache();
        vector<double> construct_sizes;
        loadStdCellSizes(getTechModel(), construct_sizes);
        double construct_enable_load = computeEnableLoad(number_entries, construct_cache, construct_sizes);
        unsigned int n_enable_stages = computeChainStages(number_entries, enable_stages_param, enable_taper,
            construct_enable_load, construct_cache, construct_sizes);
        vector<StdCell*> enable_bufs;
        String enable_ck_source = "CKGate0_Out";
        for(unsigned int s = 0; s < n_enable_stages; ++s)
        {
            const String& buf_name = "EnBuf_" + (String)s;
            StdCell* buf = getTechModel()->getStdCellLib()->createStdCell("BUF", buf_name);
            buf->construct();
            // Phase 4.2: NOT sized here. construct() leaves every stage at
            // minimum size; real per-stage sizing against the actual
            // downstream load happens in updateModel() (sizeBufChain()),
            // since it needs enable_load, which isn't known until the
            // NAND2_CKGate1_i array's own cached load is available -- see
            // updateModel()'s own comment for why a construct()-time
            // step-count-based approximation (what this used to do) doesn't
            // make EnableStages/EnableTaper mean what they say.
            portConnect(buf, "A", enable_ck_source);
            const String& buf_out_net = "EnBufOut_" + (String)s;
            createNet(buf_out_net);
            portConnect(buf, "Y", buf_out_net);
            enable_bufs.push_back(buf);
            enable_ck_source = buf_out_net;
        }
        // enable_ck_source is "CKGate0_Out" itself when EnableTaper<=0
        // (direct connection, pre-Phase-4 behaviour restored for isolation).
        const String& enable_ck_final = enable_ck_source;

        // ---- Phase 4, fix (b): per-row CK fanout -------------------------
        // A 1-D fanout along one row's B flop CK pins (not a 2-D H-tree
        // problem) -- fanout-sized BUF chain instead. RowClockTaper<=0
        // disables this (direct connection, as before) so fix (a) and fix
        // (b) can be measured in isolation.
        unsigned int row_clock_stages_param = getParameter("RowClockStages").toUInt();
        double row_clock_taper = getParameter("RowClockTaper").toDouble();
        // Part 4.2: same reasoning as construct_enable_load above -- reuse
        // the same cache/sizes already loaded for the enable chain.
        double construct_row_load = computeRowLoad(number_bits, construct_cache, construct_sizes);
        unsigned int n_buf_stages = computeChainStages(number_bits, row_clock_stages_param, row_clock_taper,
            construct_row_load, construct_cache, construct_sizes);
        vector<vector<StdCell*> > row_bufs(number_entries, vector<StdCell*>());
        vector<String> row_ck_sources(number_entries, "");

        // Connect CKGate1 - CKGate0 (via the EnBuf chain), Dec_Out
        for(unsigned int i = 0; i < number_entries; ++i)
        {
            createNet("NAND2_CKGate1_Outs" + (String)i);
            createNet("CKGate1_Outs" + (String)i);
            portConnect(nand2cg1s[i], "A", enable_ck_final);
            portConnect(nand2cg1s[i], "B", "Dec_Out" + (String)i);
            portConnect(nand2cg1s[i], "Y", "NAND2_CKGate1_Outs" + (String)i);
            portConnect(invcg1s[i], "A", "NAND2_CKGate1_Outs" + (String)i);
            portConnect(invcg1s[i], "Y", "CKGate1_Outs" + (String)i);

            // Insert the row's BUF chain between INV_CKGate1_i and its
            // flop CK pins, replacing the direct connection. Also NOT sized
            // here -- same reason as the EnBuf chain above, see
            // updateModel().
            String row_ck_source = "CKGate1_Outs" + (String)i;
            for(unsigned int s = 0; s < n_buf_stages; ++s)
            {
                const String& buf_name = "RowBuf" + (String)i + "_" + (String)s;
                StdCell* buf = getTechModel()->getStdCellLib()->createStdCell("BUF", buf_name);
                buf->construct();
                portConnect(buf, "A", row_ck_source);
                const String& buf_out_net = "RowBufOut" + (String)i + "_" + (String)s;
                createNet(buf_out_net);
                portConnect(buf, "Y", buf_out_net);
                row_bufs[i].push_back(buf);
                row_ck_source = buf_out_net;
            }
            row_ck_sources[i] = row_ck_source;
        }

        // Connect DFF array
        for(unsigned int i = 0; i < number_entries; ++i)
        {
            createNet("DFF_Out" + (String)i);
            for(unsigned int n = 0; n < number_bits; ++n)
            {
                portConnect(dffs[i], "D", "In", makeNetIndex(n));
                portConnect(dffs[i], "CK", row_ck_sources[i]);
            }
            portConnect(dffs[i], "Q", "DFF_Out" + (String)i);
        }

        // Connect Multiplexer
        createNet("Mux_Out");
        for(unsigned int i = 0; i < number_entries; ++i)
        {
            portConnect(mux, "In" + (String)i, "DFF_Out" + (String)i);
        }
        for(unsigned int i = 0; i < number_addr_bits; ++i)
        {
            portConnect(mux, "Sel" + (String)i, "RDAddr" + (String)i);
        }
        portConnect(mux, "Out", "Mux_Out");

        // Use driver multiplier to connect Mux_Out to Out
        createDriverMultiplier("OutMult");
        ElectricalDriverMultiplier* drive_mult = getDriverMultiplier("OutMult");
        getNet("Mux_Out")->addDownstreamNode(drive_mult);
        for(unsigned int n = 0; n < number_bits; ++n)
        {
            drive_mult->addDownstreamNode(getNet("Out", makeNetIndex(n)));
        }

        // Add area and power results
        for(unsigned int i = 0; i < number_entries; ++i)
        {
            addSubInstances(dffs[i], number_bits);
            addElectricalSubResults(dffs[i], number_bits);
        }

        addSubInstances(dec, 1.0);
        addElectricalSubResults(dec, 1.0);

        addSubInstances(mux, number_bits);
        addElectricalSubResults(mux, number_bits);

        addSubInstances(nand2cg0, 1.0);
        addElectricalSubResults(nand2cg0, 1.0);

        addSubInstances(invcg0, 1);
        addElectricalSubResults(invcg0, 1.0);

        for(unsigned int s = 0; s < enable_bufs.size(); ++s)
        {
            addSubInstances(enable_bufs[s], 1.0);        // n_enable_stages instances, weight 1 each
            addElectricalSubResults(enable_bufs[s], 1.0);
        }

        for(unsigned int i = 0; i < number_entries; ++i)
        {
            addSubInstances(nand2cg1s[i], 1);
            addElectricalSubResults(nand2cg1s[i], 1.0);

            addSubInstances(invcg1s[i], 1);
            addElectricalSubResults(invcg1s[i], 1.0);

            for(unsigned int s = 0; s < row_bufs[i].size(); ++s)
            {
                addSubInstances(row_bufs[i][s], 1.0);        // N * n_stages instances, weight 1 each
                addElectricalSubResults(row_bufs[i][s], 1.0);
            }
        }

        // DSENT-ASSUMPTION: sum runs over ALL N entries -- unselected entries'
        // D-input toggles are charged though only the addressed entry's clock
        // gate latches. Intentional: why DFF RAM doesn't scale with depth.
        // Add write event
        Result* write_event = getEventResult("Write");
        write_event->addSubResult(nand2cg0->getEventResult("NAND2"), nand2cg0_name, 1.0);
        write_event->addSubResult(invcg0->getEventResult("INV"), invcg0_name, 1.0);
        write_event->addSubResult(dec->getEventResult("Decode"), dec_name, 1.0);
        for(unsigned int s = 0; s < enable_bufs.size(); ++s)
        {
            write_event->addSubResult(enable_bufs[s]->getEventResult("BUF"), "EnBuf_" + (String)s, 1.0);
        }
        for(unsigned int i = 0; i < number_entries; ++i)
        {
            write_event->addSubResult(nand2cg1s[i]->getEventResult("NAND2"), nand2cg1_names[i], 1.0);
            write_event->addSubResult(invcg1s[i]->getEventResult("INV"), invcg1_names[i], 1.0);
            write_event->addSubResult(dffs[i]->getEventResult("DFFD"), dff_names[i], number_bits);
            write_event->addSubResult(dffs[i]->getEventResult("DFFQ"), dff_names[i], number_bits);
            write_event->addSubResult(dffs[i]->getEventResult("CK"), dff_names[i], number_bits);
            for(unsigned int s = 0; s < row_bufs[i].size(); ++s)
            {
                write_event->addSubResult(row_bufs[i][s]->getEventResult("BUF"), "RowBuf" + (String)i + "_" + (String)s, 1.0);
            }
        }

        // Phase 4.1: the row-enable BUF chain and per-row CK buffers switch
        // every cycle regardless of read/write activity (a real clock-
        // distribution structure doesn't stop toggling just because this
        // cycle's WE happens to be low) -- add their cost to Idle too, not
        // only Write. Not added to Read: the read path only traverses the
        // Mux, it never touches the gated enable/CK distribution at all.
        // Mirrors the EnableTree event wiring it replaces exactly (same
        // Write+Idle, not Read, placement) -- see propagateTransitionInfo()
        // for the "always toggles" pass that gives this a nonzero value.
        Result* idle_event = getEventResult("Idle");
        for(unsigned int s = 0; s < enable_bufs.size(); ++s)
        {
            idle_event->addSubResult(enable_bufs[s]->getEventResult("BUF"), "EnBuf_" + (String)s, 1.0);
        }
        for(unsigned int i = 0; i < number_entries; ++i)
        {
            for(unsigned int s = 0; s < row_bufs[i].size(); ++s)
            {
                idle_event->addSubResult(row_bufs[i][s]->getEventResult("BUF"), "RowBuf" + (String)i + "_" + (String)s, 1.0);
            }
        }

        // DSENT-ASSUMPTION: the per-entry DFFQ contribution is commented out
        // below -- reading a stable, already-registered flop output is
        // modeled as costing only the Mux traversal, no DFF-side energy.
        // Add read event
        Result* read_event = getEventResult("Read");
        //for(unsigned int i = 0; i < number_entries; ++i)
        //{
        //    read_event->addSubResult(dffs[i]->getEventResult("DFFQ"), dff_names[i], number_bits);
        //}
        read_event->addSubResult(mux->getEventResult("Mux"), mux_name, number_bits);

        return;
    }

    void DFFRAM::updateModel()
    {
        unsigned int number_entries = getParameter("NumberEntries").toUInt();
        unsigned int number_bits = getParameter("NumberBits").toUInt();
        unsigned int row_clock_stages_param = getParameter("RowClockStages").toUInt();
        double row_clock_taper = getParameter("RowClockTaper").toDouble();
        unsigned int enable_stages_param = getParameter("EnableStages").toUInt();
        double enable_taper = getParameter("EnableTaper").toDouble();

        // Overriding updateModel() replaces the base class's default
        // behavior (iterate every sub-instance and update() it) -- so every
        // sub-instance needs an explicit update() here.
        getSubInstance("Dec")->update();
        getSubInstance("Mux")->update();
        getSubInstance("NAND2_CKGate0")->update();
        getSubInstance("INV_CKGate0")->update();

        // Phase 4.2: size the EnBuf and RowBuf chains (sizeBufChain(),
        // above) against their real downstream load. The load computation
        // (not just the sizing) has to happen here too, not only in
        // constructModel(), because it must re-run whenever N or B
        // changes -- update() (unlike construct()) is exactly the hook
        // that re-runs on every parameter change. (constructModel() also
        // now computes this same load, to bound the stage count it builds
        // -- see its own comment for why reading the cache there is safe.)
        {
            Map<double>* cache = getTechModel()->getStdCellLib()->getStdCellCache();
            vector<double> sizes;
            loadStdCellSizes(getTechModel(), sizes);

            // Enable-fanout load: the total gate capacitance the enable
            // signal actually drives, i.e. the A pin (not B -- B is
            // decoder-driven) of every NAND2_CKGate1_i. NAND2_CKGate1_i is
            // always minimum size, so this is N times one fixed, cached
            // constant -- not a per-instance sum requiring N updated
            // sub-instances.
            double enable_load = computeEnableLoad(number_entries, cache, sizes);
            // Row-fanout load: the row's real downstream capacitance is B
            // copies of the DFF's own CK pin cap (per the "not a bug"
            // clarification -- dffs[i] is one object standing for all B
            // flops of that row, and the per-bit portConnect loop is what
            // puts B x clock-pin capacitance into the graph). DFFQ is
            // always minimum size too.
            double row_load = computeRowLoad(number_bits, cache, sizes);

            // Part 4.2: stage counts must be computed from the SAME
            // load/cache/sizes as constructModel() used, so they match the
            // number of sub-instances actually built there.
            unsigned int n_enable_stages = computeChainStages(number_entries, enable_stages_param, enable_taper,
                enable_load, cache, sizes);
            unsigned int n_buf_stages = computeChainStages(number_bits, row_clock_stages_param, row_clock_taper,
                row_load, cache, sizes);

            if(n_enable_stages > 0)
            {
                vector<ElectricalModel*> enable_stage_models(n_enable_stages, (ElectricalModel*)NULL);
                for(unsigned int s = 0; s < n_enable_stages; ++s)
                {
                    enable_stage_models[s] = (ElectricalModel*)getSubInstance("EnBuf_" + (String)s);
                }
                sizeBufChain(enable_stage_models, enable_load, enable_taper, cache, sizes);
            }

            if(n_buf_stages > 0)
            {
                for(unsigned int i = 0; i < number_entries; ++i)
                {
                    vector<ElectricalModel*> row_stage_models(n_buf_stages, (ElectricalModel*)NULL);
                    for(unsigned int s = 0; s < n_buf_stages; ++s)
                    {
                        row_stage_models[s] = (ElectricalModel*)getSubInstance("RowBuf" + (String)i + "_" + (String)s);
                    }
                    sizeBufChain(row_stage_models, row_load, row_clock_taper, cache, sizes);
                }
            }
        }

        // The load each chain actually sees, and every fixed-size
        // sub-instance's own cached values, are unaffected by the sizing
        // above -- update them in the same simple loop as before.
        for(unsigned int i = 0; i < number_entries; ++i)
        {
            getSubInstance("DFF_" + (String)i)->update();
            getSubInstance("NAND2_CKGate1_" + (String)i)->update();
            getSubInstance("INV_CKGate1_" + (String)i)->update();
        }

        return;
    }

    void DFFRAM::propagateTransitionInfo()
    {
        // Update probability
        unsigned int number_entries = (unsigned int)getParameter("NumberEntries");
        unsigned int number_bits = (unsigned int)getParameter("NumberBits");
        unsigned int number_addr_bits = (unsigned int)ceil(log2(number_entries));
        unsigned int row_clock_stages_param = getParameter("RowClockStages").toUInt();
        double row_clock_taper = getParameter("RowClockTaper").toDouble();
        unsigned int enable_stages_param = getParameter("EnableStages").toUInt();
        double enable_taper = getParameter("EnableTaper").toDouble();
        // Part 4.2: must match constructModel()'s stage counts exactly --
        // both are only used here to iterate over sub-instances that
        // constructModel() actually built, so they need the same
        // load/cache/sizes inputs constructModel() used (see its comment
        // for why reading the cache here, before any sub-instance is
        // necessarily update()'d, is safe).
        Map<double>* prop_cache = getTechModel()->getStdCellLib()->getStdCellCache();
        vector<double> prop_sizes;
        loadStdCellSizes(getTechModel(), prop_sizes);
        double prop_enable_load = computeEnableLoad(number_entries, prop_cache, prop_sizes);
        double prop_row_load = computeRowLoad(number_bits, prop_cache, prop_sizes);
        unsigned int n_buf_stages = computeChainStages(number_bits, row_clock_stages_param, row_clock_taper,
            prop_row_load, prop_cache, prop_sizes);
        unsigned int n_enable_stages = computeChainStages(number_entries, enable_stages_param, enable_taper,
            prop_enable_load, prop_cache, prop_sizes);

        // Update decoder
        ElectricalModel* dec = (ElectricalModel*)getSubInstance("Dec");
        for(unsigned int i = 0; i < number_addr_bits; ++i)
        {
            propagatePortTransitionInfo(dec, "Addr" + (String)i, "WRAddr" + (String)i);
        }
        dec->use();

        // Update CKGate0 nands + invs
        ElectricalModel* nand2cg0 = (ElectricalModel*)getSubInstance("NAND2_CKGate0");
        propagatePortTransitionInfo(nand2cg0, "A", "CK");
        propagatePortTransitionInfo(nand2cg0, "B", "WE");
        nand2cg0->use();
        ElectricalModel* invcg0 = (ElectricalModel*)getSubInstance("INV_CKGate0");
        propagatePortTransitionInfo(invcg0, "A", nand2cg0, "Y");
        invcg0->use();

        // Phase 4.1 fix (a): row-enable BUF chain -- real propagation,
        // feeding the NAND2_CKGate1_i array and the timing graph correctly.
        // Downstream of the WE gate, so (like invcg0 before it) it's
        // near-static during Read/Idle here -- the "always toggles" energy
        // assumption for its own BUF event is handled separately below.
        vector<ElectricalModel*> enable_bufs(n_enable_stages, (ElectricalModel*)NULL);
        ElectricalModel* enable_prev = invcg0;
        for(unsigned int s = 0; s < n_enable_stages; ++s)
        {
            enable_bufs[s] = (ElectricalModel*)getSubInstance("EnBuf_" + (String)s);
            propagatePortTransitionInfo(enable_bufs[s], "A", enable_prev, "Y");
            enable_bufs[s]->use();
            enable_prev = enable_bufs[s];
        }
        ElectricalModel* enable_ck_final_model = enable_prev;    // invcg0 if EnableTaper<=0

        // Update CKGate1 nands + invs (A now sourced from the enable
        // chain's last stage, not directly from invcg0 -- fix (a))
        vector<ElectricalModel*> nand2cg1s(number_entries, NULL);
        vector<ElectricalModel*> invcg1s(number_entries, NULL);
        vector<vector<ElectricalModel*> > row_bufs(number_entries, vector<ElectricalModel*>(n_buf_stages, (ElectricalModel*)NULL));
        vector<ElectricalModel*> row_ck_final(number_entries, (ElectricalModel*)NULL);
        for(unsigned int i = 0; i < number_entries; ++i)
        {
            nand2cg1s[i] = (ElectricalModel*)getSubInstance("NAND2_CKGate1_" + (String)i);
            propagatePortTransitionInfo(nand2cg1s[i], "A", enable_ck_final_model, "Y");
            propagatePortTransitionInfo(nand2cg1s[i], "B", dec, "Out" + (String)i);
            nand2cg1s[i]->use();

            invcg1s[i] = (ElectricalModel*)getSubInstance("INV_CKGate1_" + (String)i);
            propagatePortTransitionInfo(invcg1s[i], "A", nand2cg1s[i], "Y");
            invcg1s[i]->use();

            // Phase 4 fix (b): per-row BUF chain -- real propagation.
            ElectricalModel* prev = invcg1s[i];
            for(unsigned int s = 0; s < n_buf_stages; ++s)
            {
                row_bufs[i][s] = (ElectricalModel*)getSubInstance("RowBuf" + (String)i + "_" + (String)s);
                propagatePortTransitionInfo(row_bufs[i][s], "A", prev, "Y");
                row_bufs[i][s]->use();
                prev = row_bufs[i][s];
            }
            row_ck_final[i] = prev;    // invcg1s[i] if fix (b) is disabled
        }

        // Update DFF
        vector<ElectricalModel*> dffs(number_entries, NULL);
        for(unsigned int i = 0; i < number_entries; ++i)
        {
            dffs[i] = (ElectricalModel*)getSubInstance("DFF_" + (String)i);
            propagatePortTransitionInfo(dffs[i], "D", "In");
            propagatePortTransitionInfo(dffs[i], "CK", row_ck_final[i], "Y");
            dffs[i]->use();
        }

        // Update Mux
        ElectricalModel* mux = (ElectricalModel*)getSubInstance("Mux");
        for(unsigned int i = 0; i < number_entries; ++i)
        {
            propagatePortTransitionInfo(mux, "In" + (String)i, dffs[i], "Q");
        }
        for(unsigned int i = 0; i < number_addr_bits; ++i)
        {
            propagatePortTransitionInfo(mux, "Sel" + (String)i, "RDAddr" + (String)i);
        }
        mux->use();

        // Set output probability
        getOutputPort("Out")->setTransitionInfo(mux->getOutputPort("Out")->getTransitionInfo());

        // ---- Phase 4.1: "always toggles" pass for BUF event reporting ----
        // The row-enable BUF chain and per-row buffers are modeled as
        // switching every cycle regardless of read/write activity (a real
        // clock distribution structure doesn't stop just because this
        // cycle's WE is low) -- but the real, WE-gated propagation above
        // (needed for correct downstream NAND2/DFF timing and energy) is
        // near-static whenever WE=0. This second pass runs AFTER every real
        // downstream consumer (NAND2_CKGate1_i, the DFF CK pins) has
        // already read (snapshotted -- TransitionInfo propagation copies by
        // value, it isn't a live reference) its input, so it only changes
        // what each enable-chain BUF's and each row BUF's own "BUF" event
        // Result reports, not anything downstream. Mirrors the EnableTree
        // "Send" always-toggle pass it replaces, applied per stage since
        // each stage is now its own BUF instance rather than one lumped
        // tree model.
        TransitionInfo always_toggle(0.0, 1.0, 0.0);
        for(unsigned int s = 0; s < n_enable_stages; ++s)
        {
            assignPortTransitionInfo(enable_bufs[s], "A", always_toggle);
            enable_bufs[s]->use();
        }
        for(unsigned int i = 0; i < number_entries; ++i)
        {
            for(unsigned int s = 0; s < n_buf_stages; ++s)
            {
                assignPortTransitionInfo(row_bufs[i][s], "A", always_toggle);
                row_bufs[i][s]->use();
            }
        }

        return;
    }
} // namespace DSENT


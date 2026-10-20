#include "model/electrical/router/RouterInputPort.h"

#include <cmath>
#include <vector>

#include "model/PortInfo.h"
#include "model/EventInfo.h"
#include "model/TransitionInfo.h"
#include "model/ModelGen.h"

namespace DSENT
{
    using std::ceil;
    using std::pow;
    using std::vector;
    using LibUtil::castStringVector;

    RouterInputPort::RouterInputPort(const String& instance_name_, const TechModel* tech_model_)
        : ElectricalModel(instance_name_, tech_model_)
    {
        initParameters();
        initProperties();
    }

    RouterInputPort::~RouterInputPort()
    {}

    void RouterInputPort::initParameters()
    {
        addParameterName("NumberVirtualNetworks");
        addParameterName("NumberVirtualChannelsPerVirtualNetwork");
        addParameterName("NumberBuffersPerVirtualChannel");
        addParameterName("NumberBitsPerFlit");
        addParameterName("BufferModel");
        // Phase 5: see the class-level comment in RouterInputPort.h.
        addParameterName("WriteRate", "1.0");
        addParameterName("ReadRate", "1.0");
        addParameterName("FlitToggleRate", "0.5");
        // Phase 6 (VC-select activity fix): separate read/write variants
        // accepted since it was trivial given WriteRate/ReadRate's existing
        // precedent -- see propagateTransitionInfo() for how each is used.
        addParameterName("WriteVCSwitchRate", "1.0");
        addParameterName("ReadVCSwitchRate", "1.0");
        return;
    }

    void RouterInputPort::initProperties()
    {
        return;
    }

    RouterInputPort* RouterInputPort::clone() const
    {
        // TODO 
        return NULL;
    }

    void RouterInputPort::constructModel()
    {
        // Get parameters
        unsigned int number_vns = getParameter("NumberVirtualNetworks").toUInt();
        const vector<unsigned int>& number_vcs_per_vn_vector = castStringVector<unsigned int>(getParameter("NumberVirtualChannelsPerVirtualNetwork").split("[,]"));
        const vector<unsigned int>& number_bufs_per_vc_vector = castStringVector<unsigned int>(getParameter("NumberBuffersPerVirtualChannel").split("[,]"));
        unsigned int number_bits_per_flit = getParameter("NumberBitsPerFlit").toUInt();
        const String& buffer_model = getParameter("BufferModel");
        double write_rate = getParameter("WriteRate").toDouble();
        double read_rate = getParameter("ReadRate").toDouble();
        double flit_toggle_rate = getParameter("FlitToggleRate").toDouble();
        double write_vc_switch_rate = getParameter("WriteVCSwitchRate").toDouble();
        double read_vc_switch_rate = getParameter("ReadVCSwitchRate").toDouble();

        ASSERT((write_rate >= 0.0) && (write_rate <= 1.0), "[Error] " + getInstanceName() +
            " -> WriteRate must be between 0.0 and 1.0!");
        ASSERT((read_rate >= 0.0) && (read_rate <= 1.0), "[Error] " + getInstanceName() +
            " -> ReadRate must be between 0.0 and 1.0!");
        ASSERT((flit_toggle_rate >= 0.0) && (flit_toggle_rate <= 1.0), "[Error] " + getInstanceName() +
            " -> FlitToggleRate must be between 0.0 and 1.0!");
        ASSERT((write_vc_switch_rate >= 0.0) && (write_vc_switch_rate <= 1.0), "[Error] " + getInstanceName() +
            " -> WriteVCSwitchRate must be between 0.0 and 1.0!");
        ASSERT((read_vc_switch_rate >= 0.0) && (read_vc_switch_rate <= 1.0), "[Error] " + getInstanceName() +
            " -> ReadVCSwitchRate must be between 0.0 and 1.0!");

        ASSERT(number_vns > 0, "[Error] " + getInstanceName() + 
            " -> Number of virtual networks must be > 0!");
        ASSERT(number_vcs_per_vn_vector.size() == number_vns, "[Error] " + getInstanceName() + 
            " -> Expecting " + (String)number_vns + " number of vcs, got " + 
            getParameter("NumberVirtualChannelsPerVirtualNetwork"));
        for(unsigned int i = 0; i < number_vns; ++i)
        {
            ASSERT(number_vcs_per_vn_vector[i] > 0, "[Error] " + getInstanceName() + 
                " -> Number of virtual channels per virtual network must be > 0!");
        }
        ASSERT(number_bufs_per_vc_vector.size() == number_vns, "[Error] " + getInstanceName() + 
            " -> Expecting " + (String)number_vns + " number of bufs per vc, got " + 
            getParameter("NumberBuffersPerVirtualChannel"));
        for(unsigned int i = 0; i < number_vns; ++i)
        {
            ASSERT(number_bufs_per_vc_vector[i] > 0, "[Error] " + getInstanceName() + 
                " -> Number of buffers per virtual channel must be > 0!");
        }
        ASSERT(number_bits_per_flit > 0, "[Error] " + getInstanceName() + 
            " -> Number of bits per buffer must be > 0!");

        // Calculate total number of buffers needed in the RAM
        unsigned int total_number_vcs = 0;
        unsigned int total_number_bufs = 0;
        for(unsigned int i = 0; i < number_vns; ++i)
        {
            total_number_vcs += number_vcs_per_vn_vector[i];
            total_number_bufs += number_vcs_per_vn_vector[i] * number_bufs_per_vc_vector[i];
        }
        unsigned int number_addr_bits = (unsigned int)ceil(log2(total_number_bufs));

        // Precondition: a uniform, power-of-two per-VC slice depth (Becker
        // Sec 6.3 / Table 6.1). Only then is the RAM address a clean
        // {vc_id, idx} concatenation with a constant per-VC base -- if slice
        // depths differed per VN the bases would be unaligned and each VC
        // would need its own adder to compute a non-aligned base (roughly
        // doubling the combinational cost of the address path), which is
        // not implemented here.
        unsigned int slice_depth = number_bufs_per_vc_vector[0];
        for(unsigned int i = 1; i < number_vns; ++i)
        {
            ASSERT(number_bufs_per_vc_vector[i] == slice_depth, "[Error] " + getInstanceName() +
                " -> NumberBuffersPerVirtualChannel must be identical across all virtual "
                "networks so every VC's FIFOControl slice has the same, aligned depth -- "
                "got " + getParameter("NumberBuffersPerVirtualChannel"));
        }
        ASSERT((slice_depth & (slice_depth - 1)) == 0, "[Error] " + getInstanceName() +
            " -> NumberBuffersPerVirtualChannel (" + (String)slice_depth + ") must be a "
            "power of two -- the RAM address concatenation {vc_id, idx} only works cleanly "
            "when each VC's index wraps naturally at the top of its bit width.");

        unsigned int idx_bits = (unsigned int)ceil(log2((double)slice_depth));

        getGenProperties()->set("TotalNumberVirtualChannels", total_number_vcs);
        getGenProperties()->set("TotalNumberBuffers", total_number_bufs);
        getGenProperties()->set("NumberAddressBits", number_addr_bits);
        getGenProperties()->set("SliceIndexBits", idx_bits);
        getGenProperties()->set("NumberOutputs", 1);

        createInputPort("CK");
        createInputPort("Push");
        createInputPort("Pop");
        createInputPort("FlitIn", makeNetIndex(0, number_bits_per_flit-1));
        createOutputPort("FlitOut", makeNetIndex(0, number_bits_per_flit-1));

        // Create energy, power, and area results
        createElectricalResults();
        getEventInfo("Idle")->setStaticTransitionInfos();
        getEventInfo("Idle")->setTransitionInfo("CK", TransitionInfo(0.0, 1.0, 0.0));

        createElectricalEventResult("WriteBuffer");
        getEventInfo("WriteBuffer")->setTransitionInfo("CK", TransitionInfo(0.0, 1.0, 0.0));
        // Flit data activity on the write bus, driven by FlitToggleRate
        // (Phase 5 change (b)). At the default 0.5 this is numerically
        // identical to the old hardcoded (0.25,0.25,0.25) -- 0.5 IS the
        // historical uniform-random-data assumption, just expressed as a
        // rate instead of a literal.
        getEventInfo("WriteBuffer")->setTransitionInfo("FlitIn",
            TransitionInfo((1.0 - flit_toggle_rate) / 2.0, flit_toggle_rate / 2.0, (1.0 - flit_toggle_rate) / 2.0));
        // Push's assertion probability, scaled by WriteRate (Phase 5 change
        // (a)) -- this is what actually reaches RAM's WE via
        // propagatePortTransitionInfo(ram, "WE", "Push") in
        // propagateTransitionInfo(). Mirrors OutputVCState's Grant-on-
        // CreditDecrement scaling by Utilization exactly.
        // NOTE: even at WriteRate=1.0 this is NOT the same as the old
        // TransitionInfo(0.0,0.0,1.0) -- the old value meant "held high,
        // never toggling" (P(1)=1.0); the new formula means "toggles every
        // cycle" (P(1)=0.5 for any WriteRate>0). This is an intentional
        // consequence of the rate-driven model, not a bug -- expect write
        // energy (and, since WE's own P(1) feeds downstream gate leakage,
        // possibly leakage too) to move at WriteRate=1.0 relative to the
        // pre-Phase-5 baseline. See validation notes for the measured delta.
        getEventInfo("WriteBuffer")->setTransitionInfo("Push",
            TransitionInfo((1.0 - write_rate) / 2.0, write_rate / 2.0, (1.0 - write_rate) / 2.0));
        // Pop stays unscaled -- no pop happens during a pure write cycle;
        // mirrors CreditIn staying unscaled on CreditDecrement.
        getEventInfo("WriteBuffer")->setTransitionInfo("Pop", TransitionInfo(1.0, 0.0, 0.0));

        createElectricalEventResult("ReadBuffer");
        getEventInfo("ReadBuffer")->setTransitionInfo("CK", TransitionInfo(0.0, 1.0, 0.0));
        // Stored data is held during a read; data-change energy belongs to the
        // write event. Without this, FlitIn defaults to random and charges
        // every entry's D-input inverter into the read event, double-counting
        // against WriteBuffer (that load is already charged via q_load_cap in
        // DFFQ's own "DFFQ" event, summed into WriteBuffer).
        // This removes a real but small double-count: the dominant read-energy
        // term is select-driven mux tree switching (all N-1 MUX2 cells in the
        // broadcast-select tree toggle per read), not data activity -- see
        // e_read_linear_in_depth in the buffer_characteristics sweep tooling.
        getEventInfo("ReadBuffer")->setStaticTransitionInfo("FlitIn");   // (0.5, 0.0, 0.5)
        // Pop's assertion probability, scaled by ReadRate (Phase 5 change
        // (a), read-side symmetric with Push above) -- mirrors
        // OutputVCState's CreditIn-on-CreditIncrement scaling by
        // Utilization. Same NOTE as Push above applies: not identical to
        // the old (0.0,0.0,1.0) even at ReadRate=1.0.
        getEventInfo("ReadBuffer")->setTransitionInfo("Pop",
            TransitionInfo((1.0 - read_rate) / 2.0, read_rate / 2.0, (1.0 - read_rate) / 2.0));
        // Push stays unscaled -- mirrors Grant staying unscaled on CreditIncrement.
        getEventInfo("ReadBuffer")->setTransitionInfo("Push", TransitionInfo(1.0, 0.0, 0.0));

        // Init RAM. The data path is unchanged by the per-VC control
        // rework below: Becker Sec 6.3 notes that consolidating storage for
        // all VCs into one shared array (vs. a separate array per VC)
        // doesn't add write/read delay, since flits for every VC already
        // arrive through this same input register and leave through the
        // same crossbar port -- the shared array simply subsumes the
        // demux/mux a set of separate per-VC arrays would otherwise need.
        const String& ram_name = "RAM";
        ElectricalModel* ram = ModelGen::createRAM(buffer_model, ram_name, getTechModel());
        ram->setParameter("NumberEntries", total_number_bufs);
        ram->setParameter("NumberBits", number_bits_per_flit);
        ram->construct();

        // Per-VC control: one FIFOControl per VC, each owning that VC's own
        // read/write pointers and full/empty flag over its slice_depth-entry
        // slice of the shared RAM (replaces the old flat rd_addr_dffs +
        // placeholder-address design). All total_number_vcs instances are
        // built and clocked every cycle, so area/leakage scale correctly
        // with VC count (Table 6.1's static column). DSENT-ASSUMPTION: VC 0
        // is treated as "the selected VC" for structural address/energy
        // wiring below -- which VC is actually granted a cycle is a runtime
        // arbitration decision made elsewhere (RouterSwitchAllocator /
        // MatrixArbiter), outside this model's scope.
        vector<String> fifo_names(total_number_vcs, "");
        vector<ElectricalModel*> fifos(total_number_vcs, (ElectricalModel*)NULL);
        for(unsigned int vc = 0; vc < total_number_vcs; ++vc)
        {
            fifo_names[vc] = "FIFO" + (String)vc;
            fifos[vc] = (ElectricalModel*)ModelGen::createModel("FIFOControl", fifo_names[vc], getTechModel());
            fifos[vc]->setParameter("NumberEntries", slice_depth);
            // Phase 5: reuse the same WriteRate/ReadRate driving this
            // port's own Push/Pop EventInfo (above) for FIFOControl's
            // index-register activity, so both sides agree.
            fifos[vc]->setParameter("WriteRate", (String)write_rate);
            fifos[vc]->setParameter("ReadRate", (String)read_rate);
            fifos[vc]->construct();
            portConnect(fifos[vc], "CK", "CK");
            portConnect(fifos[vc], "Push", "Push");
            portConnect(fifos[vc], "Pop", "Pop");
            addSubInstances(fifos[vc], 1.0);          // weight 1 -- NOT total_number_vcs
            addElectricalSubResults(fifos[vc], 1.0);
        }

        // RAM address = {vc_id, idx}. vc_id is a compile-time constant per
        // FIFOControl instance, so concatenating it onto the address is
        // pure bit wiring -- no cells, no adder. VC 0's id is all-zero, so
        // its upper (vc-select) address bits tie to a constant-0 net; its
        // low idx_bits come directly from FIFO0's own WRIdx/RDIdx.
        createNet("VCAddr_WR", makeNetIndex(0, idx_bits - 1));
        createNet("VCAddr_RD", makeNetIndex(0, idx_bits - 1));
        portConnect(fifos[0], "WRIdx", "VCAddr_WR", makeNetIndex(0, idx_bits - 1));
        portConnect(fifos[0], "RDIdx", "VCAddr_RD", makeNetIndex(0, idx_bits - 1));
        createNet("GndTie");

        // Connect RAM. WE now comes from the real Push signal (replacing
        // the old placeholder tied to the flit's top bit); WRAddr/RDAddr
        // come from FIFOControl instead of the old flit-low-bits /
        // rd_addr_dffs placeholders.
        portConnect(ram, "In", "FlitIn");
        portConnect(ram, "WE", "Push");
        for(unsigned int b = 0; b < number_addr_bits; ++b)
        {
            if(b < idx_bits)
            {
                portConnect(ram, "WRAddr" + (String)b, "VCAddr_WR", makeNetIndex(b));
                portConnect(ram, "RDAddr" + (String)b, "VCAddr_RD", makeNetIndex(b));
            }
            else
            {
                portConnect(ram, "WRAddr" + (String)b, "GndTie");
                portConnect(ram, "RDAddr" + (String)b, "GndTie");
            }
        }
        portConnect(ram, "CK", "CK");
        portConnect(ram, "Out", "FlitOut");

        addSubInstances(ram, 1.0);
        addElectricalSubResults(ram, 1.0);

        // ---- Event composition -------------------------------------------
        // Only ONE VC is written, and (separately) only ONE VC is read, per
        // cycle -- do NOT loop over every VC adding a Push (or Pop)
        // sub-result, that would overcount by total_number_vcs. Every OTHER
        // FIFOControl's flops are still clocked every cycle regardless of
        // which VC is active, so their Idle-event cost is real; it's
        // included via one representative (structurally identical)
        // instance weighted by (total_number_vcs - 1), rather than calling
        // use()/addSubResult individually on every other VC.
        getEventResult("WriteBuffer")->addSubResult(ram->getEventResult("Write"), ram_name, 1.0);
        getEventResult("WriteBuffer")->addSubResult(fifos[0]->getEventResult("Push"), fifo_names[0], 1.0);
        if(total_number_vcs > 1)
        {
            getEventResult("WriteBuffer")->addSubResult(fifos[1]->getEventResult("Idle"), "FIFO_idle_rep", (double)(total_number_vcs - 1));
        }

        getEventResult("ReadBuffer")->addSubResult(ram->getEventResult("Read"), ram_name, 1.0);
        getEventResult("ReadBuffer")->addSubResult(fifos[0]->getEventResult("Pop"), fifo_names[0], 1.0);
        if(total_number_vcs > 1)
        {
            getEventResult("ReadBuffer")->addSubResult(fifos[1]->getEventResult("Idle"), "FIFO_idle_rep", (double)(total_number_vcs - 1));
        }

        return;
    }

    void RouterInputPort::propagateTransitionInfo()
    {
        unsigned int number_addr_bits = getGenProperties()->get("NumberAddressBits").toUInt();
        unsigned int idx_bits = getGenProperties()->get("SliceIndexBits").toUInt();
        unsigned int total_number_vcs = getGenProperties()->get("TotalNumberVirtualChannels").toUInt();
        const String& current_event = getGenProperties()->get("UseModelEvent");
        double write_vc_switch_rate = getParameter("WriteVCSwitchRate").toDouble();
        double read_vc_switch_rate = getParameter("ReadVCSwitchRate").toDouble();

        ElectricalModel* fifo0 = (ElectricalModel*)getSubInstance("FIFO0");

        // Trigger FIFO0's own named event so it applies ITS OWN Push/Pop/CK
        // EventInfo (counter-aware WRIdx/RDIdx activity, see FIFOControl.cc)
        // rather than whatever this model's top-level Push/Pop ports
        // happen to be propagated to -- this is what actually produces the
        // "1 x fifo.Push" / "1 x fifo.Pop" terms wired in constructModel().
        if(current_event == "WriteBuffer")
        {
            fifo0->use("Push");
        }
        else if(current_event == "ReadBuffer")
        {
            fifo0->use("Pop");
        }
        else
        {
            fifo0->use("Idle");
        }

        // Every other VC's FIFOControl is clocked every cycle regardless of
        // which VC is active -- one representative instance stands in for
        // all (total_number_vcs - 1) of them, matching the weighted
        // addSubResult wiring in constructModel().
        if(total_number_vcs > 1)
        {
            ElectricalModel* fifo_rep = (ElectricalModel*)getSubInstance("FIFO1");
            fifo_rep->use("Idle");
        }

        ElectricalModel* ram = (ElectricalModel*)getSubInstance("RAM");

        if(current_event != "Idle")
        {
            propagatePortTransitionInfo(ram, "In", "FlitIn");
            propagatePortTransitionInfo(ram, "CK", "CK");
            propagatePortTransitionInfo(ram, "WE", "Push");
            // VC-select activity fix (Phase 6): VC 0 remains the structural
            // "selected VC" (see constructModel() -- capacitance and gate
            // counts are untouched, only the activity assigned to the
            // address bus changes here).
            //
            // Between consecutive accesses either (a) the same VC is
            // selected again (a body flit of the same packet): the index
            // advanced by one and the Phase 1 counter-aware profile
            // applies, while the vc_id bits hold; or (b) a different VC is
            // selected: the index jumps to another VC's own pointer value
            // -- statistically a random jump, ~0.5 toggle per bit -- and
            // the vc_id bits also toggle at ~0.5, since VC selection isn't
            // a monotonic counter (unlike the index, every vc_id bit is
            // equally likely to flip, no bit-position decay).
            // WriteVCSwitchRate/ReadVCSwitchRate (p below) blend the two
            // cases. p=0 reproduces the old (incorrect) always-VC-0
            // behaviour; p=1 reproduces the flat 0.25/0.25/0.25 default
            // (maximum uncertainty -- a fresh VC's pointer looks uniformly
            // random from here).
            //
            // Physically, p relates to packet length: a packet occupies one
            // VC for its whole duration, so the selected VC changes roughly
            // at packet boundaries -- p ~= ((V-1)/V) / avg_packet_flits is a
            // reasonable estimate for a caller supplying this rate.
            //
            // This replaces the previous approach of propagating FIFO0's own
            // aggregate "WRIdx"/"RDIdx" port (itself only ever set from
            // WRIdx_DFF0/RDIdx_DFF0, the LSB) flatly across every index
            // address bit. That was TWO separate pre-existing artifacts, not
            // one:
            //   (1) vc_id bits pinned constant, because VC 0 is hardwired as
            //       the structurally selected VC -- the artifact this fix
            //       was commissioned for.
            //   (2) every index address bit charged at the LSB's toggle rate
            //       (0.5) instead of its own 1/2^(i+1), because DFFRAM's
            //       per-bit address ports (WRAddr0, WRAddr1, ...) were fed
            //       one shared aggregate value instead of being assigned
            //       individually -- they were always individually
            //       addressable, the old code just didn't use that.
            // Artifact (2) OVERCHARGED every upper index bit, so correcting
            // it lowers e_read/e_write relative to the old numbers -- this
            // is a correction, not a regression. It is also why the closed-
            // form formula's p=0 case does NOT reproduce the old numbers
            // except at idx_bits=1 (no upper bits exist to have been
            // overcharged there -- see VALIDATION item 2 in
            // vc_select_fix_findings.json for the measured per-V deltas).
            //
            // On the vc_id bits' static level at p=0: TransitionInfo(0.5, 0,
            // 0.5) (P(1)=0.5, zero toggles) is the physically correct
            // representation, not merely "zero toggles like the old value."
            // VC ids are roughly uniform over V, so averaged over which VC
            // ends up structurally selected, each vc_id bit sits high half
            // the time. The old TransitionInfo(1.0, 0, 0) (P(1)=0) was an
            // artifact of hardwiring VC 0 specifically, not a physical
            // property of the address bus -- so it is intentionally NOT
            // reproduced at p=0 either.
            //
            // FIFOControl's own internal WRIdx_DFF/RDIdx_DFF activity (its
            // Push/Pop D-input formula, scaled by WriteRate/ReadRate) is
            // untouched by this -- per scope, the blend below applies only
            // to the address bus presented to the shared RAM, not to
            // FIFOControl's own pointer registers, which genuinely only
            // ever increment.
            for(unsigned int b = 0; b < number_addr_bits; ++b)
            {
                double q_wr, q_rd;
                if(b < idx_bits)
                {
                    double index_rate = 1.0 / pow(2.0, (double)(b + 1));
                    q_wr = (1.0 - write_vc_switch_rate) * index_rate + write_vc_switch_rate * 0.5;
                    q_rd = (1.0 - read_vc_switch_rate) * index_rate + read_vc_switch_rate * 0.5;
                }
                else
                {
                    q_wr = write_vc_switch_rate * 0.5;
                    q_rd = read_vc_switch_rate * 0.5;
                }
                assignPortTransitionInfo(ram, "WRAddr" + (String)b,
                    TransitionInfo((1.0 - q_wr) / 2.0, q_wr / 2.0, (1.0 - q_wr) / 2.0));
                assignPortTransitionInfo(ram, "RDAddr" + (String)b,
                    TransitionInfo((1.0 - q_rd) / 2.0, q_rd / 2.0, (1.0 - q_rd) / 2.0));
            }
        }
        ram->use();
        // Set output probability
        propagatePortTransitionInfo("FlitOut", ram, "Out");
        return;
    }
} // namespace DSENT


#include "model/electrical/Multiplexer.h"

#include <cmath>

#include "model/PortInfo.h"
#include "model/TransitionInfo.h"
#include "model/EventInfo.h"
#include "model/timing_graph/ElectricalDriverMultiplier.h"
#include "model/timing_graph/ElectricalNet.h"
#include "model/std_cells/StdCell.h"
#include "model/std_cells/StdCellLib.h"

namespace DSENT
{
    namespace
    {
        // How many leaf MUX2 "S0" pins, anywhere in an m-input multiplexer
        // subtree, are driven by LOCAL select bit `bit` (0 = LSB, matching
        // the same bit numbering constructModel()/propagateTransitionInfo()
        // already use when recursing into mux0/mux1). Pure structural
        // recursion -- mirrors constructModel()'s own inputs_0/inputs_1/
        // selects_0/selects_1 splitting exactly, just counting instead of
        // building. m<=1 needs no select at all. At the level where `bit`
        // is that subtree's OWN top bit (selects-1), exactly one MUX2 (that
        // level's own muxf) uses it; otherwise it's passed through
        // unbuffered to both halves (the defect this fix addresses), so
        // it's used by however many leaves use it in EACH half, summed.
        unsigned int countSelectBitFanout(unsigned int m, unsigned int bit)
        {
            if(m <= 1)
            {
                return 0;
            }
            unsigned int selects = (unsigned int)ceil(log2((double)m));
            if(bit >= selects)
            {
                return 0;
            }
            if(bit == selects - 1)
            {
                return 1;
            }
            unsigned int inputs_0 = (unsigned int)ceil((double)m / 2.0);
            unsigned int inputs_1 = (unsigned int)floor((double)m / 2.0);
            return countSelectBitFanout(inputs_0, bit) + countSelectBitFanout(inputs_1, bit);
        }

        // Taper a select-bit buffer chain toward its real fanout, working
        // backward from the last stage -- identical approach to DFFRAM's
        // own sizeBufChain() (docs/enable_buf_taper_fix_findings.json):
        // per stage, target = (load this stage drives) / taper, then pick
        // the LARGEST available drive strength whose own input cap does
        // not exceed that target (not the smallest that's <= target, which
        // trivially always the minimum size and produces no taper at all).
        void sizeSelectBufChain(const vector<ElectricalModel*>& stages_, double final_load_,
            double taper_, Map<double>* cache_, const vector<double>& sizes_)
        {
            if(stages_.empty() || (taper_ <= 0.0) || sizes_.empty())
            {
                return;
            }
            double target = final_load_;
            for(int s = (int)stages_.size() - 1; s >= 0; --s)
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
                stages_[s]->setDrivingStrengthIdx(chosen_idx);
                target = chosen_cap;
            }
            return;
        }
    } // anonymous namespace

    Multiplexer::Multiplexer(const String& instance_name_, const TechModel* tech_model_)
        : ElectricalModel(instance_name_, tech_model_)
    {
        initParameters();
        initProperties();
    }

    Multiplexer::~Multiplexer()
    {}

    void Multiplexer::initParameters()
    {
        addParameterName("NumberInputs");
        addParameterName("NumberBits");
        addParameterName("BitDuplicate", "TRUE");
        addParameterName("IsTopLevel", "TRUE");
        // Fix for the same untapered-fanout defect the DFFRAM enable/row
        // clock chains had (docs/enable_buf_taper_fix_findings.json): the
        // top-level select bits used to fan out, completely unbuffered
        // below this level, to every MUX2 "S0" pin that shares that bit --
        // up to NumberInputs/2 of them for the LSB. Each select bit now
        // gets its own tapered BUF chain sized from ITS OWN real fanout
        // (countSelectBitFanout(), above), since different bits are used
        // by very different numbers of leaves (the MSB by just 1, the LSB
        // by up to half of NumberInputs).
        // 0 = auto: ceil(log(fanout)/log(SelectBufferTaper)), at least 1.
        addParameterName("SelectBufferStages", "0");
        addParameterName("SelectBufferTaper", "4");
        return;
    }

    void Multiplexer::initProperties()
    {
        return;
    }

    Multiplexer* Multiplexer::clone() const
    {
        return NULL;
    }

    void Multiplexer::constructModel()
    {
        // Get parameters
        unsigned int number_bits = (unsigned int) getParameter("NumberBits");
        unsigned int number_inputs = (unsigned int) getParameter("NumberInputs");
        unsigned int number_selects = (unsigned int) ceil(log2((double) number_inputs));
        bool bit_duplicate = (bool) getParameter("BitDuplicate");
        bool is_top_level = getParameter("IsTopLevel").toBool();
        
        ASSERT(number_inputs > 0, "[Error] " + getInstanceName() + " -> Number of inputs must be > 0!");
        ASSERT(number_bits > 0, "[Error] " + getInstanceName() + " -> Number of bits must be > 0!");
    
        //Construct electrical ports and nets
        //Create each input port
        for(unsigned int i = 0; i < number_inputs; ++i)
            createInputPort(    "In" + (String) i, makeNetIndex(0, number_bits-1));
        //Create select signals
        for(unsigned int i = 0; i < number_selects; ++i)
        {
            createInputPort(    "Sel" + (String)i);
        }
        //Create output
        createOutputPort(   "Out", makeNetIndex(0, number_bits-1));
        
        //Create energy, power, and area results
        createElectricalResults();
        getEventInfo("Idle")->setStaticTransitionInfos();
        createElectricalEventResult("Mux");
                    
        //Number of inputs on the 0 side
        unsigned int inputs_0 = (unsigned int) ceil((double) number_inputs / 2.0);
        unsigned int selects_0 = (unsigned int) ceil(log2((double) inputs_0));
        //Number of inputs on the 1 side
        unsigned int inputs_1 = (unsigned int) floor((double) number_inputs / 2.0);
        unsigned int selects_1 = (unsigned int) ceil(log2((double) inputs_1));
        
        //Depending on whether we want to create a 1-bit instance and have it multiplied
        //up by number of bits or actually instantiate number_bits of 1-bit instances.
        //Recursively instantiates smaller multiplexers
        if (bit_duplicate || number_bits == 1)
        {
            //If it is just a 1-input multiplexer, just connect output to input and be done
            if (number_inputs == 1)
            {
                assign("Out", "In0");
            }
            else
            {
                //If it is more than 1 input, instantiate two sub multiplexers (Mux_way0 and Mux_way1)
                //and create a final 2:1 mux (muxf) to select between them
                String mux0_name = "Mux_way0";
                String mux1_name = "Mux_way1";
                String muxf_name = "Mux2_i" + (String)number_inputs;

                Multiplexer* mux0 = new Multiplexer(mux0_name, getTechModel());
                mux0->setParameter("NumberInputs", inputs_0);
                mux0->setParameter("NumberBits", 1);
                mux0->setParameter("BitDuplicate", "TRUE");
                mux0->setParameter("IsTopLevel", "FALSE");
                mux0->construct();
            
                Multiplexer* mux1 = new Multiplexer(mux1_name, getTechModel());
                mux1->setParameter("NumberInputs", inputs_1);
                mux1->setParameter("NumberBits", 1);
                mux1->setParameter("BitDuplicate", "TRUE");                                    
                mux1->setParameter("IsTopLevel", "FALSE");
                mux1->construct();
                
                StdCell* muxf = getTechModel()->getStdCellLib()->createStdCell("MUX2", muxf_name);
                muxf->construct();

                // Select-bit buffer chains at the top level -- see the
                // comment on SelectBufferStages/SelectBufferTaper in
                // initParameters(). Each bit's own chain is a plain BUF
                // chain (not sized here -- min size from construct(), real
                // per-stage sizing happens in updateModel(), same pattern
                // as DFFRAM's EnBuf/RowBuf), with stage count derived from
                // THAT bit's own real fanout (countSelectBitFanout()),
                // since the LSB fans out to far more MUX2 leaves than the
                // MSB does. The final stage's own output net is still
                // named "SelBuf"+i, so nothing downstream of this block
                // (the mux0/mux1/muxf connections right below, and the
                // recursive pass-through at every level below this one)
                // needs to change at all.
                vector<unsigned int> select_n_stages(number_selects, 0);
                if(is_top_level)
                {
                    unsigned int select_buffer_stages_param = getParameter("SelectBufferStages").toUInt();
                    double select_buffer_taper = getParameter("SelectBufferTaper").toDouble();
                    for(unsigned int i = 0; i < number_selects; ++i)
                    {
                        unsigned int fanout_count = countSelectBitFanout(number_inputs, i);
                        unsigned int n_stages = (select_buffer_stages_param > 0)
                            ? select_buffer_stages_param
                            : std::max((unsigned int)1,
                                (unsigned int)ceil(log((double)std::max(fanout_count, (unsigned int)1))
                                    / log(select_buffer_taper)));
                        select_n_stages[i] = n_stages;

                        String sel_source = "Sel" + (String)i;
                        for(unsigned int s = 0; s < n_stages; ++s)
                        {
                            const String& buf_name = String::format("SelBuf%d_%d", i, s);
                            StdCell* buf = getTechModel()->getStdCellLib()->createStdCell("BUF", buf_name);
                            buf->construct();
                            addSubInstances(buf, 1.0);
                            addElectricalSubResults(buf, 1.0);
                            getEventResult("Mux")->addSubResult(buf->getEventResult("BUF"), buf_name, 1.0);

                            portConnect(buf, "A", sel_source);
                            const String& buf_out_net = (s == n_stages - 1)
                                ? ("SelBuf" + (String)i)
                                : String::format("SelBufOut%d_%d", i, s);
                            createNet(buf_out_net);
                            portConnect(buf, "Y", buf_out_net);
                            sel_source = buf_out_net;
                        }
                    }
                }

                //Create outputs of way0 and way1 multiplexers with final mux
                createNet("way0Out");
                createNet("way1Out");
                portConnect(mux0, "Out", "way0Out");
                portConnect(mux1, "Out", "way1Out");
                portConnect(muxf, "A", "way0Out");
                portConnect(muxf, "B", "way1Out");
                //Connect inputs to the sub multiplexers.
                //Note that multiple inputs are connected to the mux0 and mux1 input and the
                //selector signals are connected multiple times. This is just so that everything
                //is loaded appropriately since bit duplication is applied
                for (unsigned int n = 0; n < number_bits; ++n)
                {
                    //Connect inputs
                    for (unsigned int i = 0; i < inputs_0; ++i)
                        portConnect(mux0, "In" + (String) i, "In" + (String) i, makeNetIndex(n));
                    for (unsigned int i = 0; i < inputs_1; ++i)
                        portConnect(mux1, "In" + (String) i, "In" + (String) (i + inputs_0), makeNetIndex(n));                    
                    // TODO hack 
                    if(is_top_level)
                    {
                        //Connect selector bits
                        for (unsigned int i = 0; i < selects_0; ++i)
                            portConnect(mux0, "Sel" + (String)i, "SelBuf" + (String)i);
                        for (unsigned int i = 0; i < selects_1; ++i)
                            portConnect(mux1, "Sel" + (String)i, "SelBuf" + (String)i);
                        portConnect(muxf, "S0", "SelBuf" + (String)(number_selects - 1));
                    }
                    else
                    {
                        //Connect selector bits
                        for (unsigned int i = 0; i < selects_0; ++i)
                            portConnect(mux0, "Sel" + (String)i, "Sel" + (String)i);
                        for (unsigned int i = 0; i < selects_1; ++i)
                            portConnect(mux1, "Sel" + (String)i, "Sel" + (String)i);
                        portConnect(muxf, "S0", "Sel" + (String)(number_selects - 1));
                    }
                }

                //Connect final mux to outputs
                //Because we use bit duplication and so there is only only one multiplexer
                //instance, we must use driver multiplier to drive each output appropriately
                if (number_bits == 1)
                    portConnect(muxf, "Y", "Out");
                else
                {
                    createNet("OutTemp");
                    createDriverMultiplier("OutMult");
                    ElectricalDriverMultiplier* drive_mult = getDriverMultiplier("OutMult");
                    portConnect(muxf, "Y", "OutTemp");
                    getNet("OutTemp")->addDownstreamNode(drive_mult);
                    for (unsigned int n = 0; n < number_bits; ++n)
                        drive_mult->addDownstreamNode(getNet("Out", makeNetIndex(n)));
                }

                //Add area, power, and event results for each mux
                addSubInstances(mux0, number_bits);
                addElectricalSubResults(mux0, number_bits);
                addSubInstances(mux1, number_bits);
                addElectricalSubResults(mux1, number_bits);
                addSubInstances(muxf, number_bits);
                addElectricalSubResults(muxf, number_bits);
                getEventResult("Mux")->addSubResult(mux0->getEventResult("Mux"), mux0_name, number_bits);                
                getEventResult("Mux")->addSubResult(mux1->getEventResult("Mux"), mux1_name, number_bits);                
                getEventResult("Mux")->addSubResult(muxf->getEventResult("MUX2"), muxf_name, number_bits);                

            }

        }
        else
        {
            //Instantiate a bunch of 1-bit multiplexers
            for (unsigned int n = 0; n < number_bits; ++n)
            {
                String mux_name = "Mux_bit" + (String) n;

                Multiplexer* mux = new Multiplexer(mux_name, getTechModel());
                mux->setParameter("NumberInputs", number_inputs);
                mux->setParameter("NumberBits", 1);
                mux->setParameter("BitDuplicate", "TRUE");            
                mux->construct();

                // Connect inputs
                for (unsigned int i = 0; i < number_inputs; ++i)
                    portConnect(mux, "In" + (String) i, "In" + (String) i, makeNetIndex(n));
                for(unsigned int i = 0; i < number_selects; ++i)
                    portConnect(mux, "Sel" + (String)i, "Sel" + (String)i);
                portConnect(mux, "Out", "Out", makeNetIndex(n));

                //Add area, power, and event results for each mux
                addSubInstances(mux, 1.0);
                addElectricalSubResults(mux, 1.0);
                getEventResult("Mux")->addSubResult(mux->getEventResult("Mux"), mux_name, 1.0);
            }
        }

        return;
    }

    void Multiplexer::updateModel()
    {
        // Run the default walk first (updates mux0/mux1/muxf and every
        // SelBuf stage at whatever size it currently has -- construct()
        // leaves them all at minimum), then size the select-bit chains
        // against their real fanout. Not folded into constructModel()
        // because sizing must re-run whenever parameters change, same
        // reasoning as DFFRAM's EnBuf/RowBuf chains.
        Model::updateModel();

        bool is_top_level = getParameter("IsTopLevel").toBool();
        bool bit_duplicate = (bool)getParameter("BitDuplicate");
        unsigned int number_bits = (unsigned int)getParameter("NumberBits");
        if(!is_top_level || !(bit_duplicate || number_bits == 1))
        {
            return;
        }

        unsigned int number_inputs = (unsigned int)getParameter("NumberInputs");
        unsigned int number_selects = (unsigned int)ceil(log2((double)number_inputs));
        if(number_inputs <= 1)
        {
            return;
        }

        unsigned int select_buffer_stages_param = getParameter("SelectBufferStages").toUInt();
        double select_buffer_taper = getParameter("SelectBufferTaper").toDouble();

        Map<double>* cache = getTechModel()->getStdCellLib()->getStdCellCache();
        const vector<String>& size_strs = getTechModel()->get("StdCell->AvailableSizes").split("[,]");
        vector<double> sizes;
        for(unsigned int k = 0; k < size_strs.size(); ++k)
        {
            sizes.push_back(size_strs[k].toDouble());
        }
        const String& min_size_str = (String)sizes[0];

        // Every leaf MUX2's S0 pin is always minimum size (never resized
        // anywhere in this file), so its input cap is one fixed, cached
        // constant -- the real fanout load for bit i is just that times
        // however many leaves countSelectBitFanout() says share it.
        double mux2_min_s0_cap = cache->get("MUX2_X" + min_size_str + "->Cap->S0");

        for(unsigned int i = 0; i < number_selects; ++i)
        {
            unsigned int fanout_count = countSelectBitFanout(number_inputs, i);
            unsigned int n_stages = (select_buffer_stages_param > 0)
                ? select_buffer_stages_param
                : std::max((unsigned int)1,
                    (unsigned int)ceil(log((double)std::max(fanout_count, (unsigned int)1))
                        / log(select_buffer_taper)));

            vector<ElectricalModel*> stage_models(n_stages, (ElectricalModel*)NULL);
            for(unsigned int s = 0; s < n_stages; ++s)
            {
                stage_models[s] = (ElectricalModel*)getSubInstance(String::format("SelBuf%d_%d", i, s));
            }
            double final_load = (double)fanout_count * mux2_min_s0_cap;
            sizeSelectBufChain(stage_models, final_load, select_buffer_taper, cache, sizes);
        }

        return;
    }

    void Multiplexer::propagateTransitionInfo()
    {
        // The only thing can be updated are the input probabilities...so we will update them
        unsigned int number_bits = (unsigned int) getParameter("NumberBits");
        unsigned int number_inputs = (unsigned int) getParameter("NumberInputs");
        unsigned int number_selects = (unsigned int) ceil(log2((double) number_inputs));
        bool bit_duplicate = (bool) getParameter("BitDuplicate");
        bool is_top_level = getParameter("IsTopLevel").toBool();

        //Number of inputs on the 0 side
        unsigned int inputs_0 = (unsigned int) ceil((double) number_inputs / 2.0);
        unsigned int selects_0 = (unsigned int) ceil(log2((double) inputs_0));

        //Number of inputs on the 1 side
        unsigned int inputs_1 = (unsigned int) floor((double) number_inputs / 2.0);
        unsigned int selects_1 = (unsigned int) ceil(log2((double) inputs_1));

        if (bit_duplicate || number_bits == 1)
        {
            if (number_inputs == 1)
            {
                //If theres only 1 input, output transition = input transition
                propagatePortTransitionInfo("Out", "In0");
            }
            else
            { 
                // Update sub muxes with appropriate probabilities
                ElectricalModel* mux0 = (ElectricalModel*)getSubInstance("Mux_way0");
                for(unsigned int i = 0; i < inputs_0; ++i)
                {
                    propagatePortTransitionInfo(mux0, "In" + (String)i, "In" + (String)i);
                }
                for(unsigned int i = 0; i < selects_0; ++i)
                {
                    propagatePortTransitionInfo(mux0, "Sel" + (String)i, "Sel" + (String)i);
                }
                mux0->use();                
                ElectricalModel* mux1 = (ElectricalModel*)getSubInstance("Mux_way1");
                for(unsigned int i = 0; i < inputs_1; ++i)
                {
                    propagatePortTransitionInfo(mux1, "In" + (String)i, "In" + (String)(i + inputs_0));
                }
                for(unsigned int i = 0; i < selects_1; ++i)
                {
                    propagatePortTransitionInfo(mux1, "Sel" + (String)i, "Sel" + (String)i);
                }
                mux1->use();                
                ElectricalModel* muxf = (ElectricalModel*)getSubInstance("Mux2_i" + (String)number_inputs);
                propagatePortTransitionInfo(muxf, "A", mux0, "Out");
                propagatePortTransitionInfo(muxf, "B", mux1, "Out");
                propagatePortTransitionInfo(muxf, "S0", "Sel" + (String)(number_selects-1));
                muxf->use();

                // Select-bit buffer chains -- real propagation through
                // whatever stage count each bit's own chain was built with
                // (constructModel()'s own formula, recomputed identically
                // here since stage count is purely a function of
                // parameters, not of anything measured).
                if(is_top_level)
                {
                    unsigned int select_buffer_stages_param = getParameter("SelectBufferStages").toUInt();
                    double select_buffer_taper = getParameter("SelectBufferTaper").toDouble();
                    for(unsigned int i = 0; i < number_selects; ++i)
                    {
                        unsigned int fanout_count = countSelectBitFanout(number_inputs, i);
                        unsigned int n_stages = (select_buffer_stages_param > 0)
                            ? select_buffer_stages_param
                            : std::max((unsigned int)1,
                                (unsigned int)ceil(log((double)std::max(fanout_count, (unsigned int)1))
                                    / log(select_buffer_taper)));

                        ElectricalModel* prev_port_model = NULL;
                        String prev_port_name = "Sel" + (String)i;
                        for(unsigned int s = 0; s < n_stages; ++s)
                        {
                            ElectricalModel* stage = (ElectricalModel*)getSubInstance(String::format("SelBuf%d_%d", i, s));
                            if(s == 0)
                            {
                                propagatePortTransitionInfo(stage, "A", prev_port_name);
                            }
                            else
                            {
                                propagatePortTransitionInfo(stage, "A", prev_port_model, "Y");
                            }
                            stage->use();
                            prev_port_model = stage;
                        }
                    }
                }

                // Set output transition
                propagatePortTransitionInfo("Out", muxf, "Y");
            }
        }
        else
        {
            // Go through each bit and set the appropriate probability
            for (unsigned int n = 0; n < number_bits; ++n)
            {
                ElectricalModel* mux_bit = (ElectricalModel*)getSubInstance("Mux_bit" + (String) n);
                for(unsigned int i = 0; i < number_inputs; ++i)
                {
                    propagatePortTransitionInfo(mux_bit, "In" + (String)i, "In" + (String)i);
                }
                for(unsigned int i = 0; i < number_selects; ++i)
                {
                    propagatePortTransitionInfo(mux_bit, "Sel" + (String)i, "Sel" + (String)i);
                }
                mux_bit->use();
            }            

            // Set output probability to be average that of probabilties of each output bit
            ElectricalModel* mux_bit = (ElectricalModel*)getSubInstance("Mux_bit0");
            propagatePortTransitionInfo("Out", mux_bit, "Out");
        }        
        return;
    }

} // namespace DSENT


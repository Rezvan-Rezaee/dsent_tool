/* Copyright (c) 2012 Massachusetts Institute of Technology
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "model/electrical/router/OutputVCState.h"

#include <cmath>
#include <vector>

#include "model/PortInfo.h"
#include "model/TransitionInfo.h"
#include "model/EventInfo.h"
#include "model/ModelGen.h"
#include "model/std_cells/StdCellLib.h"
#include "model/std_cells/StdCell.h"

namespace DSENT
{
    using std::ceil;
    using std::log2;
    using std::pow;
    using std::vector;

    namespace
    {
        // Add every sub-instance's contribution to one top-level event
        // Result. Called once per top event (CreditDecrement,
        // CreditIncrement, Idle) with the same component set -- magnitude
        // differs per event only because propagateTransitionInfo() feeds
        // each component different (event-dependent) TransitionInfo before
        // use() is called, not because of anything here.
        void addAllSubEventResults(Model* self_, Result* event_result_, unsigned int cnt_bits_)
        {
            for(unsigned int i = 0; i < cnt_bits_; ++i)
            {
                const String& cnt_name = "Cnt_DFF" + (String)i;
                Model* cnt_dff = self_->getSubInstance(cnt_name);
                event_result_->addSubResult(cnt_dff->getEventResult("CK"), cnt_name, 1.0);
                event_result_->addSubResult(cnt_dff->getEventResult("DFFD"), cnt_name, 1.0);
                event_result_->addSubResult(cnt_dff->getEventResult("DFFQ"), cnt_name, 1.0);

                const String& mux_name = "UpDown_MUX" + (String)i;
                event_result_->addSubResult(self_->getSubInstance(mux_name)->getEventResult("MUX2"), mux_name, 1.0);

                if(i == 0)
                {
                    event_result_->addSubResult(self_->getSubInstance("Inc_INV0")->getEventResult("INV"), "Inc_INV0", 1.0);
                    event_result_->addSubResult(self_->getSubInstance("Dec_INV0")->getEventResult("INV"), "Dec_INV0", 1.0);
                }
                else
                {
                    const String& inc_xor_name = "Inc_XOR" + (String)i;
                    event_result_->addSubResult(self_->getSubInstance(inc_xor_name)->getEventResult("XOR2"), inc_xor_name, 1.0);
                    const String& dec_xor_name = "Dec_XOR" + (String)i;
                    event_result_->addSubResult(self_->getSubInstance(dec_xor_name)->getEventResult("XOR2"), dec_xor_name, 1.0);
                }
                if(i > 0 && i < cnt_bits_ - 1)
                {
                    const String& inc_and_name = "Inc_AND" + (String)i;
                    event_result_->addSubResult(self_->getSubInstance(inc_and_name)->getEventResult("AND2"), inc_and_name, 1.0);
                    const String& dec_or_name = "Dec_OR" + (String)i;
                    event_result_->addSubResult(self_->getSubInstance(dec_or_name)->getEventResult("OR2"), dec_or_name, 1.0);
                }
            }

            Model* busy_dff = self_->getSubInstance("Busy_DFF");
            event_result_->addSubResult(busy_dff->getEventResult("CK"), "Busy_DFF", 1.0);
            event_result_->addSubResult(busy_dff->getEventResult("DFFD"), "Busy_DFF", 1.0);
            event_result_->addSubResult(busy_dff->getEventResult("DFFQ"), "Busy_DFF", 1.0);

            Model* empty_dff = self_->getSubInstance("Empty_DFF");
            event_result_->addSubResult(empty_dff->getEventResult("CK"), "Empty_DFF", 1.0);
            event_result_->addSubResult(empty_dff->getEventResult("DFFD"), "Empty_DFF", 1.0);
            event_result_->addSubResult(empty_dff->getEventResult("DFFQ"), "Empty_DFF", 1.0);

            event_result_->addSubResult(self_->getSubInstance("ZeroDetect_OR")->getEventResult("OR"), "ZeroDetect_OR", 1.0);
            event_result_->addSubResult(self_->getSubInstance("Enable_OR2")->getEventResult("OR2"), "Enable_OR2", 1.0);
            event_result_->addSubResult(self_->getSubInstance("Gate_NAND2")->getEventResult("NAND2"), "Gate_NAND2", 1.0);
            event_result_->addSubResult(self_->getSubInstance("Gate_INV")->getEventResult("INV"), "Gate_INV", 1.0);
            return;
        }
    } // unnamed namespace

    OutputVCState::OutputVCState(const String& instance_name_, const TechModel* tech_model_)
        : ElectricalModel(instance_name_, tech_model_)
    {
        initParameters();
        initProperties();
    }

    OutputVCState::~OutputVCState()
    {}

    void OutputVCState::initParameters()
    {
        addParameterName("NumberEntries");
        // Credit events are throughput-driven, not clock-rate: at most one
        // credit can be sent/returned per port per cycle, since at most one
        // flit departs per input port per cycle. Default 1.0 (every cycle
        // moves a flit) is the conservative/worst-case upper bound.
        addParameterName("Utilization", "1.0");
        return;
    }

    void OutputVCState::initProperties()
    {
        return;
    }

    OutputVCState* OutputVCState::clone() const
    {
        // TODO
        return NULL;
    }

    void OutputVCState::constructModel()
    {
        unsigned int number_entries = getParameter("NumberEntries").toUInt();
        ASSERT(number_entries >= 1, "[Error] " + getInstanceName() +
            " -> NumberEntries must be >= 1!");

        // cnt_bits = ceil(log2(NumberEntries)), matching Becker Table 6.1's
        // static column exactly (no +1). An earlier version of this model
        // added +1 so the counter alone could represent the full 0..N
        // inclusive range (N+1 values) without wrapping. That's redundant:
        // Busy_DFF/Empty_DFF (below) already carry the two boundary states
        // (all-consumed / all-available) as their own dedicated flops --
        // exactly the role Table 6.1 gives its separate "busy/empty = 2*V"
        // line -- so the counter itself only needs to distinguish the
        // interior states, which fit in ceil(log2(NumberEntries)) bits.
        unsigned int cnt_bits = (unsigned int)ceil(log2((double)number_entries));
        getGenProperties()->set("CntBits", cnt_bits);

        // POLARITY: the reference source is inconsistent -- Sec 5.1
        // describes the counter as tracking AVAILABLE downstream entries,
        // Sec 6.3 as tracking OCCUPIED ones (these are complements). This
        // model implements AVAILABLE: reset/init value = NumberEntries,
        // decremented on Grant (a flit was just sent, consuming a
        // downstream slot), incremented on CreditIn (a downstream slot was
        // freed). Zero means blocked (no credit). Chosen because it needs
        // only a zero-detect (built below) and no maximum comparator --
        // credits returned can never exceed what was sent, so the count
        // can never legitimately exceed NumberEntries in this direction.

        createInputPort("CK");
        createInputPort("Grant");
        createInputPort("CreditIn");
        createOutputPort("HasCredit");
        createOutputPort("Busy");
        createOutputPort("Empty");

        createElectricalResults();
        getEventInfo("Idle")->setStaticTransitionInfos();
        getEventInfo("Idle")->setTransitionInfo("CK", TransitionInfo(0.0, 1.0, 0.0));

        // Grant/CreditIn's OWN assertion probability is scaled by
        // Utilization here (not just the destination counter bit's D
        // formula below) -- otherwise the clock-gate chain (Enable_OR2/
        // Gate_NAND2/Gate_INV) would see a fully-asserted Grant/CreditIn
        // every time this named event fires, regardless of Utilization,
        // and CreditDecrement energy would barely move with it (that gate
        // chain's own switching would dominate at low Utilization and mask
        // the counter's real, correctly-scaled activity).
        double utilization = getParameter("Utilization").toDouble();

        createElectricalEventResult("CreditDecrement");
        getEventInfo("CreditDecrement")->setTransitionInfo("CK", TransitionInfo(0.0, 1.0, 0.0));
        getEventInfo("CreditDecrement")->setTransitionInfo("Grant", TransitionInfo(1.0 - utilization, 0.0, utilization));
        getEventInfo("CreditDecrement")->setTransitionInfo("CreditIn", TransitionInfo(1.0, 0.0, 0.0));

        createElectricalEventResult("CreditIncrement");
        getEventInfo("CreditIncrement")->setTransitionInfo("CK", TransitionInfo(0.0, 1.0, 0.0));
        getEventInfo("CreditIncrement")->setTransitionInfo("CreditIn", TransitionInfo(1.0 - utilization, 0.0, utilization));
        getEventInfo("CreditIncrement")->setTransitionInfo("Grant", TransitionInfo(1.0, 0.0, 0.0));

        const StdCellLib* std_cell_lib = getTechModel()->getStdCellLib();

        // ---- Counter register (cnt_bits x DFFQ) + busy/empty (2 x DFFQ) -
        for(unsigned int i = 0; i < cnt_bits; ++i)
        {
            StdCell* cnt_dff = std_cell_lib->createStdCell("DFFQ", "Cnt_DFF" + (String)i);
            cnt_dff->construct();
            createNet("CntQ" + (String)i);
            portConnect(cnt_dff, "Q", "CntQ" + (String)i);
            addSubInstances(cnt_dff, 1.0);
            addElectricalSubResults(cnt_dff, 1.0);
        }
        StdCell* busy_dff = std_cell_lib->createStdCell("DFFQ", "Busy_DFF");
        busy_dff->construct();
        addSubInstances(busy_dff, 1.0);
        addElectricalSubResults(busy_dff, 1.0);
        StdCell* empty_dff = std_cell_lib->createStdCell("DFFQ", "Empty_DFF");
        empty_dff->construct();
        addSubInstances(empty_dff, 1.0);
        addElectricalSubResults(empty_dff, 1.0);

        // ---- Incrementer (on CreditIn): half-adder chain from Phase 1
        // (FIFOControl) -- bit 0 is an INV (carry-in is the constant 1
        // being added), so its own Q is the free carry-out into bit 1 (no
        // AND2 needed there). Bits 1..cnt_bits-1 each get an XOR2 (sum)
        // and, for all but the last bit, an AND2 (carry-out).
        for(unsigned int i = 0; i < cnt_bits; ++i)
        {
            if(i == 0)
            {
                StdCell* inc_inv0 = std_cell_lib->createStdCell("INV", "Inc_INV0");
                inc_inv0->construct();
                portConnect(inc_inv0, "A", "CntQ0");
                createNet("Inc_Sum0");
                portConnect(inc_inv0, "Y", "Inc_Sum0");
                addSubInstances(inc_inv0, 1.0);
                addElectricalSubResults(inc_inv0, 1.0);
            }
            else
            {
                StdCell* inc_xor = std_cell_lib->createStdCell("XOR2", "Inc_XOR" + (String)i);
                inc_xor->construct();
                portConnect(inc_xor, "A", "CntQ" + (String)i);
                if(i == 1)
                {
                    portConnect(inc_xor, "B", "CntQ0");
                }
                else
                {
                    portConnect(inc_xor, "B", "Inc_Carry" + (String)(i - 1));
                }
                createNet("Inc_Sum" + (String)i);
                portConnect(inc_xor, "Y", "Inc_Sum" + (String)i);
                addSubInstances(inc_xor, 1.0);
                addElectricalSubResults(inc_xor, 1.0);

                if(i < cnt_bits - 1)
                {
                    StdCell* inc_and = std_cell_lib->createStdCell("AND2", "Inc_AND" + (String)i);
                    inc_and->construct();
                    portConnect(inc_and, "A", "CntQ" + (String)i);
                    if(i == 1)
                    {
                        portConnect(inc_and, "B", "CntQ0");
                    }
                    else
                    {
                        portConnect(inc_and, "B", "Inc_Carry" + (String)(i - 1));
                    }
                    createNet("Inc_Carry" + (String)i);
                    portConnect(inc_and, "Y", "Inc_Carry" + (String)i);
                    addSubInstances(inc_and, 1.0);
                    addElectricalSubResults(inc_and, 1.0);
                }
            }
        }

        // ---- Decrementer (on Grant): the dual borrow chain -- OR2
        // replaces AND2 (same gate count), bit 0 is again an INV (borrow-in
        // is the constant 1 being subtracted), and its own output doubles
        // as the free borrow-out into bit 1.
        for(unsigned int i = 0; i < cnt_bits; ++i)
        {
            if(i == 0)
            {
                StdCell* dec_inv0 = std_cell_lib->createStdCell("INV", "Dec_INV0");
                dec_inv0->construct();
                portConnect(dec_inv0, "A", "CntQ0");
                createNet("Dec_Diff0");
                portConnect(dec_inv0, "Y", "Dec_Diff0");
                addSubInstances(dec_inv0, 1.0);
                addElectricalSubResults(dec_inv0, 1.0);
            }
            else
            {
                StdCell* dec_xor = std_cell_lib->createStdCell("XOR2", "Dec_XOR" + (String)i);
                dec_xor->construct();
                portConnect(dec_xor, "A", "CntQ" + (String)i);
                if(i == 1)
                {
                    portConnect(dec_xor, "B", "Dec_Diff0");
                }
                else
                {
                    portConnect(dec_xor, "B", "Dec_Borrow" + (String)(i - 1));
                }
                createNet("Dec_Diff" + (String)i);
                portConnect(dec_xor, "Y", "Dec_Diff" + (String)i);
                addSubInstances(dec_xor, 1.0);
                addElectricalSubResults(dec_xor, 1.0);

                if(i < cnt_bits - 1)
                {
                    StdCell* dec_or = std_cell_lib->createStdCell("OR2", "Dec_OR" + (String)i);
                    dec_or->construct();
                    portConnect(dec_or, "A", "CntQ" + (String)i);
                    if(i == 1)
                    {
                        portConnect(dec_or, "B", "Dec_Diff0");
                    }
                    else
                    {
                        portConnect(dec_or, "B", "Dec_Borrow" + (String)(i - 1));
                    }
                    createNet("Dec_Borrow" + (String)i);
                    portConnect(dec_or, "Y", "Dec_Borrow" + (String)i);
                    addSubInstances(dec_or, 1.0);
                    addElectricalSubResults(dec_or, 1.0);
                }
            }
        }

        // ---- Up/down select (cnt_bits x MUX2): choose the decrementer's
        // diff (A) or the incrementer's sum (B) as the counter's next
        // value, selected by CreditIn (S0=1 -> increment path). No third
        // "hold" input -- see HOLD BEHAVIOUR below.
        for(unsigned int i = 0; i < cnt_bits; ++i)
        {
            StdCell* mux = std_cell_lib->createStdCell("MUX2", "UpDown_MUX" + (String)i);
            mux->construct();
            portConnect(mux, "A", (i == 0) ? "Dec_Diff0" : ("Dec_Diff" + (String)i));
            portConnect(mux, "B", (i == 0) ? "Inc_Sum0" : ("Inc_Sum" + (String)i));
            portConnect(mux, "S0", "CreditIn");
            createNet("Sel_Sum" + (String)i);
            portConnect(mux, "Y", "Sel_Sum" + (String)i);
            portConnect((ElectricalModel*)getSubInstance("Cnt_DFF" + (String)i), "D", "Sel_Sum" + (String)i);
            addSubInstances(mux, 1.0);
            addElectricalSubResults(mux, 1.0);
        }

        // ---- Zero detect (1 x OR, NOR-tree role): high whenever ANY
        // counter bit is set. For an AVAILABLE-credits counter, "any bit
        // set" is exactly "count != 0" -- i.e. exactly HasCredit. Unlike
        // Phase 1's equality check, no inversion is needed here.
        ElectricalModel* zero_detect_or = (ElectricalModel*)ModelGen::createModel("OR", "ZeroDetect_OR", getTechModel());
        zero_detect_or->setParameter("NumberInputs", (String)cnt_bits);
        zero_detect_or->setParameter("NumberBits", "1");
        zero_detect_or->construct();
        for(unsigned int i = 0; i < cnt_bits; ++i)
        {
            portConnect(zero_detect_or, "In" + (String)i, "CntQ" + (String)i);
        }
        portConnect(zero_detect_or, "Out", "HasCredit");
        addSubInstances(zero_detect_or, 1.0);
        addElectricalSubResults(zero_detect_or, 1.0);

        // ---- Clock gate: NAND2 + INV, same technique DFFRAM uses to gate
        // per-row clocks. Enable = Grant OR CreditIn -- a minimal 1-gate
        // addition beyond the given SUB-INSTANCES list, needed because
        // gating one clock from two independent enable conditions requires
        // combining them first; it's O(1), not O(cnt_bits), so it doesn't
        // change the "one NAND2 + INV, saves cnt_bits muxes" cost story.
        // On a cycle with neither Grant nor CreditIn, Enable is low, the
        // gated clock never toggles, and the counter/busy/empty flops see
        // no clock edge -- only the gating cells themselves (whose inputs
        // are still driven by the real, always-toggling CK) switch.
        StdCell* enable_or2 = std_cell_lib->createStdCell("OR2", "Enable_OR2");
        enable_or2->construct();
        portConnect(enable_or2, "A", "Grant");
        portConnect(enable_or2, "B", "CreditIn");
        createNet("Enable");
        portConnect(enable_or2, "Y", "Enable");
        addSubInstances(enable_or2, 1.0);
        addElectricalSubResults(enable_or2, 1.0);

        StdCell* gate_nand2 = std_cell_lib->createStdCell("NAND2", "Gate_NAND2");
        gate_nand2->construct();
        portConnect(gate_nand2, "A", "CK");
        portConnect(gate_nand2, "B", "Enable");
        createNet("Gate_NAND_Out");
        portConnect(gate_nand2, "Y", "Gate_NAND_Out");
        addSubInstances(gate_nand2, 1.0);
        addElectricalSubResults(gate_nand2, 1.0);

        StdCell* gate_inv = std_cell_lib->createStdCell("INV", "Gate_INV");
        gate_inv->construct();
        portConnect(gate_inv, "A", "Gate_NAND_Out");
        createNet("GatedCK");
        portConnect(gate_inv, "Y", "GatedCK");
        addSubInstances(gate_inv, 1.0);
        addElectricalSubResults(gate_inv, 1.0);

        for(unsigned int i = 0; i < cnt_bits; ++i)
        {
            portConnect((ElectricalModel*)getSubInstance("Cnt_DFF" + (String)i), "CK", "GatedCK");
        }
        portConnect(busy_dff, "CK", "GatedCK");
        portConnect(empty_dff, "CK", "GatedCK");
        // DSENT-ASSUMPTION: Busy/Empty (Table 6.1's separate "busy/empty
        // flags = 2*V" allocation state, distinct from the credit counter)
        // are driven directly from Grant/CreditIn respectively -- real
        // VC-allocation set/clear logic needs a header/tail-flit signal
        // that isn't in this model's PORTS list (CK/Grant/CreditIn only),
        // so this preserves the correct flop count/gate-count structure
        // without inventing ungiven ports; DSENT never evaluates boolean
        // correctness, only the capacitance/switching-activity accounting.
        portConnect(busy_dff, "D", "Grant");
        portConnect(empty_dff, "D", "CreditIn");
        portConnect(busy_dff, "Q", "Busy");
        portConnect(empty_dff, "Q", "Empty");

        // ---- Wire up event Results --------------------------------------
        addAllSubEventResults(this, getEventResult("CreditDecrement"), cnt_bits);
        addAllSubEventResults(this, getEventResult("CreditIncrement"), cnt_bits);
        addAllSubEventResults(this, getEventResult("Idle"), cnt_bits);

        return;
    }

    void OutputVCState::propagateTransitionInfo()
    {
        unsigned int cnt_bits = getGenProperties()->get("CntBits").toUInt();
        const String& current_event = getGenProperties()->get("UseModelEvent");
        bool is_dec = (current_event == "CreditDecrement");
        bool is_inc = (current_event == "CreditIncrement");
        double utilization = getParameter("Utilization").toDouble();

        // ---- Clock gate: real propagation, CK always toggles at the top
        // (per EventInfo above); Enable only goes high during a real
        // decrement/increment. ---------------------------------------------
        ElectricalModel* enable_or2 = (ElectricalModel*)getSubInstance("Enable_OR2");
        propagatePortTransitionInfo(enable_or2, "A", "Grant");
        propagatePortTransitionInfo(enable_or2, "B", "CreditIn");
        enable_or2->use();

        ElectricalModel* gate_nand2 = (ElectricalModel*)getSubInstance("Gate_NAND2");
        propagatePortTransitionInfo(gate_nand2, "A", "CK");
        propagatePortTransitionInfo(gate_nand2, "B", enable_or2, "Y");
        gate_nand2->use();

        ElectricalModel* gate_inv = (ElectricalModel*)getSubInstance("Gate_INV");
        propagatePortTransitionInfo(gate_inv, "A", gate_nand2, "Y");
        gate_inv->use();

        // ---- Counter register: counter-aware activity model (Phase 1's
        // formula, scaled by Utilization since credit events are
        // throughput-driven, not clock-rate -- at most one credit per port
        // per cycle). Leaving this at the flat 0.25 default would overstate
        // energy by 1/Utilization at the LSB, and far more at high bits
        // (both effects compound: flat ignores both the bit-position
        // falloff AND the throughput scaling).
        for(unsigned int i = 0; i < cnt_bits; ++i)
        {
            ElectricalModel* cnt_dff = (ElectricalModel*)getSubInstance("Cnt_DFF" + (String)i);
            if(is_dec || is_inc)
            {
                double p = utilization / pow(2.0, (double)(i + 1));
                assignPortTransitionInfo(cnt_dff, "D", TransitionInfo((1.0 - p) / 2.0, p / 2.0, (1.0 - p) / 2.0));
            }
            else
            {
                assignPortTransitionInfo(cnt_dff, "D", TransitionInfo(0.5, 0.0, 0.5));
            }
            propagatePortTransitionInfo(cnt_dff, "CK", gate_inv, "Y");
            cnt_dff->use();
        }

        ElectricalModel* busy_dff = (ElectricalModel*)getSubInstance("Busy_DFF");
        propagatePortTransitionInfo(busy_dff, "D", "Grant");
        propagatePortTransitionInfo(busy_dff, "CK", gate_inv, "Y");
        busy_dff->use();

        ElectricalModel* empty_dff = (ElectricalModel*)getSubInstance("Empty_DFF");
        propagatePortTransitionInfo(empty_dff, "D", "CreditIn");
        propagatePortTransitionInfo(empty_dff, "CK", gate_inv, "Y");
        empty_dff->use();

        // ---- Incrementer / decrementer: real propagation from the
        // counter's own Q, for their own switching energy -- independent
        // of the counter-aware override applied above directly to Cnt_DFF's
        // D pin.
        for(unsigned int i = 0; i < cnt_bits; ++i)
        {
            ElectricalModel* cnt_dff_bit = (ElectricalModel*)getSubInstance("Cnt_DFF" + (String)i);
            if(i == 0)
            {
                ElectricalModel* inc_inv0 = (ElectricalModel*)getSubInstance("Inc_INV0");
                propagatePortTransitionInfo(inc_inv0, "A", cnt_dff_bit, "Q");
                inc_inv0->use();

                ElectricalModel* dec_inv0 = (ElectricalModel*)getSubInstance("Dec_INV0");
                propagatePortTransitionInfo(dec_inv0, "A", cnt_dff_bit, "Q");
                dec_inv0->use();
            }
            else
            {
                ElectricalModel* inc_xor = (ElectricalModel*)getSubInstance("Inc_XOR" + (String)i);
                propagatePortTransitionInfo(inc_xor, "A", cnt_dff_bit, "Q");
                ElectricalModel* dec_xor = (ElectricalModel*)getSubInstance("Dec_XOR" + (String)i);
                propagatePortTransitionInfo(dec_xor, "A", cnt_dff_bit, "Q");

                if(i == 1)
                {
                    ElectricalModel* cnt_bit0 = (ElectricalModel*)getSubInstance("Cnt_DFF0");
                    propagatePortTransitionInfo(inc_xor, "B", cnt_bit0, "Q");
                    ElectricalModel* dec_inv0 = (ElectricalModel*)getSubInstance("Dec_INV0");
                    propagatePortTransitionInfo(dec_xor, "B", dec_inv0, "Y");
                }
                else
                {
                    ElectricalModel* inc_and_prev = (ElectricalModel*)getSubInstance("Inc_AND" + (String)(i - 1));
                    propagatePortTransitionInfo(inc_xor, "B", inc_and_prev, "Y");
                    ElectricalModel* dec_or_prev = (ElectricalModel*)getSubInstance("Dec_OR" + (String)(i - 1));
                    propagatePortTransitionInfo(dec_xor, "B", dec_or_prev, "Y");
                }
                inc_xor->use();
                dec_xor->use();

                if(i < cnt_bits - 1)
                {
                    ElectricalModel* inc_and = (ElectricalModel*)getSubInstance("Inc_AND" + (String)i);
                    propagatePortTransitionInfo(inc_and, "A", cnt_dff_bit, "Q");
                    ElectricalModel* dec_or = (ElectricalModel*)getSubInstance("Dec_OR" + (String)i);
                    propagatePortTransitionInfo(dec_or, "A", cnt_dff_bit, "Q");
                    if(i == 1)
                    {
                        ElectricalModel* cnt_bit0 = (ElectricalModel*)getSubInstance("Cnt_DFF0");
                        propagatePortTransitionInfo(inc_and, "B", cnt_bit0, "Q");
                        ElectricalModel* dec_inv0 = (ElectricalModel*)getSubInstance("Dec_INV0");
                        propagatePortTransitionInfo(dec_or, "B", dec_inv0, "Y");
                    }
                    else
                    {
                        ElectricalModel* inc_and_prev = (ElectricalModel*)getSubInstance("Inc_AND" + (String)(i - 1));
                        propagatePortTransitionInfo(inc_and, "B", inc_and_prev, "Y");
                        ElectricalModel* dec_or_prev = (ElectricalModel*)getSubInstance("Dec_OR" + (String)(i - 1));
                        propagatePortTransitionInfo(dec_or, "B", dec_or_prev, "Y");
                    }
                    inc_and->use();
                    dec_or->use();
                }
            }
        }

        // ---- Up/down select mux: real propagation from decrementer /
        // incrementer outputs and CreditIn's select line.
        for(unsigned int i = 0; i < cnt_bits; ++i)
        {
            ElectricalModel* mux = (ElectricalModel*)getSubInstance("UpDown_MUX" + (String)i);
            if(i == 0)
            {
                ElectricalModel* dec_inv0 = (ElectricalModel*)getSubInstance("Dec_INV0");
                ElectricalModel* inc_inv0 = (ElectricalModel*)getSubInstance("Inc_INV0");
                propagatePortTransitionInfo(mux, "A", dec_inv0, "Y");
                propagatePortTransitionInfo(mux, "B", inc_inv0, "Y");
            }
            else
            {
                ElectricalModel* dec_xor = (ElectricalModel*)getSubInstance("Dec_XOR" + (String)i);
                ElectricalModel* inc_xor = (ElectricalModel*)getSubInstance("Inc_XOR" + (String)i);
                propagatePortTransitionInfo(mux, "A", dec_xor, "Y");
                propagatePortTransitionInfo(mux, "B", inc_xor, "Y");
            }
            propagatePortTransitionInfo(mux, "S0", "CreditIn");
            mux->use();
        }

        // ---- Zero detect ------------------------------------------------
        ElectricalModel* zero_detect_or = (ElectricalModel*)getSubInstance("ZeroDetect_OR");
        for(unsigned int i = 0; i < cnt_bits; ++i)
        {
            ElectricalModel* cnt_dff_bit = (ElectricalModel*)getSubInstance("Cnt_DFF" + (String)i);
            propagatePortTransitionInfo(zero_detect_or, "In" + (String)i, cnt_dff_bit, "Q");
        }
        zero_detect_or->use();

        // ---- Top-level outputs -------------------------------------------
        propagatePortTransitionInfo("HasCredit", zero_detect_or, "Out");
        propagatePortTransitionInfo("Busy", busy_dff, "Q");
        propagatePortTransitionInfo("Empty", empty_dff, "Q");

        return;
    }

} // namespace DSENT

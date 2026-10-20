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

#include "model/electrical/router/FIFOControl.h"

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
        // Result. Called once per top event (Push, Pop, Idle) with the same
        // component set -- the actual magnitude differs per event only
        // because propagateTransitionInfo() feeds each component different
        // (event-dependent) TransitionInfo before use() is called, not
        // because of anything here.
        void addAllSubEventResults(Model* self_, Result* event_result_, unsigned int idx_bits_)
        {
            for(unsigned int i = 0; i < idx_bits_; ++i)
            {
                const String& wr_name = "WRIdx_DFF" + (String)i;
                Model* wr_dff = self_->getSubInstance(wr_name);
                event_result_->addSubResult(wr_dff->getEventResult("CK"), wr_name, 1.0);
                event_result_->addSubResult(wr_dff->getEventResult("DFFD"), wr_name, 1.0);
                event_result_->addSubResult(wr_dff->getEventResult("DFFQ"), wr_name, 1.0);

                const String& rd_name = "RDIdx_DFF" + (String)i;
                Model* rd_dff = self_->getSubInstance(rd_name);
                event_result_->addSubResult(rd_dff->getEventResult("CK"), rd_name, 1.0);
                event_result_->addSubResult(rd_dff->getEventResult("DFFD"), rd_name, 1.0);
                event_result_->addSubResult(rd_dff->getEventResult("DFFQ"), rd_name, 1.0);

                const String& wr_xor_name = "WR_Incr_XOR" + (String)i;
                event_result_->addSubResult(self_->getSubInstance(wr_xor_name)->getEventResult("XOR2"), wr_xor_name, 1.0);

                const String& rd_xor_name = "RD_Incr_XOR" + (String)i;
                event_result_->addSubResult(self_->getSubInstance(rd_xor_name)->getEventResult("XOR2"), rd_xor_name, 1.0);

                const String& cmp_name = "Cmp_XOR" + (String)i;
                event_result_->addSubResult(self_->getSubInstance(cmp_name)->getEventResult("XOR2"), cmp_name, 1.0);

                if(i > 0)
                {
                    const String& wr_and_name = "WR_Incr_AND" + (String)i;
                    event_result_->addSubResult(self_->getSubInstance(wr_and_name)->getEventResult("AND2"), wr_and_name, 1.0);

                    const String& rd_and_name = "RD_Incr_AND" + (String)i;
                    event_result_->addSubResult(self_->getSubInstance(rd_and_name)->getEventResult("AND2"), rd_and_name, 1.0);
                }
            }

            Model* flag_dff = self_->getSubInstance("Flag_DFF");
            event_result_->addSubResult(flag_dff->getEventResult("CK"), "Flag_DFF", 1.0);
            event_result_->addSubResult(flag_dff->getEventResult("DFFD"), "Flag_DFF", 1.0);
            event_result_->addSubResult(flag_dff->getEventResult("DFFQ"), "Flag_DFF", 1.0);

            event_result_->addSubResult(self_->getSubInstance("EqOR")->getEventResult("OR"), "EqOR", 1.0);
            event_result_->addSubResult(self_->getSubInstance("Full_AND")->getEventResult("AND2"), "Full_AND", 1.0);
            event_result_->addSubResult(self_->getSubInstance("Empty_AND")->getEventResult("AND2"), "Empty_AND", 1.0);
            return;
        }
    } // unnamed namespace

    FIFOControl::FIFOControl(const String& instance_name_, const TechModel* tech_model_)
        : ElectricalModel(instance_name_, tech_model_)
    {
        initParameters();
        initProperties();
    }

    FIFOControl::~FIFOControl()
    {}

    void FIFOControl::initParameters()
    {
        addParameterName("NumberEntries");
        // Validation/debug knob only: "Counter" (default) applies the
        // per-bit counter-toggle-probability activity model described
        // below; "Flat" reverts index bits to the naive
        // TransitionInfo(0.25,0.25,0.25) default so the two can be
        // compared directly (see VALIDATION #4 in the design notes).
        addParameterName("ActivityModel", "Counter");
        return;
    }

    void FIFOControl::initProperties()
    {
        return;
    }

    FIFOControl* FIFOControl::clone() const
    {
        // TODO
        return NULL;
    }

    void FIFOControl::constructModel()
    {
        unsigned int number_entries = getParameter("NumberEntries").toUInt();

        ASSERT(number_entries >= 2, "[Error] " + getInstanceName() +
            " -> NumberEntries must be >= 2!");
        ASSERT((number_entries & (number_entries - 1)) == 0, "[Error] " + getInstanceName() +
            " -> NumberEntries (" + (String)number_entries + ") must be a power of two -- "
            "the pointer wraps naturally at the top of its bit width only when this holds. "
            "Relaxing this would need a modulo comparator plus a reset mux (roughly doubling "
            "the combinational cost of this model), which is not implemented here.");

        unsigned int idx_bits = (unsigned int)ceil(log2((double)number_entries));
        getGenProperties()->set("IdxBits", idx_bits);

        // Ports
        createInputPort("CK");
        createInputPort("Push");
        createInputPort("Pop");
        createOutputPort("WRIdx", makeNetIndex(0, idx_bits - 1));
        createOutputPort("RDIdx", makeNetIndex(0, idx_bits - 1));
        createOutputPort("Empty");
        createOutputPort("Full");

        // Structural constant-1 tie: the only real "second operand" a
        // pointer increment ever has is the constant 1 being added (this is
        // an incrementer, not a two-operand adder -- see the half-adder
        // chain below, and do NOT reach for RippleAdder here). It has no
        // driving sub-instance; propagateTransitionInfo() assigns its
        // consumers' TransitionInfo directly, the way a real tie-high cell
        // would.
        createNet("VddTie");

        createElectricalResults();
        getEventInfo("Idle")->setStaticTransitionInfos();
        getEventInfo("Idle")->setTransitionInfo("CK", TransitionInfo(0.0, 1.0, 0.0));

        createElectricalEventResult("Push");
        getEventInfo("Push")->setTransitionInfo("CK", TransitionInfo(0.0, 1.0, 0.0));
        getEventInfo("Push")->setTransitionInfo("Push", TransitionInfo(0.0, 0.0, 1.0));
        getEventInfo("Push")->setTransitionInfo("Pop", TransitionInfo(1.0, 0.0, 0.0));

        createElectricalEventResult("Pop");
        getEventInfo("Pop")->setTransitionInfo("CK", TransitionInfo(0.0, 1.0, 0.0));
        getEventInfo("Pop")->setTransitionInfo("Pop", TransitionInfo(0.0, 0.0, 1.0));
        getEventInfo("Pop")->setTransitionInfo("Push", TransitionInfo(1.0, 0.0, 0.0));

        const StdCellLib* std_cell_lib = getTechModel()->getStdCellLib();

        // ---- Index registers (2 x idx_bits DFFQ) + flag (1 x DFFQ) ------
        for(unsigned int i = 0; i < idx_bits; ++i)
        {
            StdCell* wr_dff = std_cell_lib->createStdCell("DFFQ", "WRIdx_DFF" + (String)i);
            wr_dff->construct();
            portConnect(wr_dff, "CK", "CK");
            portConnect(wr_dff, "Q", "WRIdx", makeNetIndex(i));
            addSubInstances(wr_dff, 1.0);
            addElectricalSubResults(wr_dff, 1.0);

            StdCell* rd_dff = std_cell_lib->createStdCell("DFFQ", "RDIdx_DFF" + (String)i);
            rd_dff->construct();
            portConnect(rd_dff, "CK", "CK");
            portConnect(rd_dff, "Q", "RDIdx", makeNetIndex(i));
            addSubInstances(rd_dff, 1.0);
            addElectricalSubResults(rd_dff, 1.0);
        }

        StdCell* flag_dff = std_cell_lib->createStdCell("DFFQ", "Flag_DFF");
        flag_dff->construct();
        portConnect(flag_dff, "CK", "CK");
        createNet("Flag_Q");
        portConnect(flag_dff, "Q", "Flag_Q");
        addSubInstances(flag_dff, 1.0);
        addElectricalSubResults(flag_dff, 1.0);

        // ---- Write / read incrementers: half-adder chains implementing
        // +1 (NOT a two-operand add -- RippleAdder would overcount by
        // ~2.5x per bit here). Bit 0's carry-in is the constant tie (its
        // carry-out is then just its own Q, for free -- no AND2 needed).
        // Bit i>0: sum_i = XOR2(Q_i, carry_{i-1}), carry_i = AND2(Q_i,
        // carry_{i-1}). The very top carry (out of bit idx_bits-1) is
        // intentionally left unused -- with NumberEntries a power of two
        // the pointer is meant to overflow/wrap for free (see the
        // power-of-two ASSERT above), so nothing needs to consume it.
        for(unsigned int i = 0; i < idx_bits; ++i)
        {
            StdCell* wr_xor = std_cell_lib->createStdCell("XOR2", "WR_Incr_XOR" + (String)i);
            wr_xor->construct();
            portConnect(wr_xor, "A", "WRIdx", makeNetIndex(i));
            if(i == 0)
            {
                portConnect(wr_xor, "B", "VddTie");
            }
            else if(i == 1)
            {
                portConnect(wr_xor, "B", "WRIdx", makeNetIndex(0));
            }
            else
            {
                portConnect(wr_xor, "B", "WR_Carry" + (String)(i - 1));
            }
            createNet("WR_Sum" + (String)i);
            portConnect(wr_xor, "Y", "WR_Sum" + (String)i);
            portConnect((ElectricalModel*)getSubInstance("WRIdx_DFF" + (String)i), "D", "WR_Sum" + (String)i);
            addSubInstances(wr_xor, 1.0);
            addElectricalSubResults(wr_xor, 1.0);

            StdCell* rd_xor = std_cell_lib->createStdCell("XOR2", "RD_Incr_XOR" + (String)i);
            rd_xor->construct();
            portConnect(rd_xor, "A", "RDIdx", makeNetIndex(i));
            if(i == 0)
            {
                portConnect(rd_xor, "B", "VddTie");
            }
            else if(i == 1)
            {
                portConnect(rd_xor, "B", "RDIdx", makeNetIndex(0));
            }
            else
            {
                portConnect(rd_xor, "B", "RD_Carry" + (String)(i - 1));
            }
            createNet("RD_Sum" + (String)i);
            portConnect(rd_xor, "Y", "RD_Sum" + (String)i);
            portConnect((ElectricalModel*)getSubInstance("RDIdx_DFF" + (String)i), "D", "RD_Sum" + (String)i);
            addSubInstances(rd_xor, 1.0);
            addElectricalSubResults(rd_xor, 1.0);

            if(i > 0)
            {
                StdCell* wr_and = std_cell_lib->createStdCell("AND2", "WR_Incr_AND" + (String)i);
                wr_and->construct();
                portConnect(wr_and, "A", "WRIdx", makeNetIndex(i));
                if(i == 1)
                {
                    portConnect(wr_and, "B", "WRIdx", makeNetIndex(0));
                }
                else
                {
                    portConnect(wr_and, "B", "WR_Carry" + (String)(i - 1));
                }
                createNet("WR_Carry" + (String)i);
                portConnect(wr_and, "Y", "WR_Carry" + (String)i);
                addSubInstances(wr_and, 1.0);
                addElectricalSubResults(wr_and, 1.0);

                StdCell* rd_and = std_cell_lib->createStdCell("AND2", "RD_Incr_AND" + (String)i);
                rd_and->construct();
                portConnect(rd_and, "A", "RDIdx", makeNetIndex(i));
                if(i == 1)
                {
                    portConnect(rd_and, "B", "RDIdx", makeNetIndex(0));
                }
                else
                {
                    portConnect(rd_and, "B", "RD_Carry" + (String)(i - 1));
                }
                createNet("RD_Carry" + (String)i);
                portConnect(rd_and, "Y", "RD_Carry" + (String)i);
                addSubInstances(rd_and, 1.0);
                addElectricalSubResults(rd_and, 1.0);
            }
        }

        // ---- Index comparator (idx_bits x XOR2) + equality reduction
        // (1 x OR, playing the NOR-tree role: its output is high whenever
        // any bit differs) ---------------------------------------------
        for(unsigned int i = 0; i < idx_bits; ++i)
        {
            StdCell* cmp_xor = std_cell_lib->createStdCell("XOR2", "Cmp_XOR" + (String)i);
            cmp_xor->construct();
            portConnect(cmp_xor, "A", "WRIdx", makeNetIndex(i));
            portConnect(cmp_xor, "B", "RDIdx", makeNetIndex(i));
            createNet("Diff" + (String)i);
            portConnect(cmp_xor, "Y", "Diff" + (String)i);
            addSubInstances(cmp_xor, 1.0);
            addElectricalSubResults(cmp_xor, 1.0);
        }

        ElectricalModel* eq_or = (ElectricalModel*)ModelGen::createModel("OR", "EqOR", getTechModel());
        eq_or->setParameter("NumberInputs", (String)idx_bits);
        eq_or->setParameter("NumberBits", "1");
        eq_or->construct();
        for(unsigned int i = 0; i < idx_bits; ++i)
        {
            portConnect(eq_or, "In" + (String)i, "Diff" + (String)i);
        }
        createNet("PtrsDiffer");
        portConnect(eq_or, "Out", "PtrsDiffer");
        addSubInstances(eq_or, 1.0);
        addElectricalSubResults(eq_or, 1.0);

        // ---- Flag: registers whether the pointers currently differ.
        // DSENT-ASSUMPTION: a fully precise full/empty status generator
        // needs an inverted (NOT PtrsDiffer / NOT Flag) term that isn't in
        // this gate budget; DSENT only tracks capacitance/switching
        // statistics (it never evaluates boolean truth), so this doesn't
        // change what's being modeled -- only the flop/gate count and
        // their activity matter, and those match the spec exactly.
        portConnect(flag_dff, "D", "PtrsDiffer");

        // ---- Empty/full gating (2 x AND2) -------------------------------
        StdCell* full_and = std_cell_lib->createStdCell("AND2", "Full_AND");
        full_and->construct();
        portConnect(full_and, "A", "Flag_Q");
        portConnect(full_and, "B", "Push");
        portConnect(full_and, "Y", "Full");
        addSubInstances(full_and, 1.0);
        addElectricalSubResults(full_and, 1.0);

        StdCell* empty_and = std_cell_lib->createStdCell("AND2", "Empty_AND");
        empty_and->construct();
        portConnect(empty_and, "A", "Flag_Q");
        portConnect(empty_and, "B", "Pop");
        portConnect(empty_and, "Y", "Empty");
        addSubInstances(empty_and, 1.0);
        addElectricalSubResults(empty_and, 1.0);

        // ---- Wire up event Results --------------------------------------
        addAllSubEventResults(this, getEventResult("Push"), idx_bits);
        addAllSubEventResults(this, getEventResult("Pop"), idx_bits);
        addAllSubEventResults(this, getEventResult("Idle"), idx_bits);

        return;
    }

    void FIFOControl::propagateTransitionInfo()
    {
        unsigned int idx_bits = getGenProperties()->get("IdxBits").toUInt();
        const String& current_event = getGenProperties()->get("UseModelEvent");
        bool is_push = (current_event == "Push");
        bool is_pop = (current_event == "Pop");
        bool use_flat_activity = (getParameter("ActivityModel") == "Flat");

        // ---- Write index register: counter-aware activity model --------
        // This replaces the flat TransitionInfo(0.25,0.25,0.25) default. A
        // binary counter's bit k toggles once every 2^k increments, so its
        // 0->1 probability per increment is 1/2^(k+1) (half the toggle
        // rate -- each toggle is either a 0->1 or a 1->0). Leaving this at
        // the flat default would over-charge every high bit as if it
        // switched as often as the LSB, up to 8x too much energy at
        // typical (8-16 entry) depths.
        for(unsigned int i = 0; i < idx_bits; ++i)
        {
            ElectricalModel* wr_dff = (ElectricalModel*)getSubInstance("WRIdx_DFF" + (String)i);
            if(is_push && use_flat_activity)
            {
                assignPortTransitionInfo(wr_dff, "D", TransitionInfo(0.25, 0.25, 0.25));
            }
            else if(is_push)
            {
                double p = 1.0 / pow(2.0, (double)(i + 1));
                assignPortTransitionInfo(wr_dff, "D", TransitionInfo((1.0 - p) / 2.0, p / 2.0, (1.0 - p) / 2.0));
            }
            else
            {
                // Not pushing this cycle -- write pointer holds.
                assignPortTransitionInfo(wr_dff, "D", TransitionInfo(0.5, 0.0, 0.5));
            }
            propagatePortTransitionInfo(wr_dff, "CK", "CK");
            wr_dff->use();
        }

        for(unsigned int i = 0; i < idx_bits; ++i)
        {
            ElectricalModel* rd_dff = (ElectricalModel*)getSubInstance("RDIdx_DFF" + (String)i);
            if(is_pop && use_flat_activity)
            {
                assignPortTransitionInfo(rd_dff, "D", TransitionInfo(0.25, 0.25, 0.25));
            }
            else if(is_pop)
            {
                double p = 1.0 / pow(2.0, (double)(i + 1));
                assignPortTransitionInfo(rd_dff, "D", TransitionInfo((1.0 - p) / 2.0, p / 2.0, (1.0 - p) / 2.0));
            }
            else
            {
                // Not popping this cycle -- read pointer holds.
                assignPortTransitionInfo(rd_dff, "D", TransitionInfo(0.5, 0.0, 0.5));
            }
            propagatePortTransitionInfo(rd_dff, "CK", "CK");
            rd_dff->use();
        }

        // ---- Write / read incrementers: real propagation from the
        // pointer's own Q, purely for the incrementer's own switching
        // energy -- independent of the counter-aware override applied
        // above directly to the destination flop's D pin. -----------------
        for(unsigned int i = 0; i < idx_bits; ++i)
        {
            ElectricalModel* wr_dff_bit = (ElectricalModel*)getSubInstance("WRIdx_DFF" + (String)i);
            ElectricalModel* wr_xor = (ElectricalModel*)getSubInstance("WR_Incr_XOR" + (String)i);
            propagatePortTransitionInfo(wr_xor, "A", wr_dff_bit, "Q");
            if(i == 0)
            {
                assignPortTransitionInfo(wr_xor, "B", TransitionInfo(0.0, 0.0, 1.0));
            }
            else if(i == 1)
            {
                ElectricalModel* wr_bit0 = (ElectricalModel*)getSubInstance("WRIdx_DFF0");
                propagatePortTransitionInfo(wr_xor, "B", wr_bit0, "Q");
            }
            else
            {
                ElectricalModel* wr_and_prev = (ElectricalModel*)getSubInstance("WR_Incr_AND" + (String)(i - 1));
                propagatePortTransitionInfo(wr_xor, "B", wr_and_prev, "Y");
            }
            wr_xor->use();

            ElectricalModel* rd_dff_bit = (ElectricalModel*)getSubInstance("RDIdx_DFF" + (String)i);
            ElectricalModel* rd_xor = (ElectricalModel*)getSubInstance("RD_Incr_XOR" + (String)i);
            propagatePortTransitionInfo(rd_xor, "A", rd_dff_bit, "Q");
            if(i == 0)
            {
                assignPortTransitionInfo(rd_xor, "B", TransitionInfo(0.0, 0.0, 1.0));
            }
            else if(i == 1)
            {
                ElectricalModel* rd_bit0 = (ElectricalModel*)getSubInstance("RDIdx_DFF0");
                propagatePortTransitionInfo(rd_xor, "B", rd_bit0, "Q");
            }
            else
            {
                ElectricalModel* rd_and_prev = (ElectricalModel*)getSubInstance("RD_Incr_AND" + (String)(i - 1));
                propagatePortTransitionInfo(rd_xor, "B", rd_and_prev, "Y");
            }
            rd_xor->use();

            if(i > 0)
            {
                ElectricalModel* wr_and = (ElectricalModel*)getSubInstance("WR_Incr_AND" + (String)i);
                propagatePortTransitionInfo(wr_and, "A", wr_dff_bit, "Q");
                if(i == 1)
                {
                    ElectricalModel* wr_bit0 = (ElectricalModel*)getSubInstance("WRIdx_DFF0");
                    propagatePortTransitionInfo(wr_and, "B", wr_bit0, "Q");
                }
                else
                {
                    ElectricalModel* wr_and_prev = (ElectricalModel*)getSubInstance("WR_Incr_AND" + (String)(i - 1));
                    propagatePortTransitionInfo(wr_and, "B", wr_and_prev, "Y");
                }
                wr_and->use();

                ElectricalModel* rd_and = (ElectricalModel*)getSubInstance("RD_Incr_AND" + (String)i);
                propagatePortTransitionInfo(rd_and, "A", rd_dff_bit, "Q");
                if(i == 1)
                {
                    ElectricalModel* rd_bit0 = (ElectricalModel*)getSubInstance("RDIdx_DFF0");
                    propagatePortTransitionInfo(rd_and, "B", rd_bit0, "Q");
                }
                else
                {
                    ElectricalModel* rd_and_prev = (ElectricalModel*)getSubInstance("RD_Incr_AND" + (String)(i - 1));
                    propagatePortTransitionInfo(rd_and, "B", rd_and_prev, "Y");
                }
                rd_and->use();
            }
        }

        // ---- Comparator + equality reduction ----------------------------
        for(unsigned int i = 0; i < idx_bits; ++i)
        {
            ElectricalModel* wr_dff_bit = (ElectricalModel*)getSubInstance("WRIdx_DFF" + (String)i);
            ElectricalModel* rd_dff_bit = (ElectricalModel*)getSubInstance("RDIdx_DFF" + (String)i);
            ElectricalModel* cmp_xor = (ElectricalModel*)getSubInstance("Cmp_XOR" + (String)i);
            propagatePortTransitionInfo(cmp_xor, "A", wr_dff_bit, "Q");
            propagatePortTransitionInfo(cmp_xor, "B", rd_dff_bit, "Q");
            cmp_xor->use();
        }

        ElectricalModel* eq_or = (ElectricalModel*)getSubInstance("EqOR");
        for(unsigned int i = 0; i < idx_bits; ++i)
        {
            ElectricalModel* cmp_xor = (ElectricalModel*)getSubInstance("Cmp_XOR" + (String)i);
            propagatePortTransitionInfo(eq_or, "In" + (String)i, cmp_xor, "Y");
        }
        eq_or->use();

        // ---- Flag ---------------------------------------------------------
        ElectricalModel* flag_dff = (ElectricalModel*)getSubInstance("Flag_DFF");
        propagatePortTransitionInfo(flag_dff, "D", eq_or, "Out");
        propagatePortTransitionInfo(flag_dff, "CK", "CK");
        flag_dff->use();

        // ---- Empty/full gating ---------------------------------------------
        ElectricalModel* full_and = (ElectricalModel*)getSubInstance("Full_AND");
        propagatePortTransitionInfo(full_and, "A", flag_dff, "Q");
        propagatePortTransitionInfo(full_and, "B", "Push");
        full_and->use();

        ElectricalModel* empty_and = (ElectricalModel*)getSubInstance("Empty_AND");
        propagatePortTransitionInfo(empty_and, "A", flag_dff, "Q");
        propagatePortTransitionInfo(empty_and, "B", "Pop");
        empty_and->use();

        // ---- Top-level output ports -----------------------------------
        // WRIdx/RDIdx are exposed as one bus port per the spec, but each
        // bit has genuinely different (position-dependent) activity -- a
        // single port can only carry one aggregate TransitionInfo. Use the
        // LSB (the highest-activity bit) as the representative value; this
        // only matters if FIFOControl is later embedded in a larger model
        // that reads this bus onward, which is out of scope here.
        propagatePortTransitionInfo("WRIdx", (ElectricalModel*)getSubInstance("WRIdx_DFF0"), "Q");
        propagatePortTransitionInfo("RDIdx", (ElectricalModel*)getSubInstance("RDIdx_DFF0"), "Q");
        propagatePortTransitionInfo("Full", full_and, "Y");
        propagatePortTransitionInfo("Empty", empty_and, "Y");

        return;
    }

} // namespace DSENT

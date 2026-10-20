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

#include "model/electrical/BroadcastHTree.h"

#include <cmath>
#include <vector>

#include "model/PortInfo.h"
#include "model/EventInfo.h"
#include "model/TransitionInfo.h"
#include "model/std_cells/StdCellLib.h"
#include "model/std_cells/StdCell.h"
#include "model/timing_graph/ElectricalLoad.h"
#include "model/timing_graph/ElectricalDelay.h"
#include "model/timing_graph/ElectricalDriver.h"
#include "model/timing_graph/ElectricalTimingTree.h"
#include "model/timing_graph/ElectricalNet.h"

namespace DSENT
{
    using std::pow;
    using std::vector;

    BroadcastHTree::BroadcastHTree(const String& instance_name_, const TechModel* tech_model_)
        : ElectricalModel(instance_name_, tech_model_)
    {
        initParameters();
        initProperties();

        m_leaf_load_ = NULL;
        m_leaf_head_driver_ = NULL;
        m_leaf_head_load_ = NULL;
    }

    BroadcastHTree::~BroadcastHTree()
    {
        clearPtrVector<StdCell>(&m_repeaters_);
        clearPtrVector<ElectricalLoad>(&m_repeater_loads_);
        clearPtrVector<ElectricalTimingTree>(&m_timing_trees_);
        clearPtrVector<StdCell>(&m_leaf_drivers_);
        delete m_leaf_load_;
        delete m_leaf_head_driver_;
        delete m_leaf_head_load_;
    }

    void BroadcastHTree::initParameters()
    {
        addParameterName("NumberLevels");
        addParameterName("NumberBits");
        addParameterName("WireLayer");
        addParameterName("WireWidthMultiplier", 1.0);
        addParameterName("WireSpacingMultiplier", 1.0);
        return;
    }

    void BroadcastHTree::initProperties()
    {
        addPropertyName("SitePitch");
        addPropertyName("TotalLoadCapPerBit");
        return;
    }

    BroadcastHTree* BroadcastHTree::clone() const
    {
        // TODO
        return NULL;
    }

    void BroadcastHTree::constructModel()
    {
        // Get parameters
        unsigned int number_levels = getParameter("NumberLevels").toUInt();
        unsigned int number_bits = getParameter("NumberBits").toUInt();
        const String& wire_layer = getParameter("WireLayer");
        double wire_width_multiplier = getParameter("WireWidthMultiplier").toDouble();
        double wire_spacing_multiplier = getParameter("WireSpacingMultiplier").toDouble();

        ASSERT(number_levels > 0, "[Error] " + getInstanceName() +
                " -> Number of levels must be > 0!");
        ASSERT(number_bits > 0, "[Error] " + getInstanceName() +
                " -> Number of bits must be > 0!");
        ASSERT(getTechModel()->isWireLayerExist(wire_layer), "[Error] " + getInstanceName() + 
                " -> Wire layer does not exist!");
        ASSERT(wire_width_multiplier >= 1.0, "[Error] " + getInstanceName() + 
                " -> Wire width multiplier must be >= 1.0!");
        ASSERT(wire_spacing_multiplier >= 1.0, "[Error] " + getInstanceName() + 
                " -> Wire spacing multiplier must be >= 1.0!");

        double wire_min_width = getTechModel()->get("Wire->" + wire_layer + "->MinWidth").toDouble();
        double wire_min_spacing = getTechModel()->get("Wire->" + wire_layer + "->MinSpacing").toDouble();

        double wire_width = wire_min_width * wire_width_multiplier;
        double wire_spacing = wire_min_spacing * wire_spacing_multiplier;

        double wire_cap_per_len = getTechModel()->calculateWireCapacitance(wire_layer, wire_width, wire_spacing, 1.0);
        double wire_res_per_len = getTechModel()->calculateWireResistance(wire_layer, wire_width, 1.0);

        getGenProperties()->set("WireWidth", wire_width);
        getGenProperties()->set("WireSpacing", wire_spacing);
        getGenProperties()->set("WireCapacitancePerLength", wire_cap_per_len);
        getGenProperties()->set("WireResistancePerLength", wire_res_per_len);

        // Create ports
        createInputPort("In", makeNetIndex(0, number_bits-1));
        createOutputPort("Out", makeNetIndex(0, number_bits-1));

        // Create connections
        createNet("InTmp");
        createNet("OutTmp");
        assignVirtualFanin("InTmp", "In");
        assignVirtualFanout("Out", "OutTmp");

        createLoad("In_Cap");
        createDelay("In_to_Out_delay");
        createDriver("Out_Ron", false); // not sizable -- see RepeatedLink.cc's identical pattern

        ElectricalLoad* in_cap = getLoad("In_Cap");
        ElectricalDelay* in_to_out_delay = getDelay("In_to_Out_delay");
        ElectricalDriver* out_ron = getDriver("Out_Ron");

        getNet("InTmp")->addDownstreamNode(in_cap);
        in_cap->addDownstreamNode(in_to_out_delay);
        // Task 2 fix: in_to_out_delay previously had NO downstream node, so
        // any crit-path walk arriving here (e.g. DFFRAM's from CK) dead-
        // ended at this tree's own internal delay node and never reached
        // "OutTmp"/"Out" -- meaning it never continued into whatever this
        // tree's Out port is portConnect'd to outside. Mirrors
        // RepeatedLink.cc's identical In_to_Out_delay -> Out_Ron -> OutTmp
        // chain exactly.
        in_to_out_delay->addDownstreamNode(out_ron);
        out_ron->addDownstreamNode(getNet("OutTmp"));

        // Init
        for(unsigned int i = 0; i < number_levels; ++i)
        {
            StdCell* repeater = getTechModel()->getStdCellLib()->createStdCell("INV", "Repeater" + (String)i);
            ElectricalLoad* repeater_load = new ElectricalLoad("RepeaterIn_Cap" + (String)i, this);
            ElectricalTimingTree* timing_tree = new ElectricalTimingTree("RepeatedLink" + (String)i, this);

            repeater->construct();
            repeater->getNet("Y")->addDownstreamNode(repeater_load);
            m_repeaters_.push_back(repeater);
            m_repeater_loads_.push_back(repeater_load);
            m_timing_trees_.push_back(timing_tree);
        }

        // Create area, power, and event results
        createElectricalAtomicResults();
        createElectricalEventResult("Send");
        addEventResult(new AtomicResult("DriveLoad"));
        addEventResult(new AtomicResult("DriveTree"));

        getEventResult("Send")->addSubResult(getEventResult("DriveLoad"), "Self", 1.0);
        getEventResult("Send")->addSubResult(getEventResult("DriveTree"), "Self", 1.0);
        return;
    }

    void BroadcastHTree::updateModel()
    {
        // Get properties
        double site_pitch = getProperty("SitePitch").toDouble();
        double total_load_cap_per_bit = getProperty("TotalLoadCapPerBit").toDouble();

        ASSERT(site_pitch > 0, "[Error] " + getInstanceName() + 
                " -> Site pitch must be > 0!");
        ASSERT(total_load_cap_per_bit >= 0.0, "[Error] " + getInstanceName() + 
                " -> Total load capacitance per bit must be >= 0!");

        // Get parameters
        unsigned int number_levels = getParameter("NumberLevels");
        unsigned int number_bits = getParameter("NumberBits");

        const String& wire_layer = getParameter("WireLayer");
        double wire_width = getGenProperties()->get("WireWidth").toDouble();
        double wire_spacing = getGenProperties()->get("WireSpacing").toDouble();
        double wire_cap_per_len = getGenProperties()->get("WireCapacitancePerLength").toDouble();
        double wire_res_per_len = getGenProperties()->get("WireResistancePerLength").toDouble();

        double leaf_load_cap = total_load_cap_per_bit / pow(2.0, (double)(number_levels-1));

        vector<double> wire_caps(number_levels, 0.0);
        vector<double> wire_ress(number_levels, 0.0);
        double wire_length = site_pitch / 2.0;
        for(unsigned int i = 0; i < number_levels; ++i)
        {
            wire_caps[i] = wire_cap_per_len * wire_length;
            wire_ress[i] = wire_res_per_len * wire_length;
            wire_length /= 2.0;
        }

        // Start sizing each stage of repeaters for a transition times. TODO: Find a heuristic about
        // how the transition time is done...place and route tools make this user-specified
        double required_transition = 40e-12;
        m_number_segments_.resize(number_levels, 1);
        for(unsigned int i = 0; i < number_levels; ++i)
        {
            Log::printLine(getInstanceName() + " -> Beginning Repeater Insertion " + (String)i);

            double transition;
            unsigned int iteration = 0;
            m_repeaters_[i]->setMinDrivingStrength();
            m_repeaters_[i]->getNet("Y")->setDistributedCap(wire_caps[i] / m_number_segments_[i]);
            m_repeaters_[i]->getNet("Y")->setDistributedRes(wire_ress[i] / m_number_segments_[i]);
            m_repeater_loads_[i]->setLoadCap(m_repeaters_[i]->getNet("A")->getTotalDownstreamCap());
            
            transition = m_timing_trees_[i]->calculateNodeTransition(m_repeaters_[i]->getNet("Y"));

            while(required_transition < transition)
            {
                Log::printLine(getInstanceName() + " -> Repeater Insertion Iteration " + (String)iteration + 
                        ": Required transition = " + (String)required_transition + 
                        ", Transition = " + (String)transition + 
                        ", Slack = " + (String)(required_transition - transition) + 
                        ", Number of repeaters = " + (String)m_number_segments_[i]);

                // Size up if transition is not met
                while(required_transition < transition)
                {
                    if(m_repeaters_[i]->hasMaxDrivingStrength())
                    {
                        break;
                    }
                    m_repeaters_[i]->increaseDrivingStrength();
                    m_repeater_loads_[i]->setLoadCap(m_repeaters_[i]->getNet("A")->getTotalDownstreamCap());
                    transition = m_timing_trees_[i]->calculateNodeTransition(m_repeaters_[i]->getNet("Y"));

                    iteration++;
                    Log::printLine(getInstanceName() + " -> Slack: " + (String)(required_transition - transition));
                }
                // Increase number of segments if thansition is not met
                if(required_transition < transition)
                {
                    m_number_segments_[i]++;
                    m_repeaters_[i]->setMinDrivingStrength();
                    m_repeaters_[i]->getNet("Y")->setDistributedCap(wire_caps[i] / m_number_segments_[i]);
                    m_repeaters_[i]->getNet("Y")->setDistributedRes(wire_ress[i] / m_number_segments_[i]);
                    m_repeater_loads_[i]->setLoadCap(m_repeaters_[i]->getNet("A")->getTotalDownstreamCap());
                    transition = m_timing_trees_[i]->calculateNodeTransition(m_repeaters_[i]->getNet("Y"));
                }
            }
            Log::printLine(getInstanceName() + " -> Repeater Insertion " + (String)i + " Ended after Iteration: " + (String)iteration + 
                    ": Required transition = " + (String)required_transition + 
                    ", Transition = " + (String)transition + 
                    ", Slack = " + (String)(required_transition - transition) + 
                    ", Number of repeaters = " + (String)m_number_segments_[i]);
        }

        // Insert inverters to ensure the transition time at the leaf
        int min_driving_strength_idx = m_repeaters_[number_levels-1]->getDrivingStrengthIdx();

        // Remove everything and rebuild again
        clearPtrVector<StdCell>(&m_leaf_drivers_);
        delete m_leaf_load_;
        delete m_leaf_head_driver_;
        delete m_leaf_head_load_;

        m_leaf_head_driver_ = getTechModel()->getStdCellLib()->createStdCell("INV", "LeafHeadDriver");
        m_leaf_head_driver_->construct();
        m_leaf_head_driver_->setDrivingStrengthIdx(min_driving_strength_idx);

        m_leaf_head_load_ = new ElectricalLoad("LeafHead_Cap", this);
        m_leaf_head_driver_->getNet("Y")->addDownstreamNode(m_leaf_head_load_);

        m_leaf_load_ = new ElectricalLoad("Leaf_Cap", this);
        m_leaf_load_->setLoadCap(leaf_load_cap);

        StdCell* inv = getTechModel()->getStdCellLib()->createStdCell("INV", "LeafDriver0");
        inv->construct();
        inv->getNet("Y")->addDownstreamNode(m_leaf_load_);
        inv->setDrivingStrengthIdx(min_driving_strength_idx);
        m_leaf_drivers_.push_back(inv);

        m_leaf_head_load_->setLoadCap(m_leaf_drivers_[0]->getNet("A")->getTotalDownstreamCap());

        // Start inserting the buffers
        ElectricalTimingTree t2("LeafHead", m_leaf_head_driver_);
        int curr_driver = 0;
        unsigned int iteration = 0;
        while(true)
        {
            ElectricalTimingTree t("LeafDriver", m_leaf_drivers_[curr_driver]);
            double transition = t.calculateNodeTransition(m_leaf_drivers_[curr_driver]->getNet("Y"));
            Log::printLine(getInstanceName() + " -> Buffer Insertion : " + (String)iteration + 
                    ": Required transition = " + (String)required_transition + 
                    ", Transition = " + (String)transition + 
                    ", Slack = " + (String)(required_transition - transition) + 
                    ", Number of buffers = " + (String)(curr_driver+1));

            // Size up the inverter at curr_driver so that it could drive the next stage
            while(required_transition < transition)
            {
                if(m_leaf_drivers_[curr_driver]->hasMaxDrivingStrength())
                {
                    const String& warning_msg = "[Warning] " + getInstanceName() + " -> Transition not met" + 
                        ": Required transition = " + (String)required_transition + 
                        ", Transition = " + (String)transition + 
                        ", Slack = " + (String)(required_transition - transition);
                    Log::printLine(std::cerr, warning_msg);
                    break;
                }
                m_leaf_drivers_[curr_driver]->increaseDrivingStrength();
                transition = t.calculateNodeTransition(m_leaf_drivers_[curr_driver]->getNet("Y"));
                iteration++;
            }
            // Add an additional inverter if the transition for the first stage does not meet the required transition
            m_leaf_head_load_->setLoadCap(m_leaf_drivers_[curr_driver]->getNet("A")->getTotalDownstreamCap());
            transition = t2.calculateNodeTransition(m_leaf_head_driver_->getNet("Y"));
            if(required_transition < transition)
            {
                inv = getTechModel()->getStdCellLib()->createStdCell("INV", "LeafDriver" + (String)(curr_driver+1));
                inv->construct();
                inv->getNet("Y")->addDownstreamNode(m_leaf_drivers_[curr_driver]->getNet("A"));
                inv->setDrivingStrengthIdx(min_driving_strength_idx);
                m_leaf_drivers_.push_back(inv);
                curr_driver++;
            }
            else
            {
                Log::printLine(getInstanceName() + " -> Buffer Insertion Ended after Iteration: " + (String)iteration + 
                        ", Number of buffers = " + (String)(curr_driver+1));
                break;
            }
        }


        // Update electrical interfaces
        getLoad("In_Cap")->setLoadCap(m_repeaters_[0]->getNet("A")->getTotalDownstreamCap());

        // In_to_Out_delay: each level is modeled as ONE representative
        // repeated segment (gate + its share of the level's distributed
        // wire RC, already set via setDistributedCap/Res above) that stands
        // in for m_number_segments_[i] identical segments cascaded in
        // series -- the same "one segment's delay x segment count" pattern
        // RepeatedLink.cc already uses for its own In_to_Out_delay. A fresh
        // performCritPathExtract() is required here: the sizing loop above
        // only ever called calculateNodeTransition(), never extract/mark,
        // so any previously marked critical path is stale (or, on a node
        // touched for the first time, unset).
        double total_delay = 0.0;
        for(unsigned int i = 0; i < number_levels; ++i)
        {
            m_timing_trees_[i]->performCritPathExtract(m_repeaters_[i]->getNet("A"));
            double seg_delay = m_timing_trees_[i]->calculateCritPathDelay(m_repeaters_[i]->getNet("A"));
            total_delay += seg_delay * m_number_segments_[i];
        }

        // Leaf buffer stage, added after the per-level tree in series:
        // LeafHeadDriver's own stage delay (driving m_leaf_head_load_, a
        // capacitive stand-in sized to match the real chain's input
        // capacitance -- same modeling shortcut as the per-level
        // repeater_load_ above) PLUS the real, fully-connected
        // m_leaf_drivers_ chain's delay. The chain's own internal links
        // (LeafDriverK.Y -> LeafDriver(K-1).A, ..., LeafDriver1.Y ->
        // LeafDriver0.A, LeafDriver0.Y -> m_leaf_load_) are real net
        // connections (addDownstreamNode), not capacitive stand-ins, so a
        // SINGLE extraction starting from the last-added driver's A net
        // (m_leaf_drivers_.back(), the stage closest to the head) already
        // recursively walks every intermediate stage down to LeafDriver0 in
        // one pass. Extracting from every element individually and summing
        // would double-count each downstream stage's delay repeatedly.
        ElectricalTimingTree leaf_head_tree("LeafHeadDelay", m_leaf_head_driver_);
        leaf_head_tree.performCritPathExtract(m_leaf_head_driver_->getNet("A"));
        total_delay += leaf_head_tree.calculateCritPathDelay(m_leaf_head_driver_->getNet("A"));

        ElectricalTimingTree leaf_chain_tree("LeafChainDelay", m_leaf_drivers_.back());
        leaf_chain_tree.performCritPathExtract(m_leaf_drivers_.back()->getNet("A"));
        total_delay += leaf_chain_tree.calculateCritPathDelay(m_leaf_drivers_.back()->getNet("A"));

        getDelay("In_to_Out_delay")->setDelay(total_delay);
        // Out_Ron's own resistance, for whatever's portConnect'd downstream
        // of this tree's "Out" port to size correctly against -- mirrors
        // RepeatedLink.cc's identical setOutputRes call. LeafDriver0 (index
        // 0, closest to the real load) is the final stage actually driving
        // the tree's output; no additional wire term applies here (unlike
        // RepeatedLink, nothing further is distributed past the leaf load).
        getDriver("Out_Ron")->setOutputRes(m_leaf_drivers_[0]->getDriver("Y_Ron")->getOutputRes());

        // Reset all the atomic results to 0 before start updating new results
        resetElectricalAtomicResults();

        // Update area, power results
        double wire_area = 0.0;
        wire_length = site_pitch / 2.0;
        unsigned int number_branches = 1;
        for(unsigned int i = 0; i < number_levels; ++i)
        {
            wire_area += wire_length * (wire_width + wire_spacing) * number_branches * number_bits;
            addElecticalAtomicResultValues(m_repeaters_[i], m_number_segments_[i] * number_branches * number_bits);
            wire_length /= 2.0;
            number_branches *= 2;
        }
        number_branches = (unsigned int)pow(2.0, (double)number_levels-1);
        addElecticalAtomicResultValues(m_leaf_head_driver_, number_branches * number_bits);
        for(unsigned int i = 0; i < m_leaf_drivers_.size(); ++i)
        {
            addElecticalAtomicResultValues(m_leaf_drivers_[i], number_branches * number_bits);
        }
        addElecticalWireAtomicResultValue(wire_layer, wire_area);

        return;
    }

    void BroadcastHTree::useModel()
    {
        unsigned int number_bits = getParameter("NumberBits").toUInt();
        unsigned int number_levels = getParameter("NumberLevels").toUInt();

        // Update the transition information for the modeled repeaters
        // Since we only modeled one repeater. So the transition information for 0->0 and 1->1 
        // is averaged out
        const TransitionInfo& trans_In = getInputPort("In")->getTransitionInfo();
        double average_static_transition = (trans_In.getNumberTransitions00() + trans_In.getNumberTransitions11()) / 2.0;
        TransitionInfo mod_trans_In(average_static_transition, trans_In.getNumberTransitions01(), average_static_transition);

        // Propagate the transition information
        propagateTransitionInfo();

        // Update leakage and event
        double energy = 0.0;
        double power = 0.0;
        unsigned int number_branches = 1;
        for(unsigned int i = 0; i < number_levels; ++i)
        {
            assignPortTransitionInfo(m_repeaters_[i], "A", mod_trans_In);
            m_repeaters_[i]->use();
            power += m_repeaters_[i]->getNddPowerResult("Leakage")->calculateSum() * m_number_segments_[i] * number_branches;
            energy += m_repeaters_[i]->getEventResult("INV")->calculateSum() * m_number_segments_[i] * number_branches;
            number_branches *= 2;
        }
        energy *= number_bits;
        getEventResult("DriveTree")->setValue(energy);

        energy = 0.0;
        assignPortTransitionInfo(m_leaf_head_driver_, "A", mod_trans_In);
        m_leaf_head_driver_->use();
        number_branches = (unsigned int)pow(2.0, (double)number_levels-1);
        power += m_leaf_head_driver_->getNddPowerResult("Leakage")->calculateSum() * number_branches;
        energy += m_leaf_head_driver_->getEventResult("INV")->calculateSum() * number_branches;
        for(unsigned int i = 0; i < m_leaf_drivers_.size(); ++i)
        {
            assignPortTransitionInfo(m_leaf_drivers_[i], "A", mod_trans_In);
            m_leaf_drivers_[i]->use();
            power += m_leaf_drivers_[i]->getNddPowerResult("Leakage")->calculateSum() * number_branches;
            energy += m_leaf_drivers_[i]->getEventResult("INV")->calculateSum() * number_branches;
        }
        power *= number_bits;
        energy *= number_bits;
        getEventResult("DriveLoad")->setValue(energy);
        getNddPowerResult("Leakage")->setValue(power);

        return;
    }

    void BroadcastHTree::propagateTransitionInfo()
    {
        propagatePortTransitionInfo("Out", "In");
        return;
    }
} // namespace DSENT


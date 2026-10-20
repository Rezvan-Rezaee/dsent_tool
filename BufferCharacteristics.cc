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

#include "BufferCharacteristics.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <map>
#include <vector>

#include "util/CommonType.h"
#include "util/Result.h"
#include "tech/TechModel.h"
#include "model/Model.h"
#include "model/ElectricalModel.h"
#include "model/ModelGen.h"
#include "model/std_cells/StdCellLib.h"
#include "model/timing_graph/ElectricalNet.h"
#include "model/timing_graph/ElectricalTimingNode.h"
#include "model/timing_graph/ElectricalTimingTree.h"

namespace DSENT
{
    namespace BufferCharacteristics
    {
        using std::cerr;
        using std::cout;
        using std::endl;
        using std::map;
        using std::ofstream;
        using std::vector;

        //---------------------------------------------------------------
        // CLI argument parsing helpers
        //---------------------------------------------------------------
        static map<String, String> parseArgs(int argc, char* argv[], int start_index)
        {
            map<String, String> args;
            for(int i = start_index; i < argc; i += 2)
            {
                String key = argv[i];
                if((key.size() < 2) || (key[0] != '-') || (key[1] != '-'))
                {
                    cerr << "[Error] Expected a flag starting with -- but got: " << key << endl;
                    std::exit(1);
                }
                key = String(key.substr(2));
                if((i + 1) >= argc)
                {
                    cerr << "[Error] Missing value for flag --" << key << endl;
                    std::exit(1);
                }
                args[key] = argv[i + 1];
            }
            return args;
        }

        static String getArgOrDefault(const map<String, String>& args_, const String& key_, const String& default_)
        {
            map<String, String>::const_iterator it = args_.find(key_);
            return (it != args_.end()) ? it->second : default_;
        }

        static String requireArg(const map<String, String>& args_, const String& key_)
        {
            map<String, String>::const_iterator it = args_.find(key_);
            if(it == args_.end())
            {
                cerr << "[Error] Missing required flag --" << key_ << endl;
                std::exit(1);
            }
            return it->second;
        }

        static vector<double> parseDoubleList(const String& s_)
        {
            vector<String> parts = s_.split(",");
            vector<double> ret;
            for(unsigned int i = 0; i < parts.size(); ++i)
            {
                ret.push_back(parts[i].toDouble());
            }
            return ret;
        }

        static vector<unsigned int> parseUIntList(const String& s_)
        {
            vector<String> parts = s_.split(",");
            vector<unsigned int> ret;
            for(unsigned int i = 0; i < parts.size(); ++i)
            {
                ret.push_back(parts[i].toUInt());
            }
            return ret;
        }

        static String joinUIntList(const vector<unsigned int>& values_)
        {
            String ret = "";
            for(unsigned int i = 0; i < values_.size(); ++i)
            {
                if(i > 0)
                {
                    ret += ",";
                }
                ret += (String)values_[i];
            }
            return ret;
        }

        //---------------------------------------------------------------
        // JSON writing helpers (hand-rolled -- fixed schema, no parser needed)
        //---------------------------------------------------------------
        static void writeJsonString(std::ostream& os_, const String& s_)
        {
            os_ << '"';
            for(unsigned int i = 0; i < s_.size(); ++i)
            {
                char c = s_[i];
                if((c == '"') || (c == '\\'))
                {
                    os_ << '\\' << c;
                }
                else if(c == '\n')
                {
                    os_ << "\\n";
                }
                else
                {
                    os_ << c;
                }
            }
            os_ << '"';
        }

        static void writeJsonNumber(std::ostream& os_, double value_)
        {
            if(std::isnan(value_) || std::isinf(value_))
            {
                os_ << "null";
                return;
            }
            char buf[64];
            snprintf(buf, sizeof(buf), "%.10g", value_);
            os_ << buf;
        }

        static void writeJsonNumberOrNull(std::ostream& os_, double value_, bool present_)
        {
            if(!present_)
            {
                os_ << "null";
            }
            else
            {
                writeJsonNumber(os_, value_);
            }
        }

        static void writeJsonBool(std::ostream& os_, bool value_)
        {
            os_ << (value_ ? "true" : "false");
        }

        //---------------------------------------------------------------
        // Per-configuration result row
        //---------------------------------------------------------------
        struct RowResult
        {
            // inputs
            String tech_name;
            double frequency_hz;
            unsigned int num_vns;
            vector<unsigned int> vcs_per_vn;
            vector<unsigned int> bufs_per_vc;
            unsigned int flit_width;
            String buffer_model;
            bool timing_opt;
            // Phase 5
            double write_rate;
            double read_rate;
            double flit_toggle_rate;
            // Phase 6
            double write_vc_switch_rate;
            double read_vc_switch_rate;

            // derived
            unsigned int total_entries;
            unsigned int addr_bits;
            unsigned int total_dff_bits;

            // Becker Table 6.1 (static management column) register counts --
            // V = total_number_vcs, idx_bits = ceil(log2(slice_depth)).
            // "model" values are counted from the ACTUAL constructed object
            // graph (hasSubInstance existence checks), not re-derived from
            // the same formula used to predict them -- see computeRow().
            unsigned int num_vcs;
            unsigned int idx_bits;
            unsigned int cnt_bits;
            unsigned int input_ctrl_regs_model;
            unsigned int output_ctrl_regs_model;
            unsigned int input_ctrl_regs_becker;
            unsigned int output_ctrl_regs_becker;
            bool input_ctrl_regs_match_becker;
            bool output_ctrl_regs_match_becker;

            // area (Area->Active only)
            double area_total, area_ram, area_ram_decoder, area_ram_mux,
                   area_ram_dff_array, area_ram_clockgate, area_rdaddr_dff;

            // leakage (NddPower->Leakage)
            double leak_total, leak_ram, leak_ram_decoder, leak_ram_mux,
                   leak_ram_dff_array, leak_ram_clockgate, leak_rdaddr_dff;

            // energy -- only valid when have_energy is true (blanked to null
            // in the JSON output when timing could not be met)
            bool have_energy;
            double e_write, e_write_decoder, e_write_mux, e_write_dff_array,
                   e_write_clockgate, e_write_rdaddr_dff;
            double e_read, e_read_decoder, e_read_mux, e_read_dff_array,
                   e_read_clockgate, e_read_rdaddr_dff;
            double e_idle;

            // timing (s)
            double t_write_data_setup, t_write_clock_path, t_read_access,
                   t_read_addr_path, t_critical, slack;
            bool timing_met;
        };

        static double queryResult(Model* model_, const String& type_, const String& sub_field_)
        {
            const Result* result = (const Result*)model_->processQuery(type_, sub_field_);
            return result->calculateSum();
        }

        // Read a child's ALREADY-computed event Result without re-triggering
        // use(event_name_) on it. This matters because Result objects are
        // shared by live pointer between a parent's aggregation and the
        // child that produced them: querying Energy on the PARENT (e.g.
        // RouterInputPort's "WriteBuffer") cascades a use() call down through
        // every descendant with the REAL propagated transition context (e.g.
        // a gated, reduced-probability CK at a given DFF), computing and
        // caching each child's own event Result along the way. Calling
        // processQuery("Energy", ...) directly ON a child afterward would
        // instead re-run use(event_name) using THAT child's own decontext-
        // ualized default EventInfo (e.g. DFFQ's own "DFFD" assumes a full,
        // ungated CK toggle) -- overwriting the correct cascaded value with a
        // wrong, generally larger one. Reading getEventResult() directly
        // sidesteps that by using whatever was last cached by the parent's
        // cascade.
        static double cachedEventEnergy(Model* model_, const String& event_name_)
        {
            return model_->getEventResult(event_name_)->calculateSum();
        }

        //---------------------------------------------------------------
        // Build, optimize, measure, and query a single RouterInputPort
        // configuration
        //---------------------------------------------------------------
        static RowResult computeRow(TechModel* tech_model_, const String& tech_name_,
            double frequency_hz_, unsigned int flit_width_, unsigned int num_vns_,
            const vector<unsigned int>& vcs_, const vector<unsigned int>& bufs_,
            const String& buffer_model_, bool timing_opt_,
            double write_rate_, double read_rate_, double flit_toggle_rate_,
            double write_vc_switch_rate_, double read_vc_switch_rate_)
        {
            RowResult row;
            row.tech_name = tech_name_;
            row.frequency_hz = frequency_hz_;
            row.num_vns = num_vns_;
            row.vcs_per_vn = vcs_;
            row.bufs_per_vc = bufs_;
            row.flit_width = flit_width_;
            row.buffer_model = buffer_model_;
            row.timing_opt = timing_opt_;
            row.write_rate = write_rate_;
            row.read_rate = read_rate_;
            row.flit_toggle_rate = flit_toggle_rate_;
            row.write_vc_switch_rate = write_vc_switch_rate_;
            row.read_vc_switch_rate = read_vc_switch_rate_;

            unsigned int total_entries = 0;
            for(unsigned int i = 0; i < vcs_.size(); ++i)
            {
                total_entries += vcs_[i] * bufs_[i];
            }
            unsigned int addr_bits = (total_entries > 1) ? (unsigned int)ceil(log2((double)total_entries)) : 0;
            unsigned int number_bits = flit_width_;
            // Per-VC slice index width (RouterInputPort's own "SliceIndexBits"
            // formula) -- NOT addr_bits. addr_bits also counts the constant-
            // tied VC-select bits above the slice, which have no real flops.
            unsigned int idx_bits = (bufs_[0] > 1) ? (unsigned int)ceil(log2((double)bufs_[0])) : 0;

            row.total_entries = total_entries;
            row.addr_bits = addr_bits;
            row.total_dff_bits = total_entries * flit_width_;

            String vcs_csv = joinUIntList(vcs_);
            String bufs_csv = joinUIntList(bufs_);

            ElectricalModel* rip = (ElectricalModel*)ModelGen::createModel(
                "RouterInputPort", "RouterInputPort", tech_model_);
            rip->setParameter("NumberVirtualNetworks", (String)num_vns_);
            rip->setParameter("NumberVirtualChannelsPerVirtualNetwork", vcs_csv);
            rip->setParameter("NumberBuffersPerVirtualChannel", bufs_csv);
            rip->setParameter("NumberBitsPerFlit", (String)flit_width_);
            rip->setParameter("BufferModel", buffer_model_);
            rip->setParameter("WriteRate", (String)write_rate_);
            rip->setParameter("ReadRate", (String)read_rate_);
            rip->setParameter("FlitToggleRate", (String)flit_toggle_rate_);
            rip->setParameter("WriteVCSwitchRate", (String)write_vc_switch_rate_);
            rip->setParameter("ReadVCSwitchRate", (String)read_vc_switch_rate_);
            rip->construct();
            rip->update();

            ElectricalModel* ram = (ElectricalModel*)rip->getSubInstance("RAM");
            ElectricalModel* dec = (ElectricalModel*)ram->getSubInstance("Dec");
            ElectricalModel* mux = (ElectricalModel*)ram->getSubInstance("Mux");
            ElectricalModel* nand2cg0 = (ElectricalModel*)ram->getSubInstance("NAND2_CKGate0");
            ElectricalModel* invcg0 = (ElectricalModel*)ram->getSubInstance("INV_CKGate0");

            vector<ElectricalModel*> dffs(total_entries, (ElectricalModel*)NULL);
            vector<ElectricalModel*> nand2cg1s(total_entries, (ElectricalModel*)NULL);
            vector<ElectricalModel*> invcg1s(total_entries, (ElectricalModel*)NULL);
            for(unsigned int i = 0; i < total_entries; ++i)
            {
                dffs[i] = (ElectricalModel*)ram->getSubInstance("DFF_" + (String)i);
                nand2cg1s[i] = (ElectricalModel*)ram->getSubInstance("NAND2_CKGate1_" + (String)i);
                invcg1s[i] = (ElectricalModel*)ram->getSubInstance("INV_CKGate1_" + (String)i);
            }
            // RouterInputPort's Phase 2 rework replaced the old flat
            // RDAddr_DFF bank with one FIFOControl per VC; the equivalent
            // read-pointer register now lives inside FIFO0 as RDIdx_DFF0..
            // idx_bits-1 (VC0 is the structurally-representative VC, see
            // RouterInputPort.cc's own DSENT-ASSUMPTION comment).
            ElectricalModel* fifo0 = (ElectricalModel*)rip->getSubInstance("FIFO0");
            vector<ElectricalModel*> rdaddr_dffs(idx_bits, (ElectricalModel*)NULL);
            for(unsigned int j = 0; j < idx_bits; ++j)
            {
                rdaddr_dffs[j] = (ElectricalModel*)fifo0->getSubInstance("RDIdx_DFF" + (String)j);
            }

            // ---- Part B: Becker Table 6.1 (static column) register audit ----
            // Input side: count FIFO0's ACTUAL WRIdx_DFF/RDIdx_DFF/Flag_DFF
            // sub-instances via existence checks (not just re-deriving the
            // formula), then scale by the real, structurally-built VC count.
            unsigned int num_vcs = 0;
            for(unsigned int i = 0; i < vcs_.size(); ++i)
            {
                num_vcs += vcs_[i];
            }
            bool input_struct_ok = fifo0->hasSubInstance("Flag_DFF");
            for(unsigned int i = 0; i < idx_bits; ++i)
            {
                input_struct_ok = input_struct_ok
                    && fifo0->hasSubInstance("WRIdx_DFF" + (String)i)
                    && fifo0->hasSubInstance("RDIdx_DFF" + (String)i);
            }
            input_struct_ok = input_struct_ok
                && !fifo0->hasSubInstance("WRIdx_DFF" + (String)idx_bits)
                && !fifo0->hasSubInstance("RDIdx_DFF" + (String)idx_bits);
            unsigned int input_ctrl_regs_per_vc = input_struct_ok ? (2 * idx_bits + 1) : 0;
            unsigned int input_ctrl_regs_model = num_vcs * input_ctrl_regs_per_vc;

            // Output side: OutputVCState is only ever built from Router.cc in
            // the full model, but it's independently instantiable (Part A),
            // so build one representative standalone instance here purely to
            // count its register structure -- same slice depth, discarded
            // immediately after.
            ElectricalModel* ovc = (ElectricalModel*)ModelGen::createModel(
                "OutputVCState", "OutputVCState_audit", tech_model_);
            ovc->setParameter("NumberEntries", (String)bufs_[0]);
            ovc->construct();
            unsigned int cnt_bits = ovc->getGenProperties()->get("CntBits").toUInt();
            bool output_struct_ok = ovc->hasSubInstance("Busy_DFF") && ovc->hasSubInstance("Empty_DFF");
            for(unsigned int i = 0; i < cnt_bits; ++i)
            {
                output_struct_ok = output_struct_ok && ovc->hasSubInstance("Cnt_DFF" + (String)i);
            }
            output_struct_ok = output_struct_ok && !ovc->hasSubInstance("Cnt_DFF" + (String)cnt_bits);
            unsigned int output_ctrl_regs_per_vc = output_struct_ok ? (cnt_bits + 2) : 0;
            unsigned int output_ctrl_regs_model = num_vcs * output_ctrl_regs_per_vc;
            delete ovc;

            // Becker's Table 6.1 formulas, exactly as published (input side:
            // "VC pointers = 2*V*ceil(log2(B/V))" + "Flags = V"; output side:
            // "Busy/empty = 2*V" + "VC counters = V*ceil(log2(B/V))"). NOTE:
            // OutputVCState's own cnt_bits is ceil(log2(NumberEntries))+1, one
            // bit wider than Becker's raw ceil(log2(B/V)) -- see the
            // DSENT-ASSUMPTION comment in OutputVCState.cc constructModel().
            // That is an intentional, documented design choice (representing
            // the full downstream depth without wraparound), not a bug, but
            // it DOES mean output_ctrl_regs_model will legitimately exceed
            // output_ctrl_regs_becker by exactly num_vcs registers. Report
            // it, do not hide it or "fix" it away.
            unsigned int input_ctrl_regs_becker = num_vcs * (2 * idx_bits + 1);
            unsigned int output_ctrl_regs_becker = num_vcs * (idx_bits + 2);

            row.num_vcs = num_vcs;
            row.idx_bits = idx_bits;
            row.cnt_bits = cnt_bits;
            row.input_ctrl_regs_model = input_ctrl_regs_model;
            row.output_ctrl_regs_model = output_ctrl_regs_model;
            row.input_ctrl_regs_becker = input_ctrl_regs_becker;
            row.output_ctrl_regs_becker = output_ctrl_regs_becker;
            row.input_ctrl_regs_match_becker = (input_ctrl_regs_model == input_ctrl_regs_becker);
            row.output_ctrl_regs_match_becker = (output_ctrl_regs_model == output_ctrl_regs_becker);

            // DFFRAM's per-bit "D" connect loop re-issues portConnect onto the
            // same single-bit DFFQ "D" pin for every bit index, so only the
            // last-connected bit index actually has real downstream fanout.
            // Find it empirically rather than assuming a fixed index.
            int write_bit_index = -1;
            for(unsigned int n = 0; n < flit_width_; ++n)
            {
                ElectricalNet* net = ram->getNet("In", makeNetIndex(n));
                if(!net->getDownstreamNodes()->empty())
                {
                    write_bit_index = (int)n;
                    break;
                }
            }

            double period = 1.0 / frequency_hz_;
            bool timing_met = true;

            if(timing_opt_)
            {
                ElectricalTimingTree opt_tree("RouterInputPort", rip);

                // Covers the clock-gating chain, DFF CK-to-Q, Mux, Out, and the
                // RDAddr_DFF pointer-update chain -- all downstream of the
                // top-level CK port.
                bool ok_ck = opt_tree.performTimingOpt(rip->getNet("CK"), period);

                // Covers the write-data-setup path (In -> D -> D_Setup).
                bool ok_in = true;
                if(write_bit_index >= 0)
                {
                    ok_in = opt_tree.performTimingOpt(
                        rip->getNet("FlitIn", makeNetIndex(write_bit_index)), period);
                }

                // Covers the address-to-output path (RDAddr -> Mux Sel -> Out),
                // one independent sub-graph per address bit.
                bool ok_addr = true;
                for(unsigned int j = 0; j < addr_bits; ++j)
                {
                    bool ok = opt_tree.performTimingOpt(ram->getNet("RDAddr" + (String)j), period);
                    ok_addr = ok_addr && ok;
                }

                timing_met = ok_ck && ok_in && ok_addr;
            }

            // Timing measurement -- always performed (post-optimization, if
            // any), so the four named paths are reported for every row.
            ElectricalTimingTree tree("RouterInputPort", rip);

            double t_write_data_setup = 0.0;
            if(write_bit_index >= 0)
            {
                // Naturally terminates at D_Setup_delay -- no truncation needed.
                t_write_data_setup = tree.performCritPathExtract(
                    ram->getNet("In", makeNetIndex(write_bit_index)));
            }

            double t_write_clock_path = 0.0;
            {
                // Truncate the walk right after each DFF's own CK pin (stopping
                // before the DFF's internal CK-to-Q propagation) so the CK ->
                // gating-chain -> DFF-CK-pin segment can be measured on its own.
                vector<ElectricalNet*> dff_ck_nets(total_entries, (ElectricalNet*)NULL);
                for(unsigned int i = 0; i < total_entries; ++i)
                {
                    dff_ck_nets[i] = dffs[i]->getNet("CK");
                }
                for(unsigned int i = 0; i < total_entries; ++i)
                {
                    vector<ElectricalTimingNode*>* down = dff_ck_nets[i]->getDownstreamNodes();
                    for(unsigned int k = 0; k < down->size(); ++k)
                    {
                        down->at(k)->setFalsePath(true);
                    }
                }
                t_write_clock_path = tree.performCritPathExtract(ram->getNet("CK"));
                // Reset immediately -- these flags are shared mutable node
                // state reused by every other measurement below.
                for(unsigned int i = 0; i < total_entries; ++i)
                {
                    vector<ElectricalTimingNode*>* down = dff_ck_nets[i]->getDownstreamNodes();
                    for(unsigned int k = 0; k < down->size(); ++k)
                    {
                        down->at(k)->setFalsePath(false);
                    }
                }
            }

            double t_read_access = 0.0;
            for(unsigned int i = 0; i < total_entries; ++i)
            {
                // Natural walk: CK_Cap -> CK_to_Q_delay -> Q_Ron -> Q -> Mux -> Out.
                double d = tree.performCritPathExtract(dffs[i]->getNet("CK"));
                if(d > t_read_access)
                {
                    t_read_access = d;
                }
            }

            double t_read_addr_path = 0.0;
            for(unsigned int j = 0; j < addr_bits; ++j)
            {
                // Natural walk: Mux.Sel_j -> ... -> Out.
                double d = tree.performCritPathExtract(ram->getNet("RDAddr" + (String)j));
                if(d > t_read_addr_path)
                {
                    t_read_addr_path = d;
                }
            }

            double t_critical = std::max(std::max(t_write_data_setup, t_write_clock_path),
                                          std::max(t_read_access, t_read_addr_path));
            double slack = period - t_critical;

            row.t_write_data_setup = t_write_data_setup;
            row.t_write_clock_path = t_write_clock_path;
            row.t_read_access = t_read_access;
            row.t_read_addr_path = t_read_addr_path;
            row.t_critical = t_critical;
            row.slack = slack;
            row.timing_met = timing_met;

            // Evaluate exactly once, after all timing-optimization resizing is
            // complete, so Energy/NddPower results reflect the final sizes.
            rip->evaluate();

            // --- Area (Area->Active only -- never sum with Metal1Wire) ---
            row.area_ram_decoder = queryResult(dec, "Area", "Active") * 1.0;
            double area_mux_own = queryResult(mux, "Area", "Active");
            row.area_ram_mux = area_mux_own * number_bits;
            double area_dff_sum = 0.0;
            for(unsigned int i = 0; i < total_entries; ++i)
            {
                area_dff_sum += queryResult(dffs[i], "Area", "Active");
            }
            row.area_ram_dff_array = area_dff_sum * number_bits;
            double area_cg = queryResult(nand2cg0, "Area", "Active") * 1.0
                            + queryResult(invcg0, "Area", "Active") * 1.0;
            for(unsigned int i = 0; i < total_entries; ++i)
            {
                area_cg += queryResult(nand2cg1s[i], "Area", "Active") * 1.0
                         + queryResult(invcg1s[i], "Area", "Active") * 1.0;
            }
            row.area_ram_clockgate = area_cg;
            double area_rdaddr = 0.0;
            for(unsigned int j = 0; j < idx_bits; ++j)
            {
                area_rdaddr += queryResult(rdaddr_dffs[j], "Area", "Active") * 1.0;
            }
            row.area_rdaddr_dff = area_rdaddr;
            row.area_ram = queryResult(ram, "Area", "Active");
            row.area_total = queryResult(rip, "Area", "Active");

            // --- Leakage (NddPower->Leakage, same hierarchy split as area) ---
            row.leak_ram_decoder = queryResult(dec, "NddPower", "Leakage") * 1.0;
            double leak_mux_own = queryResult(mux, "NddPower", "Leakage");
            row.leak_ram_mux = leak_mux_own * number_bits;
            double leak_dff_sum = 0.0;
            for(unsigned int i = 0; i < total_entries; ++i)
            {
                leak_dff_sum += queryResult(dffs[i], "NddPower", "Leakage");
            }
            row.leak_ram_dff_array = leak_dff_sum * number_bits;
            double leak_cg = queryResult(nand2cg0, "NddPower", "Leakage") * 1.0
                            + queryResult(invcg0, "NddPower", "Leakage") * 1.0;
            for(unsigned int i = 0; i < total_entries; ++i)
            {
                leak_cg += queryResult(nand2cg1s[i], "NddPower", "Leakage") * 1.0
                         + queryResult(invcg1s[i], "NddPower", "Leakage") * 1.0;
            }
            row.leak_ram_clockgate = leak_cg;
            double leak_rdaddr = 0.0;
            for(unsigned int j = 0; j < idx_bits; ++j)
            {
                leak_rdaddr += queryResult(rdaddr_dffs[j], "NddPower", "Leakage") * 1.0;
            }
            row.leak_rdaddr_dff = leak_rdaddr;
            row.leak_ram = queryResult(ram, "NddPower", "Leakage");
            row.leak_total = queryResult(rip, "NddPower", "Leakage");

            // --- Energy (already C*Vdd^2 per 0->1 transition -- no 1/2 factor) ---
            row.have_energy = timing_met;
            if(timing_met)
            {
                // Trigger the WriteBuffer cascade on the PARENT once; this is
                // what actually recomputes every descendant's event Result
                // under the real (gated) Write transition context. Then read
                // the write-related children's now-cached values directly
                // (cachedEventEnergy), without re-triggering them.
                row.e_write = queryResult(rip, "Energy", "WriteBuffer");
                row.e_write_decoder = cachedEventEnergy(dec, "Decode") * 1.0;
                double e_cg_w = cachedEventEnergy(nand2cg0, "NAND2") * 1.0
                              + cachedEventEnergy(invcg0, "INV") * 1.0;
                for(unsigned int i = 0; i < total_entries; ++i)
                {
                    e_cg_w += cachedEventEnergy(nand2cg1s[i], "NAND2") * 1.0
                            + cachedEventEnergy(invcg1s[i], "INV") * 1.0;
                }
                row.e_write_clockgate = e_cg_w;
                double e_dff_w = 0.0;
                for(unsigned int i = 0; i < total_entries; ++i)
                {
                    e_dff_w += cachedEventEnergy(dffs[i], "DFFD")
                             + cachedEventEnergy(dffs[i], "DFFQ")
                             + cachedEventEnergy(dffs[i], "CK");
                }
                row.e_write_dff_array = e_dff_w * number_bits;
                // Mux/RDAddr_DFF are not part of DFFRAM's Write event or
                // RouterInputPort's WriteBuffer event -- report faithfully as 0.
                row.e_write_mux = 0.0;
                row.e_write_rdaddr_dff = 0.0;

                // Trigger the ReadBuffer cascade; this overwrites the cached
                // values on shared children (e.g. Mux) with the Read
                // transition context, so read it back out immediately after.
                row.e_read = queryResult(rip, "Energy", "ReadBuffer");
                row.e_read_mux = cachedEventEnergy(mux, "Mux") * number_bits;
                // DFFRAM's own Read event only sums Mux (the per-entry DFFQ
                // contribution is commented out in DFFRAM.cc) -- report the
                // pre-existing zeros faithfully, not something this feature fixes.
                row.e_read_decoder = 0.0;
                row.e_read_clockgate = 0.0;
                row.e_read_dff_array = 0.0;
                double e_rdaddr_r = 0.0;
                for(unsigned int j = 0; j < idx_bits; ++j)
                {
                    e_rdaddr_r += cachedEventEnergy(rdaddr_dffs[j], "DFFD")
                                + cachedEventEnergy(rdaddr_dffs[j], "DFFQ")
                                + cachedEventEnergy(rdaddr_dffs[j], "CK");
                }
                row.e_read_rdaddr_dff = e_rdaddr_r * 1.0;

                row.e_idle = queryResult(rip, "Energy", "Idle");
            }
            else
            {
                row.e_write = row.e_write_decoder = row.e_write_mux = row.e_write_dff_array
                    = row.e_write_clockgate = row.e_write_rdaddr_dff = 0.0;
                row.e_read = row.e_read_decoder = row.e_read_mux = row.e_read_dff_array
                    = row.e_read_clockgate = row.e_read_rdaddr_dff = 0.0;
                row.e_idle = 0.0;
            }

            delete rip;
            return row;
        }

        //---------------------------------------------------------------
        // JSON row / array emission
        //---------------------------------------------------------------
        static void writeEnergyPowerField(std::ostream& os_, const String& key_,
            double energy_value_, double frequency_hz_, bool present_, bool is_last_)
        {
            os_ << "\"" << key_ << "\": ";
            writeJsonNumberOrNull(os_, present_ ? (energy_value_ * frequency_hz_) : 0.0, present_);
            if(!is_last_)
            {
                os_ << ", ";
            }
        }

        static void writeRow(std::ostream& os_, const RowResult& r_)
        {
            os_ << "  {\n";

            os_ << "    \"inputs\": {";
            os_ << "\"tech_name\": ";
            writeJsonString(os_, r_.tech_name);
            os_ << ", \"frequency_hz\": ";
            writeJsonNumber(os_, r_.frequency_hz);
            os_ << ", \"num_vns\": " << r_.num_vns;
            os_ << ", \"vcs_per_vn\": [";
            for(unsigned int i = 0; i < r_.vcs_per_vn.size(); ++i)
            {
                if(i > 0) os_ << ", ";
                os_ << r_.vcs_per_vn[i];
            }
            os_ << "], \"bufs_per_vc\": [";
            for(unsigned int i = 0; i < r_.bufs_per_vc.size(); ++i)
            {
                if(i > 0) os_ << ", ";
                os_ << r_.bufs_per_vc[i];
            }
            os_ << "], \"flit_width\": " << r_.flit_width;
            os_ << ", \"buffer_model\": ";
            writeJsonString(os_, r_.buffer_model);
            os_ << ", \"timing_opt\": ";
            writeJsonBool(os_, r_.timing_opt);
            os_ << ", \"write_rate\": "; writeJsonNumber(os_, r_.write_rate);
            os_ << ", \"read_rate\": "; writeJsonNumber(os_, r_.read_rate);
            os_ << ", \"flit_toggle_rate\": "; writeJsonNumber(os_, r_.flit_toggle_rate);
            os_ << ", \"write_vc_switch_rate\": "; writeJsonNumber(os_, r_.write_vc_switch_rate);
            os_ << ", \"read_vc_switch_rate\": "; writeJsonNumber(os_, r_.read_vc_switch_rate);
            os_ << "},\n";

            os_ << "    \"derived\": {";
            os_ << "\"total_entries\": " << r_.total_entries;
            os_ << ", \"addr_bits\": " << r_.addr_bits;
            os_ << ", \"total_dff_bits\": " << r_.total_dff_bits;
            os_ << "},\n";

            os_ << "    \"registers\": {";
            os_ << "\"num_vcs\": " << r_.num_vcs;
            os_ << ", \"idx_bits\": " << r_.idx_bits;
            os_ << ", \"cnt_bits\": " << r_.cnt_bits;
            os_ << ", \"storage_registers\": " << r_.total_dff_bits;
            os_ << ", \"input_ctrl_regs_model\": " << r_.input_ctrl_regs_model;
            os_ << ", \"output_ctrl_regs_model\": " << r_.output_ctrl_regs_model;
            os_ << ", \"total_ctrl_regs_model\": " << (r_.input_ctrl_regs_model + r_.output_ctrl_regs_model);
            os_ << ", \"input_ctrl_regs_becker\": " << r_.input_ctrl_regs_becker;
            os_ << ", \"output_ctrl_regs_becker\": " << r_.output_ctrl_regs_becker;
            os_ << ", \"total_ctrl_regs_becker\": " << (r_.input_ctrl_regs_becker + r_.output_ctrl_regs_becker);
            os_ << ", \"input_ctrl_regs_match_becker\": "; writeJsonBool(os_, r_.input_ctrl_regs_match_becker);
            os_ << ", \"output_ctrl_regs_match_becker\": "; writeJsonBool(os_, r_.output_ctrl_regs_match_becker);
            os_ << "},\n";

            os_ << "    \"area\": {";
            os_ << "\"area_total\": "; writeJsonNumber(os_, r_.area_total);
            os_ << ", \"area_ram\": "; writeJsonNumber(os_, r_.area_ram);
            os_ << ", \"area_ram_decoder\": "; writeJsonNumber(os_, r_.area_ram_decoder);
            os_ << ", \"area_ram_mux\": "; writeJsonNumber(os_, r_.area_ram_mux);
            os_ << ", \"area_ram_dff_array\": "; writeJsonNumber(os_, r_.area_ram_dff_array);
            os_ << ", \"area_ram_clockgate\": "; writeJsonNumber(os_, r_.area_ram_clockgate);
            os_ << ", \"area_rdaddr_dff\": "; writeJsonNumber(os_, r_.area_rdaddr_dff);
            os_ << "},\n";

            os_ << "    \"leakage\": {";
            os_ << "\"leak_total\": "; writeJsonNumber(os_, r_.leak_total);
            os_ << ", \"leak_ram\": "; writeJsonNumber(os_, r_.leak_ram);
            os_ << ", \"leak_ram_decoder\": "; writeJsonNumber(os_, r_.leak_ram_decoder);
            os_ << ", \"leak_ram_mux\": "; writeJsonNumber(os_, r_.leak_ram_mux);
            os_ << ", \"leak_ram_dff_array\": "; writeJsonNumber(os_, r_.leak_ram_dff_array);
            os_ << ", \"leak_ram_clockgate\": "; writeJsonNumber(os_, r_.leak_ram_clockgate);
            os_ << ", \"leak_rdaddr_dff\": "; writeJsonNumber(os_, r_.leak_rdaddr_dff);
            os_ << "},\n";

            bool E = r_.have_energy;
            os_ << "    \"energy\": {";
            os_ << "\"e_write\": "; writeJsonNumberOrNull(os_, r_.e_write, E);
            os_ << ", \"e_write_decoder\": "; writeJsonNumberOrNull(os_, r_.e_write_decoder, E);
            os_ << ", \"e_write_mux\": "; writeJsonNumberOrNull(os_, r_.e_write_mux, E);
            os_ << ", \"e_write_dff_array\": "; writeJsonNumberOrNull(os_, r_.e_write_dff_array, E);
            os_ << ", \"e_write_clockgate\": "; writeJsonNumberOrNull(os_, r_.e_write_clockgate, E);
            os_ << ", \"e_write_rdaddr_dff\": "; writeJsonNumberOrNull(os_, r_.e_write_rdaddr_dff, E);
            os_ << ", \"e_read\": "; writeJsonNumberOrNull(os_, r_.e_read, E);
            os_ << ", \"e_read_decoder\": "; writeJsonNumberOrNull(os_, r_.e_read_decoder, E);
            os_ << ", \"e_read_mux\": "; writeJsonNumberOrNull(os_, r_.e_read_mux, E);
            os_ << ", \"e_read_dff_array\": "; writeJsonNumberOrNull(os_, r_.e_read_dff_array, E);
            os_ << ", \"e_read_clockgate\": "; writeJsonNumberOrNull(os_, r_.e_read_clockgate, E);
            os_ << ", \"e_read_rdaddr_dff\": "; writeJsonNumberOrNull(os_, r_.e_read_rdaddr_dff, E);
            os_ << ", \"e_idle\": "; writeJsonNumberOrNull(os_, r_.e_idle, E);
            os_ << "},\n";

            os_ << "    \"power\": {";
            writeEnergyPowerField(os_, "p_write", r_.e_write, r_.frequency_hz, E, false);
            writeEnergyPowerField(os_, "p_write_decoder", r_.e_write_decoder, r_.frequency_hz, E, false);
            writeEnergyPowerField(os_, "p_write_mux", r_.e_write_mux, r_.frequency_hz, E, false);
            writeEnergyPowerField(os_, "p_write_dff_array", r_.e_write_dff_array, r_.frequency_hz, E, false);
            writeEnergyPowerField(os_, "p_write_clockgate", r_.e_write_clockgate, r_.frequency_hz, E, false);
            writeEnergyPowerField(os_, "p_write_rdaddr_dff", r_.e_write_rdaddr_dff, r_.frequency_hz, E, false);
            writeEnergyPowerField(os_, "p_read", r_.e_read, r_.frequency_hz, E, false);
            writeEnergyPowerField(os_, "p_read_decoder", r_.e_read_decoder, r_.frequency_hz, E, false);
            writeEnergyPowerField(os_, "p_read_mux", r_.e_read_mux, r_.frequency_hz, E, false);
            writeEnergyPowerField(os_, "p_read_dff_array", r_.e_read_dff_array, r_.frequency_hz, E, false);
            writeEnergyPowerField(os_, "p_read_clockgate", r_.e_read_clockgate, r_.frequency_hz, E, false);
            writeEnergyPowerField(os_, "p_read_rdaddr_dff", r_.e_read_rdaddr_dff, r_.frequency_hz, E, false);
            writeEnergyPowerField(os_, "p_idle", r_.e_idle, r_.frequency_hz, E, true);
            os_ << "},\n";

            os_ << "    \"timing\": {";
            os_ << "\"t_write_data_setup\": "; writeJsonNumber(os_, r_.t_write_data_setup);
            os_ << ", \"t_write_clock_path\": "; writeJsonNumber(os_, r_.t_write_clock_path);
            os_ << ", \"t_read_access\": "; writeJsonNumber(os_, r_.t_read_access);
            os_ << ", \"t_read_addr_path\": "; writeJsonNumber(os_, r_.t_read_addr_path);
            os_ << ", \"t_critical\": "; writeJsonNumber(os_, r_.t_critical);
            os_ << ", \"slack\": "; writeJsonNumber(os_, r_.slack);
            os_ << ", \"timing_met\": "; writeJsonBool(os_, r_.timing_met);
            os_ << "}\n";

            os_ << "  }";
        }

        static void writeJsonArray(std::ostream& os_, const vector<RowResult>& rows_)
        {
            os_ << "[\n";
            for(unsigned int i = 0; i < rows_.size(); ++i)
            {
                writeRow(os_, rows_[i]);
                if((i + 1) < rows_.size())
                {
                    os_ << ",";
                }
                os_ << "\n";
            }
            os_ << "]\n";
        }

        //---------------------------------------------------------------
        // Entry point
        //---------------------------------------------------------------
        int run(int argc, char* argv[])
        {
            map<String, String> args = parseArgs(argc, argv, 2);

            String tech_path = requireArg(args, "tech");
            vector<double> frequency_list = parseDoubleList(requireArg(args, "frequency"));
            vector<unsigned int> flit_width_list = parseUIntList(requireArg(args, "flit-width"));
            unsigned int num_vns = getArgOrDefault(args, "num-vns", "1").toUInt();
            String vcs_raw = requireArg(args, "vcs-per-vn");
            String bufs_raw = requireArg(args, "bufs-per-vc");
            String buffer_model = getArgOrDefault(args, "buffer-model", "DFFRAM");
            bool timing_opt = getArgOrDefault(args, "timing-opt", "true").toBool();
            String out_path = requireArg(args, "out");
            // Phase 5: defaults match RouterInputPort's own parameter defaults.
            double write_rate = getArgOrDefault(args, "write-rate", "1.0").toDouble();
            double read_rate = getArgOrDefault(args, "read-rate", "1.0").toDouble();
            double flit_toggle_rate = getArgOrDefault(args, "flit-toggle-rate", "0.5").toDouble();
            double write_vc_switch_rate = getArgOrDefault(args, "write-vc-switch-rate", "1.0").toDouble();
            double read_vc_switch_rate = getArgOrDefault(args, "read-vc-switch-rate", "1.0").toDouble();

            if(buffer_model != "DFFRAM")
            {
                cerr << "[Error] --buffer-model only supports DFFRAM currently (got '"
                     << buffer_model << "')." << endl;
                return 1;
            }
            if(num_vns == 0)
            {
                cerr << "[Error] --num-vns must be > 0." << endl;
                return 1;
            }

            vector<vector<unsigned int> > vcs_points;
            vector<vector<unsigned int> > bufs_points;
            if(num_vns == 1)
            {
                vector<unsigned int> vcs_scalars = parseUIntList(vcs_raw);
                vector<unsigned int> bufs_scalars = parseUIntList(bufs_raw);
                for(unsigned int i = 0; i < vcs_scalars.size(); ++i)
                {
                    vcs_points.push_back(vector<unsigned int>(1, vcs_scalars[i]));
                }
                for(unsigned int i = 0; i < bufs_scalars.size(); ++i)
                {
                    bufs_points.push_back(vector<unsigned int>(1, bufs_scalars[i]));
                }
            }
            else
            {
                vector<unsigned int> vcs_fixed = parseUIntList(vcs_raw);
                vector<unsigned int> bufs_fixed = parseUIntList(bufs_raw);
                if(vcs_fixed.size() != num_vns)
                {
                    cerr << "[Error] --vcs-per-vn must have exactly " << num_vns
                         << " comma-separated values when --num-vns > 1 (got "
                         << vcs_fixed.size() << ")." << endl;
                    return 1;
                }
                if(bufs_fixed.size() != num_vns)
                {
                    cerr << "[Error] --bufs-per-vc must have exactly " << num_vns
                         << " comma-separated values when --num-vns > 1 (got "
                         << bufs_fixed.size() << ")." << endl;
                    return 1;
                }
                vcs_points.push_back(vcs_fixed);
                bufs_points.push_back(bufs_fixed);
            }

            Log::allocate("/tmp/dsent_buffer_characteristics.log");

            TechModel* tech_model = new TechModel();
            tech_model->readFile(tech_path);
            StdCellLib* std_cell_lib = new StdCellLib(tech_model);
            tech_model->setStdCellLib(std_cell_lib);

            vector<RowResult> rows;
            for(unsigned int fi = 0; fi < frequency_list.size(); ++fi)
            {
                for(unsigned int wi = 0; wi < flit_width_list.size(); ++wi)
                {
                    for(unsigned int vi = 0; vi < vcs_points.size(); ++vi)
                    {
                        for(unsigned int bi = 0; bi < bufs_points.size(); ++bi)
                        {
                            RowResult row = computeRow(tech_model, tech_path,
                                frequency_list[fi], flit_width_list[wi], num_vns,
                                vcs_points[vi], bufs_points[bi], buffer_model, timing_opt,
                                write_rate, read_rate, flit_toggle_rate,
                                write_vc_switch_rate, read_vc_switch_rate);
                            rows.push_back(row);
                        }
                    }
                }
            }

            ofstream ofs(out_path.c_str());
            if(!ofs.is_open())
            {
                cerr << "[Error] Could not open output file: " << out_path << endl;
                return 1;
            }
            writeJsonArray(ofs, rows);
            ofs.close();

            Log::release();

            cout << "Wrote " << rows.size() << " row(s) to " << out_path << endl;
            return 0;
        }
    } // namespace BufferCharacteristics
} // namespace DSENT

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

#ifndef __DSENT_MODEL_ELECTRICAL_ROUTER_ROUTER_INPUT_PORT_H__
#define __DSENT_MODEL_ELECTRICAL_ROUTER_ROUTER_INPUT_PORT_H__

#include "util/CommonType.h"
#include "model/ElectricalModel.h"

namespace DSENT
{
    // Router input port: one shared DFFRAM storage array across all VCs
    // (Becker Sec 6.3 -- a shared array doesn't add write/read delay vs.
    // per-VC arrays, since it subsumes the demux/mux those would need),
    // controlled by one FIFOControl instance per virtual channel, each
    // owning that VC's own read/write pointers and full/empty flag over
    // its slice of the shared array. See constructModel()/
    // propagateTransitionInfo() for the per-VC event composition.
    //
    // Phase 5 (traffic-driven activity): three parameters replace what used
    // to be hardcoded worst-case activity assumptions --
    //   WriteRate      -- fraction of cycles a flit is written to this port.
    //   ReadRate       -- fraction of cycles a flit departs this port.
    //   FlitToggleRate -- fraction of flit data bits that flip per write.
    // All three are intended to be supplied per epoch from an external
    // activity-counter simulator (real traffic statistics), not guessed.
    // WriteRate/ReadRate default to 1.0, reproducing the historical DSENT
    // assumption of a continuously written/read buffer. FlitToggleRate
    // defaults to 0.5 (NOT 1.0) -- that reproduces the historical uniform-
    // random TransitionInfo(0.25,0.25,0.25) flit-data assumption; 1.0 would
    // instead mean "every bit flips on every write", which is a different
    // and wrong default. Leaving all three at their defaults reproduces
    // Phase 4's numbers for area/leakage bit-for-bit and for FlitIn energy,
    // but NOT for write energy -- see the comment on the "Push" TransitionInfo
    // in constructModel() for why that one term is intentionally different
    // even at WriteRate=1.0.
    //
    // Phase 6 (VC-select activity fix): WriteVCSwitchRate/ReadVCSwitchRate
    // -- probability that two consecutive accesses of this port's kind come
    // from different VCs. VC 0 remains the structural "selected VC" (see
    // constructModel()); this only fixes the SWITCHING ACTIVITY assigned to
    // the RAM's address bus, which previously assumed VC 0 is selected
    // forever (vc_id bits permanently constant-0, understating e_read/
    // e_write's V-dependence). Default 1.0 (every access switches VC) is
    // the conservative upper bound, matching DSENT's historical flat
    // TransitionInfo(0.25,0.25,0.25) assumption for these bits; 0.0
    // reproduces the old (incorrect) always-VC-0 activity exactly. See
    // propagateTransitionInfo() for the blend formula and its rationale.
    class RouterInputPort : public ElectricalModel
    {
        public:
            RouterInputPort(const String& instance_name_, const TechModel* tech_model_);
            virtual ~RouterInputPort();

        public:
            // Set a list of properties' name needed to construct model
            void initParameters();
            // Set a list of properties' name needed to construct model
            void initProperties();

            // Clone and return a new instance
            virtual RouterInputPort* clone() const;

        protected:
            // Build the model
            virtual void constructModel();
            virtual void propagateTransitionInfo();

    }; // class RouterInputPort
} // namespace DSENT

#endif // __DSENT_MODEL_ELECTRICAL_ROUTER_ROUTER_INPUT_PORT_H__


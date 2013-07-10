/*
 * This file is part of VLE, a framework for multi-modeling, simulation
 * and analysis of complex dynamical systems.
 * http://www.vle-project.org
 *
 * Copyright (c) 2013 INRA http://www.inra.fr
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 */
#ifndef PASSIVEAGENT_HPP
#define PASSIVEAGENT_HPP

#include <vle/extension/mas/Agent.hpp>
#include <vle/devs/Time.hpp>
#include <vle/devs/Dynamics.hpp>

using namespace boost;
namespace vd = vle::devs;
namespace vv = vle::value;

namespace vle {
namespace extension {
namespace mas {

class PassiveAgent : public Agent, public vd::Dynamics
{
public:
    PassiveAgent(const vd::DynamicsInit &init, const vd::InitEventList &events)
    :vd::Dynamics(init, events),mOutputPortName("agent_output")
    ,mInputPortName("positions")
    {}

    virtual void init() = 0;
    virtual void handleEvent(Event::property_map&) = 0;
    
    /* vle::devs::Dynamics override */
    virtual vd::Time init(const vd::Time& time);
    virtual void internalTransition(const vd::Time&);
    virtual vd::Time timeAdvance() const;
    virtual void output(const vd::Time&, vd::ExternalEventList&) const;
    virtual void externalTransition(
        const vd::ExternalEventList&, const vd::Time&);
protected:
    std::string mOutputPortName;
    std::string mInputPortName;
};

}}}// namespace vle extension mas
#endif

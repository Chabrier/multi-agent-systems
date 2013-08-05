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
#include <vle/utils/Exception.hpp>
#include <vle/devs/Dynamics.hpp>

#include <vle/extension/mas/Scheduler.hpp>

namespace vd = vle::devs;

namespace mas
{
namespace test
{
namespace dynamics
{

class GenericAgent : public vd::Dynamics
{
public:
    typedef enum { INIT,IDLE } states;
    GenericAgent(const vd::DynamicsInit& init,
                 const vd::InitEventList& events)
    :vd::Dynamics(init,events),mState(INIT)
    { }

    vd::Time init(const vd::Time& time)
    {
        std::cout << "(init) ";
        switch(mState) {
            case INIT:
                std::cout << "state=INIT" << std::endl;
                return 0.0;
            break;
            case IDLE:
                std::cout << "state=IDLE" << std::endl;
                throw vle::utils::InternalError("function init called in state"\
                                                " IDLE : forbidden state");
            break;
        }

    }

    void internalTransition(const vd::Time&)
    {
        std::cout << "(internalTransition) ";
        switch(mState) {
            case INIT:
                std::cout << "state=INIT";
                mState = IDLE;
            break;
            case IDLE:
                std::cout << "state=IDLE";
            break;
        }
        std::cout << std::endl;
    }

    vd::Time timeAdvance() const
    {
        std::cout << "(timeAdvance) ";
        switch(mState) {
            case INIT:
                throw vle::utils::InternalError("function timeAdvance called "\
                                                "in state IDLE : forbidden "\
                                                "state");
            break;
            case IDLE:
                std::cout << "state=IDLE" << std::endl;
                return vd::infinity;
            break;
        }
    }

    void output(const vd::Time&, vd::ExternalEventList&) const
    {
        std::cout << "(output)";
        switch(mState) {
            case INIT:  //Send initialization informations
                std::cout << "state=INIT";
                std::cout << " - Send initial infos";
            break;
            case IDLE:
                std::cout << "state=IDLE";
            break;
        }
        std::cout << std::endl;
    }

    void externalTransition(const vd::ExternalEventList&, const vd::Time&)
    {
        std::cout << "(externalTransition) ";
        switch(mState) {
            case INIT:
                std::cout << "state=INIT";
            break;
            case IDLE:
                std::cout << "state=IDLE";
            break;
        }
        std::cout << std::endl;
    }
private:
    states mState;
    Scheduler<Event> mScheduler;
};

}}} //namespace mas test dynamics
DECLARE_DYNAMICS(mas::test::dynamics::GenericAgent)

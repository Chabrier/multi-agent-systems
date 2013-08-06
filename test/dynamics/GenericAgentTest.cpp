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
#include <iostream>

#include <vle/utils/Exception.hpp>
#include <vle/devs/Dynamics.hpp>

#include <vle/extension/mas/Scheduler.hpp>
#include <vle/extension/mas/Events.hpp>

namespace vd = vle::devs;
namespace vu = vle::utils;
using namespace vle::extension::mas;

namespace mas
{
namespace test
{
namespace dynamics
{

/** @class GenericAgent
 *  @brief Generic Agent class
 *  It allows user to create an agent model with 3 functions (agent_init,
 *  agent_dynamic, and agent_handleEvent)
 *  @see void agent_dynamic()
 *  @see void agent_init()
 *  @see void agent_handleEvent(const Event&)
 */
class GenericAgent : public vd::Dynamics
{
public:
    GenericAgent(const vd::DynamicsInit &init,
                 const vd::InitEventList &events)
    :vd::Dynamics(init,events),mState(INIT),mCurrentTime(0.0)
    { }

    vd::Time init(const vd::Time &t)
    {
        std::cout << "(init) ";
        mCurrentTime = t;
        switch(mState) {
            case INIT:
                std::cout << "state=INIT" << std::endl;
                /* Call internal transition */
                return 0.0;
            break;
            case IDLE:
                throw vu::InternalError("function init called in state"\
                                        " IDLE : forbidden state");
            break;
            case OUTPUT:
                throw vu::InternalError("function init called in state"\
                                        " OUTPUT : forbidden state");
        }
        /* Avoid compiler warning*/
        return vd::infinity;
    }

    void internalTransition(const vd::Time &t)
    {
        std::cout << "(internalTransition) ";
        mCurrentTime = t;
        switch(mState) {
            case INIT:
                std::cout << "state=INIT";
                /* model initialization */
                agent_init();
                mState = IDLE;
            break;
            case IDLE:
                std::cout << "state=IDLE";
                /* model behaviour */
                agent_dynamic();
            break;
            case OUTPUT:
                std::cout << "state=OUTPUT";
                /* remove messages (they have been sent!)*/
                mState = IDLE;
                mEventsToSend.clear();
            break;
        }

        /* Send all the messages */
        if(mEventsToSend.size() > 0)
            mState = OUTPUT;
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
                std::cout << "state=IDLE";
                if (mScheduler.empty()) {
                    std::cout << "return infinity" << std::endl;
                    /* Waiting state */
                    return vd::infinity;
                } else {
                    std::cout << "return "
                              << mScheduler.next_event()["time"]
                                            ->toDouble().value()
                              << std::endl;
                    /* Wake me when next event is ready*/
                    return mScheduler.next_event()["time"]->toDouble().value();
                }
            break;
            case OUTPUT:
                std::cout << "state=OUTPUT return 0.0"<< std::endl;
                /* Call vle::devs::output */
                return 0.0;
            break;
        }

        /* Avoid compiler warning */
        return vd::infinity;
    }

    void output(const vd::Time&, vd::ExternalEventList &event_list) const
    {
        std::cout << "(output)";
        switch(mState) {
            case INIT:
                std::cout << "state=INIT";
            break;
            case IDLE:
                std::cout << "state=IDLE";
            break;
            case OUTPUT:
                std::cout << "state=OUTPUT";
                /* Send ALL the messages */
                sendMessages(event_list);
            break;
        }
        std::cout << std::endl;
    }

    void externalTransition(const vd::ExternalEventList &event_list,
                            const vd::Time &t)
    {
        std::cout << "(externalTransition) ";
        mCurrentTime = t;
        switch(mState) {
            case INIT:
            case IDLE:
            case OUTPUT:
                /* Handle external event in any case*/
                handleExternalEvents(event_list);
            break;
        }

        /* Send all the messages */
        if(mEventsToSend.size() > 0)
            mState = OUTPUT;

        std::cout << std::endl;
    }
protected:
    /** @todo Pure virtual agent functions. Modeler must override them */
    void agent_dynamic(){}
    void agent_init(){}
    void agent_handleEvent(const Event&){}
private:
    /** @brief send all the messages in send buffer */
    void sendMessages(vd::ExternalEventList& event_list) const
    {
        for (const auto& eventToSend : mEventsToSend) {
            vd::ExternalEvent* event = new vd::ExternalEvent(cOutputPortName);

            for (const auto& p_name : eventToSend.properties()) {
                vv::Value *v = eventToSend[p_name.first].get()->clone();
                event << vd::attribute(p_name.first, v);
            }
            event << vd::attribute("from",getModelName());
            event_list.push_back(event);
        }
    }

    /** @brief  Copy external events and calls user function
     *  @see    agent_handleEvent*/
    void handleExternalEvents(const vd::ExternalEventList &event_list)
    {
        for (const auto& event : event_list) {
            if (event->getPortName() == cInputPortName) {
                Event incomingEvent;
                for (const auto& attribute : event->getAttributes()) {
                    incomingEvent.add_property(attribute.first,
                                   Event::value_ptr(attribute.second->clone()));
                }
                agent_handleEvent(incomingEvent);
            }
        }
    }
protected:
    static const std::string cOutputPortName;   /**< Agent output port name */
    static const std::string cInputPortName;    /**< Agent input port name */
private:
    typedef enum { INIT,IDLE,OUTPUT } states;

    states mState;                      /**< Agent current state */
    double mCurrentTime;                /**< Last known simulation time */
    Scheduler<Event> mScheduler;        /**< Agent scheduler */
    std::vector<Event> mEventsToSend;   /**< Events to send whith devs::output*/
};

const std::string GenericAgent::cOutputPortName = "agent_output";
const std::string GenericAgent::cInputPortName = "agent_input";

}}} //namespace mas test dynamics
DECLARE_DYNAMICS(mas::test::dynamics::GenericAgent)

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
#ifndef GENERIC_AGENT_HPP
#define GENERIC_AGENT_HPP

#include <iostream>
#include <unordered_map>

#include <vle/utils/Exception.hpp>
#include <vle/devs/Dynamics.hpp>

#include <vle/extension/mas/Scheduler.hpp>
#include <vle/extension/mas/Message.hpp>
#include <vle/extension/mas/Effect.hpp>

#include <boost/bind.hpp>
namespace vd = vle::devs;
namespace vu = vle::utils;
using namespace vle::extension::mas;

namespace vle
{
namespace extension
{
namespace mas
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
    GenericAgent(const vd::DynamicsInit &init, const vd::InitEventList &events);

    /* vle::devs override functions */
    virtual vd::Time init(const vd::Time&);
    virtual void internalTransition(const vd::Time&);
    virtual vd::Time timeAdvance() const;
    virtual void output(const vd::Time&, vd::ExternalEventList&) const;
    virtual void externalTransition(const vd::ExternalEventList&,
                                    const vd::Time&);

    inline void addEffect(const std::string& name,
                          const Effect::EffectFunction& f)
    {mEffectBinder.insert(std::make_pair(name,f));}

    inline void applyEffect(const std::string& name, const Effect& e)
    {mEffectBinder.at(name)(e);}
protected:
    /** @brief Pure virtual agent functions. Modeler must override them */
    virtual void agent_dynamic() = 0;
    /** @brief Pure virtual agent functions. Modeler must override them */
    virtual void agent_init() = 0;
    /** @brief Pure virtual agent functions. Modeler must override them */
    virtual void agent_handleEvent(const Message&) = 0;

    /* Utils functions */
    inline void sendMessage(Message& m) { mMessagesToSend.push_back(m); }
private:
    /** @brief send all the messages in send buffer */
    void sendMessages(vd::ExternalEventList& event_list) const;

    /** @brief  Copy external events and calls user function
     *  @see    agent_handleEvent*/
    void handleExternalEvents(const vd::ExternalEventList &event_list);
protected:
    static const std::string cOutputPortName;   /**< Agent output port name */
    static const std::string cInputPortName;    /**< Agent input port name */

    Scheduler<Effect> mScheduler;    /**< Agent scheduler */
    double           mCurrentTime;  /**< Last known simulation time */
    double           mLastUpdate;   /**< Last time the model had been updated */
private:
    typedef enum {INIT,   /**< initialization state:initialize vars and behaviour*/
                  IDLE,   /**< idle state : listen network and do dynamic*/
                  OUTPUT  /**< output state : send messages*/
    } states;             /**< states of machine state*/

    states             mState;          /**< Agent current state */
    std::vector<Message> mMessagesToSend;   /**< Events to send whith devs::output*/
    std::unordered_map<std::string,Effect::EffectFunction> mEffectBinder;
};

}}} //namespace vle extension mas
#endif

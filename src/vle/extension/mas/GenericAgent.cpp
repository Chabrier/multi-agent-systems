#include <vle/extension/mas/GenericAgent.hpp>

namespace vle {
namespace extension {
namespace mas {


const std::string GenericAgent::cOutputPortName = "agent_output";
const std::string GenericAgent::cInputPortName =  "agent_input";

GenericAgent::GenericAgent(const vd::DynamicsInit &init,
                           const vd::InitEventList &events)
:vd::Dynamics(init,events),mState(INIT),mCurrentTime(0.0)
{ }

vd::Time GenericAgent::init(const vd::Time &t)
{
    mCurrentTime = t;
    switch(mState) {
        case INIT:
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

void GenericAgent::internalTransition(const vd::Time &t)
{
    mCurrentTime = t;
    switch(mState) {
        case INIT:
            /* model initialization */
            agent_init();
            mState = IDLE;
            mLastUpdate = t;
        break;
        case IDLE:
            /* model behaviour */
            agent_dynamic();
            mLastUpdate = t;
        break;
        case OUTPUT:
            /* remove messages (they have been sent!)*/
            mState = IDLE;
            mMessagesToSend.clear();
        break;
    }

    /* Send all the messages */
    if(mMessagesToSend.size() > 0)
        mState = OUTPUT;
}

vd::Time GenericAgent::timeAdvance() const
{
    switch(mState) {
        case INIT:
            throw vle::utils::InternalError("function timeAdvance called "\
                                            "in state IDLE : forbidden "\
                                            "state");
        break;
        case IDLE:
            if (mScheduler.empty()) {
                /* Waiting state */
                return vd::infinity;
            } else {
                /* Wake me when next event is ready*/
                double ta = mScheduler.nextEffect().getDate() - mCurrentTime;
                if (ta < 0) {
                    return 0;
                } else {
                    return ta;
                }
            }
        break;
        case OUTPUT:
            /* Call vle::devs::output */
            return 0.0;
        break;
    }

    /* Avoid compiler warning */
    return vd::infinity;
}

void GenericAgent::output(const vd::Time &t,
                          vd::ExternalEventList &event_list) const
{
    switch(mState) {
        case INIT:
        break;
        case IDLE:
        break;
        case OUTPUT:
            /* Send ALL the messages */
            sendMessages(event_list);
        break;
    }
}

void GenericAgent::externalTransition(const vd::ExternalEventList &event_list,
                                      const vd::Time &t)
{
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
    if(mMessagesToSend.size() > 0)
        mState = OUTPUT;
}


void GenericAgent::sendMessages(vd::ExternalEventList& event_list) const
{
    for (const auto& messageToSend : mMessagesToSend) {
        vd::ExternalEvent* DEVS_event = new vd::ExternalEvent(cOutputPortName);
        for (const auto& p_name : messageToSend.getInformations()) {
            vv::Value *v = p_name.second.get()->clone();
            DEVS_event << vd::attribute(p_name.first, v);
        }
        DEVS_event << vd::attribute("sender",messageToSend.getSender());
        DEVS_event << vd::attribute("receiver",messageToSend.getReceiver());
        DEVS_event << vd::attribute("subject",messageToSend.getSubject());
        event_list.push_back(DEVS_event);
    }
}


void GenericAgent::handleExternalEvents(
                                    const vd::ExternalEventList &event_list)
{
    for (const auto& event : event_list) {
        if (event->getPortName() == cInputPortName) {
            std::string receiver = event->getAttributeValue("receiver")
                                        .toString().value();
            std::string sender = event->getAttributeValue("sender")
                                      .toString().value();
            std::string subject = event->getAttributeValue("subject")
                                       .toString().value();

            if (receiver == Message::BROADCAST || receiver == getModelName()) {
                Message incomingM(sender,receiver,subject);

                for (const auto& attribute : event->getAttributes()) {
                    incomingM.add(attribute.first,attribute.second->clone());
                }
                agent_handleEvent(incomingM);
            }
        }
    }
}


}}} //namespace vle extension mas

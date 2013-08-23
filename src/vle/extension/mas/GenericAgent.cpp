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
    std::cout << "[" << getModelName() << "][" << t << "]"
              << "(init) ";
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

void GenericAgent::internalTransition(const vd::Time &t)
{
    std::cout << "[" << getModelName() << "][" << t << "]"
              << "(internalTransition) ";
    mCurrentTime = t;
    switch(mState) {
        case INIT:
            std::cout << "state=INIT";
            /* model initialization */
            agent_init();
            mState = IDLE;
            mLastUpdate = t;
        break;
        case IDLE:
            std::cout << "state=IDLE";
            /* model behaviour */
            agent_dynamic();
            mLastUpdate = t;
        break;
        case OUTPUT:
            std::cout << "state=OUTPUT";
            /* remove messages (they have been sent!)*/
            mState = IDLE;
            mMessagesToSend.clear();
        break;
    }

    /* Send all the messages */
    if(mMessagesToSend.size() > 0)
        mState = OUTPUT;
    std::cout << std::endl;
}

vd::Time GenericAgent::timeAdvance() const
{
    std::cout << "[" << getModelName() << "][" << mCurrentTime << "]"
              << "(timeAdvance) ";
    switch(mState) {
        case INIT:
            throw vle::utils::InternalError("function timeAdvance called "\
                                            "in state IDLE : forbidden "\
                                            "state");
        break;
        case IDLE:
            std::cout << "state=IDLE";
            if (mScheduler.empty()) {
                std::cout << " return infinity" << std::endl;
                /* Waiting state */
                return vd::infinity;
            } else {
                std::cout << " return "
                          << mScheduler.nextEffect().getDate() - mCurrentTime
                          << std::endl;
                /* Wake me when next event is ready*/
                return mScheduler.nextEffect().getDate() - mCurrentTime;
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

void GenericAgent::output(const vd::Time &t,
                          vd::ExternalEventList &event_list) const
{
    std::cout << "[" << getModelName() << "][" << t << "]"
              << "(output)";
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

void GenericAgent::externalTransition(const vd::ExternalEventList &event_list,
                                      const vd::Time &t)
{
    std::cout << "[" << getModelName() << "][" << t << "]"
              << "(externalTransition) ";
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

    std::cout << std::endl;
}


void GenericAgent::sendMessages(vd::ExternalEventList& event_list) const
{
    for (const auto& messageToSend : mMessagesToSend) {
        vd::ExternalEvent* DEVS_event = new vd::ExternalEvent(cOutputPortName);
        std::cout << "< ";
        for (const auto& p_name : messageToSend.getInformations()) {
            vv::Value *v = p_name.second.get()->clone();
            DEVS_event << vd::attribute(p_name.first, v);
            std::cout << "<\"" << p_name.first << "\",\"" << *v << "\">";
        }
        DEVS_event << vd::attribute("sender",messageToSend.getSender());
        DEVS_event << vd::attribute("receiver",messageToSend.getReceiver());
        DEVS_event << vd::attribute("subject",messageToSend.getSubject());
        event_list.push_back(DEVS_event);
        std::cout << " >";
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
                Message incomingMessage(sender,receiver,subject);

                for (const auto& attribute : event->getAttributes()) {
                    incomingMessage.getInformations()[attribute.first] =
                                  Message::value_ptr(attribute.second->clone());
                }
                agent_handleEvent(incomingMessage);
            }
        }
    }
}


}}} //namespace vle extension mas

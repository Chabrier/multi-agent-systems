#include <vle/extension/mas/GenericAgent.hpp>

namespace vle {
namespace extension {
namespace mas {


const std::string GenericAgent::cOutputPortName = "agent_output";
const std::string GenericAgent::cInputPortName = "agent_input";

GenericAgent::GenericAgent(const vd::DynamicsInit &init,
                           const vd::InitEventList &events)
:vd::Dynamics(init,events),mState(INIT),mCurrentTime(0.0)
{ }

vd::Time GenericAgent::init(const vd::Time &t)
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

void GenericAgent::internalTransition(const vd::Time &t)
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

vd::Time GenericAgent::timeAdvance() const
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

void GenericAgent::output(const vd::Time&,
                          vd::ExternalEventList &event_list) const
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

void GenericAgent::externalTransition(
                                    const vd::ExternalEventList &event_list,
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


void GenericAgent::sendMessages(vd::ExternalEventList& event_list) const
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


void GenericAgent::handleExternalEvents(
                                    const vd::ExternalEventList &event_list)
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


}}} //namespace vle extension mas

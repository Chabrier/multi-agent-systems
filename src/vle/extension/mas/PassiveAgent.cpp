#include <vle/extension/mas/PassiveAgent.hpp>

namespace vle {
namespace extension {
namespace mas {

vd::Time PassiveAgent::init(const vd::Time& time)
{
    mLastUpdate = time;
    return vd::infinity;
}

void PassiveAgent::internalTransition(const vd::Time& /*time*/)
{
    mEventsToSend.clear();
}

vd::Time PassiveAgent::timeAdvance() const
{
    if (mEventsToSend.size() > 0)
        return 0.0;

    return vd::infinity;
}

void PassiveAgent::output(const vd::Time&, vd::ExternalEventList& output) const
{
    for(std::vector<Event>::const_iterator it = mEventsToSend.begin();
    it != mEventsToSend.end(); ++it) {
        vd::ExternalEvent* event = new vd::ExternalEvent(mOutputPortName);
        event << vd::attribute("time", it->time());

        for(Event::property_map::const_iterator itE = it->properties_cbegin();
        itE != it->properties_cend(); ++itE){
            vv::Value *v = it->property(itE->first).get()->clone();
            event << vd::attribute(itE->first, v);
        }
        output.push_back(event);
    }
}

void PassiveAgent::externalTransition(const vd::ExternalEventList& event,
                        const vd::Time&)
{
    for(vd::ExternalEventList::const_iterator itE = event.begin();
        itE != event.end(); ++itE){
        if((*itE)->getPortName() == mInputPortName){
            Event::property_map parameters;
            for(vv::MapValue::const_iterator it = (*itE)->getAttributes().begin();
              it != (*itE)->getAttributes().end(); ++it){
                parameters.insert(
                    std::make_pair(
                        it->first,
                        std::shared_ptr<vv::Value>(it->second->clone())));
            }
            handleEvent(parameters);
        }
    }
}

}}} // namespace vle extension mas

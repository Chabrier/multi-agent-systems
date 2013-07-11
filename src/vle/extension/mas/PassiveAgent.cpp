#include <vle/extension/mas/PassiveAgent.hpp>

namespace vle {
namespace extension {
namespace mas {

vd::Time PassiveAgent::init(const vd::Time& time)
{
    mLastUpdate = time;
    return vd::infinity;
}

void PassiveAgent::internalTransition(const vd::Time&)
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
    for(auto itE(mEventsToSend.cbegin());itE != mEventsToSend.end(); ++itE) {
        vd::ExternalEvent* event = new vd::ExternalEvent(mOutputPortName);

        for(auto itP(itE->properties_cbegin());itP != itE->properties_cend();
        ++itP){
            vv::Value *v = itE->property(itP->first).get()->clone();
            event << vd::attribute(itP->first, v);
        }
        output.push_back(event);
    }
}

void PassiveAgent::externalTransition(const vd::ExternalEventList& event,
                        const vd::Time&)
{
    for(auto itE(event.cbegin());itE != event.cend(); ++itE){
        if((*itE)->getPortName() == mInputPortName){
            Event::property_map parameters;
            for(auto it((*itE)->getAttributes().begin());
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

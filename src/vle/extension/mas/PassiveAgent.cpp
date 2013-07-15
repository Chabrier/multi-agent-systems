#include <vle/extension/mas/PassiveAgent.hpp>

namespace vle
{
namespace extension
{
namespace mas
{

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
    for (auto eventToSend : mEventsToSend) {
        vd::ExternalEvent* event = new vd::ExternalEvent(mOutputPortName);

        for (auto p_name : eventToSend.properties()) {
            vv::Value *v = eventToSend.property(p_name.first).get()->clone();
            event << vd::attribute(p_name.first, v);
        }
        output.push_back(event);
    }
}

void PassiveAgent::externalTransition(const vd::ExternalEventList& events,
                                      const vd::Time&)
{
    for (auto event : events) {
        if (event->getPortName() == mInputPortName) {
            Event::property_map parameters;
            for (auto attribute : event->getAttributes()) {
                parameters.insert(
                    std::make_pair(
                        attribute.first,
                        std::shared_ptr<vv::Value>(attribute.second->clone())));
            }
            handleEvent(parameters);
        }
    }
}

}
}
} // namespace vle extension mas

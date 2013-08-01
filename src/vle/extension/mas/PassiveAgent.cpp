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
    mCurrentTime = time;
    return vd::infinity;
}

void PassiveAgent::internalTransition(const vd::Time& time)
{
    mCurrentTime = time;
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
    for (const auto& eventToSend : mEventsToSend) {
        vd::ExternalEvent* event = new vd::ExternalEvent(mOutputPortName);

        for (const auto& p_name : eventToSend.properties()) {
            vv::Value *v = eventToSend.property(p_name.first).get()->clone();
            event << vd::attribute(p_name.first, v);
        }
        event << vd::attribute("from",getModelName());
        output.push_back(event);
    }
}

void PassiveAgent::externalTransition(const vd::ExternalEventList& events,
                                      const vd::Time& time)
{
    mCurrentTime = time;
    for (const auto& event : events) {
        if (event->getPortName() == mInputPortName) {
            Event parameters;
            for (auto attribute : event->getAttributes()) {
                parameters.add_property(
                                   attribute.first,
                                   Event::value_ptr(attribute.second->clone()));
            }
            handleEvent(parameters);
        }
    }
}

}
}
} // namespace vle extension mas

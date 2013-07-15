#include <vle/extension/mas/Events.hpp>

namespace vle
{
namespace extension
{
namespace mas
{

bool operator> (const Event& a, const Event& b)
{
    return a.mProperties.at("time")->toDouble().value() >
           b.mProperties.at("time")->toDouble().value();
}

std::shared_ptr<vv::Value> Event::operator[](const std::string& p)const
{
    return property(p);
}

}
}
}

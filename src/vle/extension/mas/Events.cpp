#include <vle/extension/mas/Events.hpp>

namespace vle
{
namespace extension
{
namespace mas
{

bool operator> (const Event& a, const Event& b)
{
    return a.property("time")->toDouble().value() >
           b.property("time")->toDouble().value();
}

Event::value_ptr Event::property(const std::string& p)const
{
    try {
        return mProperties.at(p);
    } catch (std::exception e){
        std::string txt = e.what();
        txt = "Event class - at(\""+ p +"\") failed ("
              + txt + ").";
        throw std::logic_error(txt);
    }
}

std::shared_ptr<vv::Value> Event::operator[](const std::string& p)const
{
    return property(p);
}

}
}
}

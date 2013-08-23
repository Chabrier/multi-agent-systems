#include <vle/extension/mas/Effect.hpp>

namespace vle {
namespace extension {
namespace mas {

Effect::Effect(const vd::Time& t,
               const std::string& name,
               const std::string& origin)
:mDate(t),mName(name),mOrigin(origin)
{}


bool operator> (const Effect &a,const Effect &b)
{
    return a.mDate > b.mDate;
}

}}}// namespace vle extension mas

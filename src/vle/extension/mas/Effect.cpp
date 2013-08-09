#include <vle/extension/mas/Effect.hpp>

namespace vle {
namespace extension {
namespace mas {

vd::Time Effect::getDate() const
{
    return mDate;
}

const std::string& Effect::getName() const
{
    return mName;
}

bool operator> (const Effect &a,const Effect &b)
{
    return a.mDate > b.mDate;
}

property_map& Effect::getInformations()
{
    return mInformations;
}
const property_map& Effect::getInformations() const
{
    return mInformations;
}

}}}// namespace vle extension mas

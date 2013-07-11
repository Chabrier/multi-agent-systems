#include <vle/extension/mas/Events.hpp>

namespace vle {
namespace extension {
namespace mas {

bool operator> (const Event& a, const Event& b)
{ return a.mProperties.at("time") > b.mProperties.at("time"); }

}}}

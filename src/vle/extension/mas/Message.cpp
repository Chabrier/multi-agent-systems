#include <vle/extension/mas/Message.hpp>

namespace vle {
namespace extension {
namespace mas {

const std::string Message::BROADCAST = "BROADCAST";

Message::Message(const std::string& sender,
                 const std::string& receiver,
                 const std::string& subject)
:mSender(sender),mReceiver(receiver),mSubject(subject)
{}

}}}//namespace vle extension mas

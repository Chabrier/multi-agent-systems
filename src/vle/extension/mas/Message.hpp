/*
 * This file is part of VLE, a framework for multi-modeling, simulation
 * and analysis of complex dynamical systems.
 * http://www.vle-project.org
 *
 * Copyright (c) 2013 INRA http://www.inra.fr
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 */
#ifndef MESSAGE_HPP
#define MESSAGE_HPP
#include <vle/value/Value.hpp>
#include <unordered_map>
#include <vle/extension/mas/PropertyContainer.hpp>

namespace vle {
namespace extension {
namespace mas {

namespace vv = vle::value;

/** @class Message
 *  @brief Allows to handle messages received from the network, or to send a
 *         message
 *
 * This class uses generic type vle::value:Value to store informations. You can
 * easily send a message by specifing a sender in constructor.
 * The Message::BROADCAST value allows to end a message to all agents.
 *
 * @see vle::value:Value
 */
class Message : public PropertyContainer
{
/* Public functions */
public:
    Message(const std::string&,const std::string&,const std::string&);

    inline std::string getSender() const
    {return mSender;}

    inline std::string getReceiver() const
    {return mReceiver;}

    inline std::string getSubject() const
    {return mSubject;}

/* Private functions */
private:
    Message();

/* Public constants */
public:
    static const std::string BROADCAST;

/* Private members */
private:
    std::string mSender;
    std::string mReceiver;
    std::string mSubject;
};

}}} //namespace vle extension mas
#endif

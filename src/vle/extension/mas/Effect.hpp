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
#ifndef EFFECT_HPP
#define EFFECT_HPP
#include <boost/function.hpp>
#include <vle/devs/Time.hpp>
#include <vle/value/Value.hpp>
#include <unordered_map>
#include <vle/extension/mas/PropertyContainer.hpp>

namespace vd = vle::devs;
namespace vv = vle::value;

namespace vle {
namespace extension {
namespace mas {

/** @class Effect
 *  @brief Describes an effect on agents
 *
 *  This class is used to apply effect on agents.
 */
class Effect : public PropertyContainer
{
public:
    /**  Function prototype which will be use to apply effect */
    typedef boost::function<void (const Effect&)> EffectFunction;

public:
    Effect(const vd::Time& t,const std::string& name,const std::string& origin)
    :mDate(t),mName(name),mOrigin(origin)
    {}

    inline vd::Time getDate() const
    {return mDate;}

    inline const std::string& getName() const
    {return mName;}

    /* Operator overload */
    friend bool operator==(const Effect& a,const Effect& b)
    {
        return (a.mDate == b.mDate)
               &&(a.mName == b.mName)
               &&(a.mOrigin == b.mOrigin);
    }
    friend bool operator< (const Effect& a,const Effect& b)
    {return a.mDate < b.mDate;}
    friend bool operator!=(const Effect& a, const Effect& b)
    {return !operator==(a,b);}
    friend bool operator> (const Effect& a, const Effect& b)
    {return  operator< (b,a);}
    friend bool operator<=(const Effect& a, const Effect& b)
    {return !operator> (a,b);}
    friend bool operator>=(const Effect& a, const Effect& b)
    {return !operator< (a,b);}
private:
    Effect();
private:
    vd::Time     mDate; /**< Date when effect must be applied */
    std::string  mName; /**< Name of effect */
    std::string  mOrigin; /**< Origin(model name) of effect */
};

}}} //namespace vle extension mas
#endif

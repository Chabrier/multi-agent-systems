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
#include <vle/devs/Time.hpp>
#include <vle/value/Value.hpp>
#include <map>
namespace vd = vle::devs;
namespace vv = vle::value;

namespace vle {
namespace extension {
namespace mas {

class Effect
{
public:
    typedef std::shared_ptr<vv::Value> value_ptr;
    typedef std::map<std::string, value_ptr> property_map;
    Effect(const vd::Time&,const std::string&,const std::string&);

    vd::Time getDate() const;
    const std::string& getName() const;
    property_map&  getInformations();
    const property_map&  getInformations() const;

    friend bool operator> (const Effect&,const Effect&);
protected:
private:
    Effect();
private:
    vd::Time     mDate;
    std::string  mName;
    std::string  mOrigin;
    property_map mInformations;
};

}}} //namespace vle extension mas
#endif

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
#ifndef EVENTS_HPP
#define EVENTS_HPP

#include <vle/devs/Time.hpp>
#include <vle/value/Value.hpp>
#include <vle/value/Double.hpp>

#include <map>
#include <boost/numeric/ublas/vector.hpp>

using namespace boost::numeric;

namespace vv = vle::value;

namespace vle
{
namespace extension
{
namespace mas
{

class Event
{
public:
    typedef std::map<std::string, std::shared_ptr<vv::Value>> property_map;
    Event(): Event(0) {}
    Event(double t)
    {
        add_property("time", new vv::Double(t));
    }

    std::shared_ptr<vv::Value> property(const std::string& title)const
    {
        return mProperties.at(title);
    }

    bool exist_property(const std::string &title)const
    {
        return (mProperties.find(title) != mProperties.end());
    }

    void add_property(const std::string &t, vv::Value * && v)
    {
        add_property(t, std::shared_ptr<vv::Value>(v));
    }

    void add_property(const std::string &t, const std::shared_ptr<vv::Value> &v)
    {
        if (exist_property(t))
            mProperties.erase(t);
        mProperties.insert(std::make_pair(t, v));
    }

    property_map::const_iterator properties_cbegin() const
    {
        return mProperties.cbegin();
    }

    property_map::const_iterator properties_cend() const
    {
        return mProperties.cend();
    }

    friend bool operator> (const Event& a, const Event& b);
    std::shared_ptr<vv::Value> operator[](const std::string&)const;
protected:
private:
    std::map<std::string, std::shared_ptr<vv::Value> > mProperties;
};

}
}
}// namespace vle extension mas
#endif

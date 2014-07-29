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
#ifndef IBM_TEST_GODLUA_HPP
#define IBM_TEST_GODLUA_HPP

#include "GodProxy.hpp"

#include <boost/lexical_cast.hpp>

#include <vle/value/Value.hpp>
#include <vle/devs/Executive.hpp>
#include <vle/value/Map.hpp>
#include <vle/utils/Rand.hpp>

#include<math.h>

#include <vle/extension/mas/collision/Vector2d.hpp>

namespace vd = vle::devs;
namespace vv = vle::value;
namespace vp = vle::vpz;
namespace vu = vle::utils;

namespace mas
{
namespace test
{
namespace dynamics
{

class God : public vd::Executive
{
public:
    typedef std::vector<std::string> Strings;

    God(const vd::ExecutiveInit& init,
        const vd::InitEventList& events);


    vd::Time init(const vd::Time&);

    void createSky(double n, double s, double e, double w);

    void createBird(double x, double y, double dx, double dy, double radius);

    bool inANeigborhood(double x, double y);

    void connect(const std::string& new_agent);


private:
    int mBirdNumbers;
    Strings mModels;
    std::string mView;

    typedef std::map< std::string, std::array<double, 3>> KeepCordinates;

    KeepCordinates mKeepCoordinates;

    double mNorth;
    double mSouth;
    double mEast;
    double mWest;

    vu::Rand mRand;

    double mRadiusMin;
    double mRadiusMax;
    double mSpeedMin;
    double mSpeedMax;
    double mPopulation;

    std::string mLuaScript;

    lua_State *L;
    GodProxy mGP;
};


}
}
} // namespace vle example
#endif

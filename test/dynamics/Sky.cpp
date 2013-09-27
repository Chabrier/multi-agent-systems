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
#include <cmath>

#include <vle/value/Value.hpp>
#include <vle/devs/Dynamics.hpp>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/arithmetic/dot_product.hpp>
#include <boost/numeric/ublas/vector.hpp>

#include <vle/extension/mas/GenericAgent.hpp>
#include <vle/extension/mas/collision/Types.hpp>
#include <vle/extension/mas/collision/Segment.hpp>
#include <vle/extension/mas/collision/Circle.hpp>

using namespace vle::extension::mas;

namespace mas
{
namespace test
{
namespace dynamics
{

namespace vd = vle::devs;
namespace vv = vle::value;
namespace bg = boost::geometry;
namespace bn = boost::numeric;

class Sky : public GenericAgent
{
public:
    Sky(const vd::DynamicsInit& init, const vd::InitEventList& events)
        : GenericAgent(init, events)
    {
        mNorth = events.exist("north") ? events.getDouble("north") : -1;
        mSouth = events.exist("south") ? events.getDouble("south") : -1;
        mEast = events.exist("east") ? events.getDouble("east") : -1;
        mWest = events.exist("west") ? events.getDouble("west") : -1;
    }

    void agent_init() {}

    void agent_dynamic() {}

    void agent_handleEvent(const Message& message)
    {
        if (message.getSubject() == "birdPosition") {
            sendEnterAgainEvent(message.getSender());
        }
    }

    void sendEnterAgainEvent(const std::string& ball_name)
    {
        Message m(getModelName(),ball_name,"enterAgain");

        m.add("north",vv::Double::create(mNorth));
        m.add("south",vv::Double::create(mSouth));
        m.add("east",vv::Double::create(mEast));
        m.add("west",vv::Double::create(mWest));

        sendMessage(m);
    }

private:
    double mNorth;
    double mSouth;
    double mEast;
    double mWest;
};
}
}
} // namespace vle example
DECLARE_DYNAMICS(mas::test::dynamics::Sky)



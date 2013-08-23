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

class WallG : public GenericAgent
{
public:
    WallG(const vd::DynamicsInit& init, const vd::InitEventList& events)
        : GenericAgent(init, events)
    {
        Point x1(events.exist("x1") ? events.getDouble("x1") : -1,
                 events.exist("y1") ? events.getDouble("y1") : -1);
        Point x2(events.exist("x2") ? events.getDouble("x2") : -1,
                 events.exist("y2") ? events.getDouble("y2") : -1);

        mSegment.setEnd1(x1);
        mSegment.setEnd2(x2);
    }

    void agent_init() {}

    void agent_dynamic() {}

    void agent_handleEvent(const Message& message)
    {
        if (message.getSubject() == "ball_position") {
            double dx = message.get("dx")->toDouble().value();
            double dy = message.get("dy")->toDouble().value();
            double c_x = message.get("x")->toDouble().value();
            double c_y = message.get("y")->toDouble().value();
            double radius = message.get("radius")->toDouble().value();
            Vector2d v_ball(dx,dy);
            Circle circle(Point(c_x,c_y),radius);

            if(circle.inCollision(mSegment,v_ball)) {
                sendCollisionEvent(message.getSender());
            }
        }
    }

    void sendCollisionEvent(const std::string& ball_name)
    {
        Message m(getModelName(),ball_name,"collision");

        m.add("wall_x1",vv::Double::create(mSegment.getEnd1().x()));
        m.add("wall_y1",vv::Double::create(mSegment.getEnd1().y()));
        m.add("wall_x2",vv::Double::create(mSegment.getEnd2().x()));
        m.add("wall_y2",vv::Double::create(mSegment.getEnd2().y()));

        sendMessage(m);
    }

private:
    Segment mSegment;
};
}
}
} // namespace vle example
DECLARE_DYNAMICS(mas::test::dynamics::WallG)



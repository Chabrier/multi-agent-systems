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
            Vector2d v_ball(message.getInformations().at("dx")->toDouble().value(),
                            message.getInformations().at("dy")->toDouble().value());
            Circle circle(Point(message.getInformations().at("x")->toDouble().value(),
                                message.getInformations().at("y")->toDouble().value()),
                                message.getInformations().at("radius")->toDouble().value());

            if(circle.inCollision(mSegment,v_ball)) {
                CollisionPoints cp = circle.collisionPoints(mSegment,v_ball);
                Point collision_pt = cp.object1CollisionPosition;
                double distance, time;
                std::string ball_name;

                distance = bg::distance(circle.getCenter(), collision_pt);
                time = distance / v_ball.norm();
                if(distance > 0) {
                    sendCollisionEvent(collision_pt,
                                       distance,
                                       time+mCurrentTime,
                                       message.getSender());
                }
            }
        }
    }

    void sendCollisionEvent(Point xy_collision,
                            double collision_distance,
                            double collision_time,
                            const std::string& ball_name)
    {
        Message m(getModelName(),ball_name,"collision");
        Message::property_map& pm = m.getInformations();
        pm.insert(std::make_pair("new_x",
                       Message::value_ptr(vv::Double::create(xy_collision.x()))));
        pm.insert(std::make_pair("new_y",
                       Message::value_ptr(vv::Double::create(xy_collision.y()))));
        pm.insert(std::make_pair("wall_x1",
                       Message::value_ptr(vv::Double::create(mSegment.getEnd1().x()))));
        pm.insert(std::make_pair("wall_y1",
                       Message::value_ptr(vv::Double::create(mSegment.getEnd1().y()))));
        pm.insert(std::make_pair("wall_x2",
                       Message::value_ptr(vv::Double::create(mSegment.getEnd2().x()))));
        pm.insert(std::make_pair("wall_y2",
                       Message::value_ptr(vv::Double::create(mSegment.getEnd2().y()))));
        pm.insert(std::make_pair("with",
                       Message::value_ptr(vv::String::create("wall"))));

        sendMessage(m);
    }

private:
    Segment mSegment;
};
}
}
} // namespace vle example
DECLARE_DYNAMICS(mas::test::dynamics::WallG)



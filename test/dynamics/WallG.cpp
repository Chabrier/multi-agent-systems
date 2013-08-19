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
#include <vle/extension/mas/Events.hpp>
#include <vle/extension/mas/collision/Types.hpp>
#include <vle/extension/mas/collision/Segment.hpp>
#include <vle/extension/mas/collision/Circle.hpp>

using namespace vle::extension::mas;
namespace vd = vle::devs;
namespace vv = vle::value;
namespace bg = boost::geometry;
namespace bn = boost::numeric;

namespace mas
{
namespace test
{
namespace dynamics
{

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

    void agent_handleEvent(const Event& event)
    {
        if (event.property("type")->toString().value() == "ball_position") {
            Vector2d v_ball(event.property("dx")->toDouble().value(),
                            event.property("dy")->toDouble().value());
            Circle circle(Point(event.property("x")->toDouble().value(),
                                event.property("y")->toDouble().value()),
                          event.property("radius")->toDouble().value());

            if(circle.inCollision(mSegment,v_ball)) {
                CollisionPoints cp = circle.collisionPoints(mSegment,v_ball);
                Point collision_pt = cp.object1CollisionPosition;
                double distance, time;
                std::string ball_name;

                distance = bg::distance(circle.getCenter(), collision_pt);
                time = distance / v_ball.norm();
                ball_name = event.property("from")->toString().value();
                if(distance > 0) {
                    sendCollisionEvent(collision_pt,
                                       distance,
                                       time+mCurrentTime,
                                       ball_name);
                }
            }
        }
    }

    void sendCollisionEvent(Point xy_collision,
                            double collision_distance,
                            double collision_time,
                            const std::string& ball_name)
    {
        Event new_collision(collision_time);

        new_collision.add_property("new_x",
                                   vv::Double::create(xy_collision.x()));
        new_collision.add_property("new_y",
                                   vv::Double::create(xy_collision.y()));
        new_collision.add_property("wall_x1",
                                   vv::Double::create(mSegment.getEnd1().x()));
        new_collision.add_property("wall_y1",
                                   vv::Double::create(mSegment.getEnd1().y()));
        new_collision.add_property("wall_x2",
                                   vv::Double::create(mSegment.getEnd2().x()));
        new_collision.add_property("wall_y2",
                                   vv::Double::create(mSegment.getEnd2().y()));
        new_collision.add_property("with",
                                   vv::String::create("wall"));
        new_collision.add_property("collision_distance",
                                   vv::Double::create(collision_distance));
        new_collision.add_property("type",vv::String::create("collision"));
        new_collision.add_property("to",vv::String::create(ball_name));
        sendEvent(new_collision);
    }

private:
    Segment mSegment;
};
}
}
} // namespace vle example
DECLARE_DYNAMICS(mas::test::dynamics::WallG)



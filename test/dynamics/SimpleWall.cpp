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

#include <vle/extension/mas/PassiveAgent.hpp>
#include <vle/extension/mas/Events.hpp>
#include <vle/extension/mas/Utils.hpp>

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

class SimpleWall : public PassiveAgent
{
public:
    SimpleWall(const vd::DynamicsInit& init, const vd::InitEventList& events)
        : PassiveAgent(init, events)
    {
        x1.x((events.exist("x1")) ? events.getDouble("x1") : -1);
        x2.x((events.exist("x2")) ? events.getDouble("x2") : -1);
        x1.y((events.exist("y1")) ? events.getDouble("y1") : -1);
        x2.y((events.exist("y2")) ? events.getDouble("y2") : -1);
    }

    void init() {}

    void handleEvent(Event& event)
    {
        if (event.property("type")->toString().value() == "ball_position") {
            point xy_ball, collision_pt; //Ball position
            vector v_ball(2);            //Ball direction vector
            bool collision = false;
	    double radius;

            xy_ball.x(event.property("x")->toDouble().value());
            xy_ball.y(event.property("y")->toDouble().value());
            v_ball[0] = (event.property("dx")->toDouble().value());
            v_ball[1] = (event.property("dy")->toDouble().value());
	    radius = (event.property("radius")->toDouble().value());

            std::tie(collision, collision_pt) = collision_point(x1,
                                                                x2,
                                                                xy_ball,
                                                                v_ball,
								radius);
            if(collision) {
                vector direction(2);
                double distance, time;
                std::string ball_name;

                distance = bg::distance(xy_ball, collision_pt);
                time = distance / bn::ublas::norm_2(v_ball);
                ball_name = event.property("from")->toString().value();
                if(distance > 0) {
                    direction = new_direction(x1,x2,xy_ball,v_ball);
                    sendCollisionEvent(collision_pt,direction,
                                       distance,time+mCurrentTime,ball_name);
                }
            }
        }
    }

    void sendCollisionEvent(point xy_collision,
                            vector new_vector,
                            double collision_distance,
                            double collision_time,
                            const std::string& ball_name)
    {
        Event new_collision(collision_time);

        new_collision.add_property("new_dx",vv::Double::create(new_vector[0]));
        new_collision.add_property("new_dy",vv::Double::create(new_vector[1]));
        new_collision.add_property("new_x",
                                   vv::Double::create(xy_collision.x()));
        new_collision.add_property("new_y",
                                   vv::Double::create(xy_collision.y()));
        new_collision.add_property("collision_distance",
                                   vv::Double::create(collision_distance));
        new_collision.add_property("type",vv::String::create("collision"));
        new_collision.add_property("to",vv::String::create(ball_name));
        mEventsToSend.push_back(new_collision);
    }

private:
    point x1;
    point x2;
};
}
}
} // namespace vle example
DECLARE_DYNAMICS(mas::test::dynamics::SimpleWall)


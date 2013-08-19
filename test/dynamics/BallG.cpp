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
#include <vle/value/Map.hpp>
#include <vle/value/Value.hpp>
#include <vle/devs/Dynamics.hpp>
#include <vle/utils/Tools.hpp>

#include <boost/geometry/geometries/point_xy.hpp>

#include <vle/extension/mas/GenericAgent.hpp>
#include <vle/extension/mas/collision/Vector2d.hpp>
#include <vle/extension/mas/collision/Types.hpp>
#include <vle/extension/mas/collision/Circle.hpp>

namespace vd = vle::devs;
namespace vv = vle::value;
namespace bg = boost::geometry;
namespace bn = boost::numeric;

using namespace vle::extension::mas;

namespace mas
{
namespace test
{
namespace dynamics
{

class BallG : public GenericAgent
{
public:
    BallG(const vd::DynamicsInit& init, const vd::InitEventList& events)
        : GenericAgent(init, events)
    {
        mCircle.getCenter().x(events.exist("x") ? events.getDouble("x"):-1);
        mCircle.getCenter().y(events.exist("y") ? events.getDouble("y"):-1);
        mDirection.x() = events.exist("dx") ? events.getDouble("dx") : -1;
        mDirection.y() = events.exist("dy") ? events.getDouble("dy") : -1;
        mCircle.getRadius() = events.exist("radius")
                              ? events.getDouble("radius"):0;
    }
protected:
    void agent_init()
    {
        sendMyInformation();
    }

    void agent_dynamic()
    {
        Event next_event = mScheduler.next_event();
        mScheduler.remove_next_event();

        if (next_event["type"]->toString().value() == "collision") {
            bool corner = false;
            Vector2d newDirection;
            if(next_event["with"]->toString().value() == "wall") {
                Segment s(Point(next_event["wall_x1"]->toDouble().value(),
                                next_event["wall_y1"]->toDouble().value()),
                          Point(next_event["wall_x2"]->toDouble().value(),
                                next_event["wall_y2"]->toDouble().value()));
                newDirection = mCircle.newDirection(s,mDirection);

                if (!mScheduler.empty() &&
                    mScheduler.next_event()["with"]->toString().value() == "wall") {
                    Event next_event2 = mScheduler.next_event();

                    if(next_event2["type"]->toString().value() == "collision") {
                        Segment s2(Point(next_event2["wall_x1"]->toDouble().value(),
                                        next_event2["wall_y1"]->toDouble().value()),
                                  Point(next_event2["wall_x2"]->toDouble().value(),
                                        next_event2["wall_y2"]->toDouble().value()));

                        Vector2d newDirection2 = mCircle.newDirection(s2,mDirection);

                        if((newDirection.x() == newDirection2.y())
                           &&(newDirection.y() == newDirection2.y()))
                           corner = true;
                        else if(next_event["collision_distance"]->toDouble().value()
                           == next_event2["collision_distance"]->toDouble().value())
                            corner = true;
                        else if(next_event["time"]->toDouble().value()
                                == next_event2["time"]->toDouble().value())
                            corner = true;
                    }
                }

            } else if (next_event["with"]->toString().value()
                       == "ball") {
                newDirection.x() = next_event["new_dx"]->toDouble().value();
                newDirection.y() = next_event["new_dy"]->toDouble().value();
            }

            mCircle.getCenter().x(next_event["new_x"]->toDouble().value());
            mCircle.getCenter().y(next_event["new_y"]->toDouble().value());

            if (!corner) {
                mDirection = newDirection;
            } else {
                mDirection= mDirection * -1;
            }
            sendMyInformation();
        }

        while (!mScheduler.empty()) {
            mScheduler.remove_next_event();
        }
    }

    void agent_handleEvent(const Event &event)
    {
        std::string type = event.property("type")->toString().value();

        vd::Time delta_t = mCurrentTime - mLastUpdate;
        double x = (mDirection.x() * delta_t) + mCircle.getCenter().x();
        double y = (mDirection.y() * delta_t) + mCircle.getCenter().y();
        Circle currentCircle(Point(x,y),mCircle.getRadius());

        if(type == "collision") {
            std::string receiver = event.property("to")->toString().value();
            if(receiver == getModelName())
                mScheduler.add_event(event);
        } else if (type == "ball_position") {
            Point c2(event.property("x")->toDouble().value(),
                     event.property("y")->toDouble().value());
            Vector2d d2(event.property("dx")->toDouble().value(),
                        event.property("dy")->toDouble().value());

            double radius = event.property("radius")->toDouble().value();
            if(currentCircle.inCollision(mDirection, Circle(c2,radius),d2)) {
                CollisionPoints cp = currentCircle.collisionPoints(mDirection,
                                                              Circle(c2,radius),
                                                              d2);
                Vector2d new_direction = currentCircle.newDirection(mDirection,
                                                              Circle(c2,radius),
                                                              d2);
                Vector2d o_new_direction = Circle(c2,radius).newDirection(d2,
                                                                          currentCircle,
                                                                          mDirection);
                double distance = bg::distance(cp.object1CollisionPosition,
                                               currentCircle.getCenter());
                double time = distance / mDirection.norm();
                double distance2 = bg::distance(cp.object2CollisionPosition,
                                                c2);
                double time2 = distance2 / d2.norm();
                std::string to = event.property("from")->toString().value();
                addCollisionEvent(cp.object1CollisionPosition,
                                  new_direction,
                                  distance,
                                  time+mCurrentTime,
                                  to);
                // Update other ball
                sendCollisionEvent(cp.object2CollisionPosition,
                                   o_new_direction,
                                   distance2,
                                   time2+mCurrentTime,
                                   to);
            }
        }
    }

    vv::Value* observation(const vd::ObservationEvent& event) const
    {
        vd::Time delta_t = event.getTime() - mLastUpdate;
        std::cout << "MyCurrent time" << event.getTime() << "Mylast Update:" << mLastUpdate << " delta" << delta_t << std::endl;
        double x = (mDirection.x() * delta_t) + mCircle.getCenter().x();
        double y = (mDirection.y() * delta_t) + mCircle.getCenter().y();
        if (event.onPort("x")) {
            return new vv::Double(x);
        }
        if (event.onPort("y")) {
            return new vv::Double(y);
        }
        if (event.onPort("coordinates")) {
            std::string output;
            output = "(" + vle::utils::toScientificString(x)
                 + ";" + vle::utils::toScientificString(y);
            output += ";" + vle::utils::toScientificString(mCircle.getRadius())
                      +")";
                return new vv::String(output);
        }
        return 0;
    }

    void sendMyInformation()
    {
        Event event(0);

        event.add_property("x", vv::Double::create(mCircle.getCenter().x()));
        event.add_property("y", vv::Double::create(mCircle.getCenter().y()));
        event.add_property("dx", vv::Double::create(mDirection.x()));
        event.add_property("dy", vv::Double::create(mDirection.y()));
        event.add_property("from", vv::String::create(getModelName()));
        event.add_property("type", vv::String::create("ball_position"));
        event.add_property("radius", vv::Double::create(mCircle.getRadius()));

        sendEvent(event);
    }

    void sendCollisionEvent(Point xy_collision,
                           Vector2d new_vector,
                           double collision_distance,
                           double collision_time,
                           const std::string& ball_name)
    {
        Event new_collision(collision_time);

        new_collision.add_property("new_dx",
                                   new vv::Double(new_vector.x()));
        new_collision.add_property("new_dy",
                                   new vv::Double(new_vector.y()));
        new_collision.add_property("new_x",
                                   new vv::Double(xy_collision.x()));
        new_collision.add_property("new_y",
                                   new vv::Double(xy_collision.y()));
        new_collision.add_property("collision_distance",
                                   new vv::Double(collision_distance));
        new_collision.add_property("type",
                                   new vv::String("collision"));
        new_collision.add_property("with",
                                   new vv::String("ball"));
        new_collision.add_property("to",new vv::String(ball_name));
        new_collision.add_property("from",new vv::String(getModelName()));

        sendEvent(new_collision);
    }

    void addCollisionEvent(Point xy_collision,
                           Vector2d new_vector,
                           double collision_distance,
                           double collision_time,
                           const std::string& ball_name)
    {
        Event new_collision(collision_time);

        new_collision.add_property("new_dx",
                                   new vv::Double(new_vector.x()));
        new_collision.add_property("new_dy",
                                   new vv::Double(new_vector.y()));
        new_collision.add_property("new_x",
                                   new vv::Double(xy_collision.x()));
        new_collision.add_property("new_y",
                                   new vv::Double(xy_collision.y()));
        new_collision.add_property("collision_distance",
                                   new vv::Double(collision_distance));
        new_collision.add_property("type",
                                   new vv::String("collision"));
        new_collision.add_property("with",
                                   new vv::String("ball"));
        new_collision.add_property("to",new vv::String(ball_name));
        new_collision.add_property("from",new vv::String(getModelName()));
        mScheduler.add_event(new_collision);
        std::cout << "ADD BALL COLLISION: " << new_vector.x()
                  << ";" << new_vector.y()
                  << " " << xy_collision.x() << ";" << xy_collision.y()
                  << " time=" << collision_time << std::endl;
    }

private:
    Circle   mCircle;
    Vector2d mDirection;
};

}}} //namespace mas test dynamics
DECLARE_DYNAMICS(mas::test::dynamics::BallG)

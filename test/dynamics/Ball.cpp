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

#include <vle/extension/mas/ActiveAgent.hpp>
#include <vle/extension/mas/Events.hpp>
#include <vle/extension/mas/Utils.hpp>

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

class Ball : public ActiveAgent
{
public:
    Ball(const vd::DynamicsInit& init, const vd::InitEventList& events)
        : ActiveAgent(init, events),mDirection(2)
    {
        mDirectionChanged = true;
        mPosition.x((events.exist("x")) ? events.getDouble("x") : -1);
        mPosition.y((events.exist("y")) ? events.getDouble("y") : -1);
        mDirection(0)= (events.exist("dx")) ? events.getDouble("dx") : -1;
        mDirection(1)= (events.exist("dy")) ? events.getDouble("dy") : -1;
        mRadius= (events.exist("radius")) ? events.getDouble("radius") : 0;
    }

    void init() { }

    vd::Time init(const vd::Time& time)
    {
        mLastUpdate = time;
        mCurrentTime = time;
        return 0;
    }

    vd::Time timeAdvance() const
    {
        if (!mScheduler.empty()) {
            return mScheduler.next_event()["time"]->toDouble().value()
                   - mCurrentTime;
        }

        if (mDirectionChanged)
            return 0;

        return vd::infinity;
    }

    void internalTransition(const vd::Time& time)
    {
        mCurrentTime = time;
        mDirectionChanged = false;
        if (!mScheduler.empty()) {
            Event next_event = mScheduler.next_event();
            mScheduler.remove_next_event();

            if (next_event["type"]->toString().value() == "collision") {
                bool corner = false;

                if (!mScheduler.empty()) {
                    Event next_event2 = mScheduler.next_event();

                    if((next_event["new_x"]->toDouble().value()
                       == next_event2["new_x"]->toDouble().value())
                       &&(next_event["new_y"]->toDouble().value()
                       == next_event2["new_y"]->toDouble().value()))
                       corner = true;
                    else if(next_event["collision_distance"]->toDouble().value()
                       == next_event2["collision_distance"]->toDouble().value())
                        corner = true;
                    else if(next_event["time"]->toDouble().value()
                            == next_event2["time"]->toDouble().value())
                        corner = true;
                }

                mPosition.x(next_event["new_x"]->toDouble().value());
                mPosition.y(next_event["new_y"]->toDouble().value());
                if (!corner) {
                    mDirection(0)= next_event["new_dx"]->toDouble().value();
                    mDirection(1)= next_event["new_dy"]->toDouble().value();
                } else {
                    mDirection= mDirection * -1;
                }
                mDirectionChanged = true;
            }

            while (!mScheduler.empty()) {
                mScheduler.remove_next_event();
            }

            mLastUpdate = time;
        }
    }

    void externalTransition(const vd::ExternalEventList& events,
                            const vd::Time& time)
    {
        mCurrentTime = time;
        for (const auto& event : events) {
            if (event->getPortName() == "agent_input") {
                Event new_e;
                std::string type;

                for (const auto& attribute : event->getAttributes()) {
                    new_e.add_property(attribute.first,
                                       attribute.second->clone());
                }
                type = new_e.property("type")->toString().value();
                if(type == "collision"
                   &&
                   new_e.property("to")->toString().value() == getModelName())
                    mScheduler.add_event(new_e);
                else if (type == "ball_position") {
                    ballCollision(new_e);

                }
            }
        }
    }

    void output(const vd::Time&,
                vd::ExternalEventList& output) const
    {
        if (mDirectionChanged) {
            vd::ExternalEvent* event = new vd::ExternalEvent("agent_output");
            event << vd::attribute("x", mPosition.x())
                  << vd::attribute("y", mPosition.y())
                  << vd::attribute("dx", mDirection(0))
                  << vd::attribute("dy", mDirection(1))
                  << vd::attribute("from", getModelName())
                  << vd::attribute("type", "ball_position")
                  << vd::attribute("radius", mRadius);
            output.push_back(event);
        }
    }

    vv::Value* observation(const vd::ObservationEvent& event) const
    {
        vd::Time delta_t = event.getTime() - mLastUpdate;
        double x = (mDirection(0) * delta_t) + mPosition.x();
        double y = (mDirection(1) * delta_t) + mPosition.y();
        if (event.onPort("x")) {
            return new vv::Double(x);
        }
        if (event.onPort("y")) {
            return new vv::Double(y);
        }
        if (event.onPort("coordinates")) {
            std::string output;
            output = "(" + vle::utils::toScientificString(x)
                 + ";" + vle::utils::toScientificString(y) + ")";
                return new vv::String(output);
        }
        return 0;
    }

    void ballCollision(Event new_e)
    {
        // 1. Compute collision with another ball
        // Case 1 : collinear vector + on the same line
        // Case 2 : other type of collision
        // 2. Add collision event in scheduler
        bool collision = false;
        vector event_ball_direction(2);
        vector b_to_b(2);
        vector new_direction(2);
        point event_ball_position;
        point collision_point;

        event_ball_position.x(new_e.property("x")
                              ->toDouble().value());
        event_ball_position.y(new_e.property("y")
                              ->toDouble().value());

        event_ball_direction(0) = new_e.property("dx")->toDouble().value();
        event_ball_direction(1) = new_e.property("dy")->toDouble().value();
        b_to_b(0) = mPosition.x() - event_ball_position.x();
        b_to_b(1) = mPosition.y() - event_ball_position.y();

        if(collinear(mDirection,event_ball_direction)) {
            double dp, dp2;
            dp = event_ball_direction(0)
                 *(mPosition.x()-event_ball_position.x())
                 +event_ball_direction(1)
                 *(mPosition.y()-event_ball_position.y());

            dp2 = mDirection(0)
                 *(event_ball_position.x()-mPosition.x())
                 +mDirection(1)
                 *(event_ball_position.y()-mPosition.y());
            if(collinear(mDirection,b_to_b) && dp > 0 && dp2 >0) { // Case 1
                double R = bn::ublas::norm_2(mDirection) /
                          (bn::ublas::norm_2(mDirection)+
                           bn::ublas::norm_2(event_ball_direction));
                collision_point.x(mPosition.x() + R*b_to_b(0)*-1);
                collision_point.y(mPosition.y() + R*b_to_b(1)*-1);
                collision = true;
                new_direction= mDirection * -1;
            }
        } else {// Case 2
            double det, dot_prod;
            bool exists = true;
            /* Compute collision point
             * ax + by + c = 0 */
            matrix ab(2,2);
            vector c(2);

            //wall equation
            ab(0,0) = -mDirection(1); //a
            ab(0,1) =  mDirection(0); //b
            //c
            c(0) = mDirection(1)*mPosition.x() - mDirection(0)*mPosition.y();

            //ball direction equation
            ab(1,0) = -event_ball_direction(1); //a
            ab(1,1) =  event_ball_direction(0); //b
            c(1) = event_ball_direction(1)*event_ball_position.x() -
               event_ball_direction(0)*event_ball_position.y(); //c

            det = ab(0,0)*ab(1,1) - ab(1,0)*ab(0,1);
            if(det != 0) { // Invertible matrix
            double x,y;
            x = (-1*((ab(1,1)*c(0)) - (ab(0,1)*c(1)))) / det;
            y = (-1*((ab(0,0)*c(1)) - (ab(1,0)*c(0)))) / det;
            collision_point.x(x);
            collision_point.y(y);
            } else {
            exists = false;
            }

            //We check if there is a collision direction or not
            dot_prod = (collision_point.x() - mPosition.x()) * mDirection(0) +
                   (collision_point.y() - mPosition.y()) * mDirection(1);
            if(dot_prod <= 0)
            exists = false;

            dot_prod = (collision_point.x() - event_ball_position.x())
                   * event_ball_direction(0) +
                   (collision_point.y() - event_ball_position.y())
                   * event_ball_direction(1);

            if(dot_prod <= 0)
            exists = false;

            if(exists) {
                double distance, distance_o, time, time_o;
                        distance = bg::distance(mPosition, collision_point);
                        distance_o = bg::distance(event_ball_position, collision_point);
                time = distance / bn::ublas::norm_2(mDirection);
                time_o = distance_o / bn::ublas::norm_2(event_ball_direction);

                if(time == time_o) {
                       collision = true;
                       new_direction = (event_ball_direction/
                                    bn::ublas::norm_2(event_ball_direction))
                                   * bn::ublas::norm_2(mDirection);
                }
            }
        //return std::make_tuple(exists,collision_point);
        }
        // 2. Add event in scheduler
        if(collision) {
            double distance = bg::distance(mPosition, collision_point);
            double time = distance / bn::ublas::norm_2(mDirection);
            addCollisionEvent(collision_point,
                               new_direction,
                               distance,
                               time+mCurrentTime,
                               getModelName());
        }
    }

    void addCollisionEvent(point xy_collision,
                           vector new_vector,
                           double collision_distance,
                           double collision_time,
                           const std::string& ball_name)
    {
        Event new_collision(collision_time);

        new_collision.add_property("new_dx",
                                   new vv::Double(new_vector[0]));
        new_collision.add_property("new_dy",
                                   new vv::Double(new_vector[1]));
        new_collision.add_property("new_x",
                                   new vv::Double(xy_collision.x()));
        new_collision.add_property("new_y",
                                   new vv::Double(xy_collision.y()));
        new_collision.add_property("collision_distance",
                                   new vv::Double(collision_distance));
        new_collision.add_property("type",
                                   new vv::String("collision"));
        new_collision.add_property("to",new vv::String(ball_name));
        mScheduler.add_event(new_collision);
    }
private:
    bool mDirectionChanged;
    point mPosition;
    vector mDirection;
    vd::Time mCurrentTime;
    double mRadius;
};

}
}
} // namespace vle example
DECLARE_DYNAMICS(mas::test::dynamics::Ball)

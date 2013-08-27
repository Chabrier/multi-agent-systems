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

#define toDouble(X) X->toDouble().value()

using namespace vle::extension::mas;

namespace mas
{
namespace test
{
namespace dynamics
{

namespace vu = vle::utils;
namespace vd = vle::devs;
namespace vv = vle::value;
namespace bg = boost::geometry;
namespace bn = boost::numeric;

class BallG : public GenericAgent
{
public:


    /**************************************************************************/
    BallG(const vd::DynamicsInit& init, const vd::InitEventList& events)
        : GenericAgent(init, events)
    {
        mCircle.getCenter().x(events.exist("x") ? events.getDouble("x"):-1);
        mCircle.getCenter().y(events.exist("y") ? events.getDouble("y"):-1);
        mDirection.x() = events.exist("dx") ? events.getDouble("dx") : -1;
        mDirection.y() = events.exist("dy") ? events.getDouble("dy") : -1;
        mCircle.getRadius() = events.exist("radius")
                              ? events.getDouble("radius"):0;

        addEffect("doCollision",
                  boost::bind(&BallG::doCollision,this,_1));
    }


protected:
    void agent_init()
    {
        sendMyInformation();
    }

    void agent_dynamic()
    {
        Effect nextEffect = mScheduler.nextEffect();

        applyEffect(nextEffect.getName(),nextEffect);
    }

    void agent_handleEvent(const Message &message)
    {
        std::string subject = message.getSubject();
        Circle currentCircle = getCurrentCircle();
        if(subject == "ball_position" || subject == "collision_callback") {
            double c2_x = toDouble(message.get("x"));
            double c2_y = toDouble(message.get("y"));
            double c2_dx = toDouble(message.get("dx"));
            double c2_dy = toDouble(message.get("dy"));
            double c2_radius = toDouble(message.get("radius"));
            double date;

            Point p2(c2_x,c2_y);
            Vector2d d2(c2_dx,c2_dy);
            Circle c2(Point(c2_x,c2_y),c2_radius);
            if(currentCircle.inCollision(mDirection,c2,d2)) {
                CollisionPoints cp = currentCircle.collisionPoints(mDirection,
                                                                   c2,
                                                                   d2);
                Vector2d new_direction = currentCircle.newDirection(mDirection,
                                                                    c2,
                                                                    d2);
                double distance = bg::distance(cp.object1CollisionPosition,
                                               currentCircle.getCenter());
                double date = (distance / mDirection.norm()) + mCurrentTime;

                Effect collision = collisionEffect(date,
                                                   message.getSender(),
                                                   cp.object1CollisionPosition,
                                                   new_direction);
                if (!mScheduler.exists(collision))
                    mScheduler.addEffect(collision);
                else
                    mScheduler.update(collision);

                if (subject == "ball_position") {
                    sendCollisionCallback(message.getSender());
                }
            }
        } else if (subject == "collision") {
            double wall_x1 = toDouble(message.get("wall_x1"));
            double wall_y1 = toDouble(message.get("wall_y1"));
            double wall_x2 = toDouble(message.get("wall_x2"));
            double wall_y2 = toDouble(message.get("wall_y2"));

            Segment s(Point(wall_x1,wall_y1),Point(wall_x2,wall_y2));
            if(currentCircle.inCollision(s,mDirection)) {
                CollisionPoints cp = currentCircle.collisionPoints(s,
                                                                   mDirection);
                Vector2d new_direction = currentCircle.newDirection(s,
                                                                   mDirection);
                double date = (bg::distance(cp.object1CollisionPosition,
                                               currentCircle.getCenter())
                              / mDirection.norm()) + mCurrentTime;

                Effect collision = collisionEffect(date,
                                                   message.getSender(),
                                                   cp.object1CollisionPosition,
                                                   new_direction);
                if (!mScheduler.exists(collision))
                    mScheduler.addEffect(collision);
                else
                    mScheduler.update(collision);
            }
        } else if (subject == "collision_sync") {
            Effect e(vd::infinity,
                     message.get("effect")->toString().value(),
                     message.get("origin")->toString().value());
            if (mScheduler.exists(e))
                mScheduler.update(e);
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
            output = str(boost::format("(%1%;%2%;%3%)")
                         % vu::toScientificString(x)
                         % vu::toScientificString(y)
                         % vu::toScientificString(mCircle.getRadius()));
            return new vv::String(output);
        }
        return 0;
    }

    /**************************** Utils ***************************************/
    Circle getCurrentCircle() const
    {
        double delta_t = mCurrentTime - mLastUpdate;
        double x = (mDirection.x() * delta_t) + mCircle.getCenter().x();
        double y = (mDirection.y() * delta_t) + mCircle.getCenter().y();

        return Circle(Point(x,y),mCircle.getRadius());
    }

    void sendMyInformation()
    {
        Message m(getModelName(),Message::BROADCAST,"ball_position");

        m.add("x",vv::Double::create(mCircle.getCenter().x()));
        m.add("y",vv::Double::create(mCircle.getCenter().y()));
        m.add("dx",vv::Double::create(mDirection.x()));
        m.add("dy",vv::Double::create(mDirection.y()));
        m.add("radius",vv::Double::create(mCircle.getRadius()));

        sendMessage(m);
    }

    void sendCollisionCallback(const std::string& to)
    {
        vd::Time delta_t = mCurrentTime - mLastUpdate;
        double x = (mDirection.x() * delta_t) + mCircle.getCenter().x();
        double y = (mDirection.y() * delta_t) + mCircle.getCenter().y();
        Message m(getModelName(),to,"collision_callback");

        m.add("x",vv::Double::create(x));
        m.add("y",vv::Double::create(y));
        m.add("dx",vv::Double::create(mDirection.x()));
        m.add("dy",vv::Double::create(mDirection.y()));
        m.add("radius",vv::Double::create(mCircle.getRadius()));

        sendMessage(m);
    }

    void sendCollisionSync(const Effect& e)
    {
        Message m(getModelName(),e.getOrigin(),"collision_sync");

        m.add("effect",vv::String::create(e.getName()));
        m.add("origin",vv::String::create(getModelName()));

        sendMessage(m);
    }

    /*************************** Effect functions *****************************/
    /* doCollision : What do this effect ?
     * - It computes collision position.
     * - It updates ball direction.
     * - It sends informative message to all agents after the update. */
    void doCollision(const Effect& e)
    {
        double x =  toDouble(e.get("x"));
        double y =  toDouble(e.get("y"));
        double dx = toDouble(e.get("dx"));
        double dy = toDouble(e.get("dy"));

        /* Apply effect */
        mDirection = Vector2d(dx,dy);
        mCircle.getCenter() = Point(x,y);

        /* Send my information */
        Scheduler<Effect> tmp = mScheduler;
        std::for_each(tmp.elements().begin(),
                      tmp.elements().end(),
                      [this](Effect effect) {
                          //this->sendCollisionSync(effect);
                          Effect e(vd::infinity,
                                   effect.getName(),
                                   effect.getOrigin());
                          this->mScheduler.update(e);
                      });
        sendMyInformation();
    }

    /**************************** Effect "factory" ****************************/
    Effect collisionEffect(double t,const std::string& source,
                           const Point& position,const Vector2d& direction)
    {
        Effect effect(t,"doCollision",source);

        effect.add("x",vv::Double::create(position.x()));
        effect.add("y",vv::Double::create(position.y()));
        effect.add("dx",vv::Double::create(direction.x()));
        effect.add("dy",vv::Double::create(direction.y()));

        return effect;
    }
private:
    Circle   mCircle;
    Vector2d mDirection;
};

}}} //namespace mas test dynamics
DECLARE_DYNAMICS(mas::test::dynamics::BallG)

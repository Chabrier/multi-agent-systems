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
            double c2_x = toDouble(message.getInformations().at("x"));
            double c2_y = toDouble(message.getInformations().at("y"));
            double c2_dx = toDouble(message.getInformations().at("dx"));
            double c2_dy = toDouble(message.getInformations().at("dy"));
            double c2_radius = toDouble(message.getInformations().at("radius"));
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
                                                cp.object1CollisionPosition.x(),
                                                cp.object1CollisionPosition.y(),
                                                   new_direction.x(),
                                                   new_direction.y());
                mScheduler.addEffect(collision);
                if (subject == "ball_position") {
                    sendCollisionCallback(message.getSender());
                }
            }
        } else if (subject == "collision") {
            double wall_x1 = toDouble(message.getInformations().at("wall_x1"));
            double wall_y1 = toDouble(message.getInformations().at("wall_y1"));
            double wall_x2 = toDouble(message.getInformations().at("wall_x2"));
            double wall_y2 = toDouble(message.getInformations().at("wall_y2"));

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
                                                cp.object1CollisionPosition.x(),
                                                cp.object1CollisionPosition.y(),
                                                   new_direction.x(),
                                                   new_direction.y());
                mScheduler.addEffect(collision);
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
        Message::property_map& pm = m.getInformations();

        Message::value_ptr x, y, dx, dy, radius;

        x = Message::value_ptr(vv::Double::create(mCircle.getCenter().x()));
        y = Message::value_ptr(vv::Double::create(mCircle.getCenter().y()));
        dx = Message::value_ptr(vv::Double::create(mDirection.x()));
        dy = Message::value_ptr(vv::Double::create(mDirection.y()));
        radius = Message::value_ptr(vv::Double::create(mCircle.getRadius()));

        pm.insert(std::make_pair("x",x));
        pm.insert(std::make_pair("y",y));
        pm.insert(std::make_pair("dx",dx));
        pm.insert(std::make_pair("dy",dy));
        pm.insert(std::make_pair("radius",radius));

        sendMessage(m);
    }

    void sendCollisionCallback(const std::string& to)
    {
        vd::Time delta_t = mCurrentTime - mLastUpdate;
        double x = (mDirection.x() * delta_t) + mCircle.getCenter().x();
        double y = (mDirection.y() * delta_t) + mCircle.getCenter().y();
        Message m(getModelName(),to,"collision_callback");
        Message::property_map& pm = m.getInformations();
        pm.insert(std::make_pair("x",
                                    Message::value_ptr(vv::Double::create(x))));
        pm.insert(std::make_pair("y",
                                    Message::value_ptr(vv::Double::create(y))));
        pm.insert(std::make_pair("dx",
                       Message::value_ptr(vv::Double::create(mDirection.x()))));
        pm.insert(std::make_pair("dy",
                       Message::value_ptr(vv::Double::create(mDirection.y()))));
        pm.insert(std::make_pair("radius",
                  Message::value_ptr(vv::Double::create(mCircle.getRadius()))));

        sendMessage(m);
    }

    /*************************** Effect functions *****************************/
    /* doCollision : What do this effect ?
     * - It computes collision position.
     * - It updates ball direction.
     * - It sends informative message to all agents after the update. */
    void doCollision(const Effect& e)
    {
        double x = toDouble(e.getInformations().at("x"));
        double y = toDouble(e.getInformations().at("y"));
        double dx = toDouble(e.getInformations().at("dx"));
        double dy = toDouble(e.getInformations().at("dy"));

        /* Apply effect */
        mDirection = Vector2d(dx,dy);
        mCircle.getCenter() = Point(x,y);

        /* Send my information */
        sendMyInformation();

        /* */
        while (!mScheduler.empty()) {
            mScheduler.removeNextEffect();
        }
    }

    /**************************** Effect "factory" ****************************/
    Effect collisionEffect(double t,const std::string& source,
                           double x,double y,double dx,double dy)
    {
        Effect effect(t,"doCollision",source);
        Effect::property_map &collisionProperties = effect.getInformations();
        collisionProperties.insert(std::make_pair("x",
                                     Effect::value_ptr(vv::Double::create(x))));
        collisionProperties.insert(std::make_pair("y",
                                     Effect::value_ptr(vv::Double::create(y))));
        collisionProperties.insert(std::make_pair("dx",
                                    Effect::value_ptr(vv::Double::create(dx))));
        collisionProperties.insert(std::make_pair("dy",
                                    Effect::value_ptr(vv::Double::create(dy))));

        return effect;
    }
private:
    Circle   mCircle;
    Vector2d mDirection;
};

}}} //namespace mas test dynamics
DECLARE_DYNAMICS(mas::test::dynamics::BallG)

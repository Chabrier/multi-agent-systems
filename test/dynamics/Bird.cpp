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
#include <vle/devs/DynamicsDbg.hpp>

#include <vle/utils/Tools.hpp>

#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometry.hpp>
#include <boost/geometry/geometries/linestring.hpp>

#include<math.h>

#include <vle/extension/mas/GenericAgent.hpp>
#include <vle/extension/mas/collision/Vector2d.hpp>
#include <vle/extension/mas/collision/Types.hpp>
#include <vle/extension/mas/collision/Circle.hpp>

#define toDouble(X) X->toDouble().value()

using namespace vle::extension::mas;
namespace vd = vle::devs;

typedef boost::geometry::model::linestring<Point> Line;

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

class BirdInfo
{
public:

    BirdInfo(double x, double y, double dx, double dy)
        : mX(x), mY(y), mXDirection(dx), mYDirection(dy)
    {
    }

    double mX;
    double mY;
    double mXDirection;
    double mYDirection;
};

class Bird : public GenericAgent
{
public:

    Bird(const vd::DynamicsInit& init, const vd::InitEventList& events)
        : GenericAgent(init, events)
    {
        mCircle.getCenter().x(events.exist("x") ? events.getDouble("x"):-1);
        mCircle.getCenter().y(events.exist("y") ? events.getDouble("y"):-1);
        mDirection.x() = events.exist("dx") ? events.getDouble("dx") : -1;
        mDirection.y() = events.exist("dy") ? events.getDouble("dy") : -1;
        mCircle.getRadius() = events.exist("radius")
            ? events.getDouble("radius"):0;

        mSeparation  = events.exist("separation") ? events.getDouble("separation") : 2;
        mMaxSeparateTurn  = events.exist("maxSeparateTurn") ? events.getDouble("maxSeparateTurn") : 3;
        mMaxAlignTurn  = events.exist("maxAlignTurn") ? events.getDouble("maxAlignTurn") : 5;

        addEffect("enterAgain",
                  boost::bind(&Bird::enterAgain,this,_1));

        addEffect("updateAccordingNeighborhood",
                  boost::bind(&Bird::updateAccordingNeighborhood,this,_1));

        addEffect("enterOrLeaveNeighborhood",
                   boost::bind(&Bird::enterOrLeaveNeighborhood,this,_1));
    }


protected:
    void agent_init()
    {
        sendBirdInformation();

        Effect update = updateAccordingNeighborhoodEffect(mCurrentTime + 1,
                                                          getModelName()+"toto");

        mScheduler.addEffect(update);
    }

    void agent_dynamic()
    {
        Effect nextEffect = mScheduler.nextEffect();

        applyEffect(nextEffect.getName(),nextEffect);
    }

    void agent_handleEvent(const vd::ExternalEvent* message)
    {
        std::string subject = message->getAttributeValue("subject").toString().value();
        if( subject == "enterAgain") {
            double north = message->getDoubleAttributeValue("north");
            double south = message->getDoubleAttributeValue("south");
            double east = message->getDoubleAttributeValue("east");
            double west = message->getDoubleAttributeValue("west");

            // calcul de la + grande dimension du ciel + une marge
            Vector2d diag(north - south, east - west);

            double bigDim = diag.norm() + 11;

            // on calcul le point d'intersection
            Vector2d dirBird = mDirection.normalize();
            Circle currentBird = getCurrentCircle();

            double xOutside = currentBird.getCenter().x() +  dirBird.x() * bigDim;
            double yOutside = currentBird.getCenter().y() +  dirBird.y() * bigDim;

            Line trajectoire;

            // La trajectoire
            trajectoire.push_back(Point(currentBird.getCenter().x(), currentBird.getCenter().y()));

            trajectoire.push_back(Point(xOutside, yOutside));

            //Le ciel
            Line ciel;

            ciel.push_back(Point(east , north));
            ciel.push_back(Point(west, north));
            ciel.push_back(Point(west, south));
            ciel.push_back(Point(east, south));
            ciel.push_back(Point(east, north));

            // intersection
            std::vector<Point> intersection;
            boost::geometry::intersection(trajectoire, ciel, intersection);
            // durée
            if (intersection.size() == 0 ) {
                throw vle::utils::ModellingError(str(boost::format("MODELING_ERROR: The bird seems have leaved the sky(%1%;%2%;%3%;%4%;%5%;%6%;%7%)")
                                                     % (mCurrentTime - mLastUpdate)
                                                     % mCircle.getCenter().x()
                                                     % mCircle.getCenter().y()
                                                     % currentBird.getCenter().x()
                                                     % currentBird.getCenter().y()
                                                     % xOutside
                                                     % yOutside));
            }

            Vector2d centerWall(intersection.at(0).x()-currentBird.getCenter().x(),
                                intersection.at(0).y()-currentBird.getCenter().y());

            double centerWallDistance = centerWall.norm();

            double date = (centerWallDistance / mDirection.norm()) + mCurrentTime;

            if (date <= mCurrentTime)
                date = mCurrentTime;

            // another way to cross the sky

            double xInterOp, yInterOp;

            xInterOp = intersection.at(0).x();
            yInterOp = intersection.at(0).y();

            if (intersection.at(0).x() <= east + 0.01)
                xInterOp = west - 0.1;
            if (intersection.at(0).x() >= west - 0.01)
                xInterOp = east + 0.1;
            if (intersection.at(0).y() <= north + 0.01)
                yInterOp = south - 0.1;
            if (intersection.at(0).y() >= south - 0.01)
                yInterOp = north + 0.1;


            // // La trajectoire opposée
            // Line trajectoireOpposee;
            // xOutside = currentBird.getCenter().x() +  (-dirBird.x() * bigDim);
            // yOutside = currentBird.getCenter().y() +  (-dirBird.y() * bigDim);
            // double xBirdMoreInside = (currentBird.getCenter().x() +  intersection.at(0).x())/2;
            // double yBirdMoreInside = (currentBird.getCenter().y() +  intersection.at(0).y())/2;

            // // La trajectoire
            // trajectoireOpposee.push_back(Point(xBirdMoreInside, yBirdMoreInside));
            // trajectoireOpposee.push_back(Point(xOutside, yOutside));
            // // intersection
            // std::vector<Point> intersectionOpposee;
            //  //Le ciel intérieur
            // Line interieurCiel;

            // interieurCiel.push_back(Point(east + 0.1, north + 0.1));
            // interieurCiel.push_back(Point(west - 0.1, north + 0.1));
            // interieurCiel.push_back(Point(west - 0.1, south - 0.1));
            // interieurCiel.push_back(Point(east + 0.1, south - 0.1));
            // interieurCiel.push_back(Point(east + 0.1, north + 0.1));

            // boost::geometry::intersection(trajectoireOpposee, interieurCiel, intersectionOpposee);

            // double xInterOp, yInterOp;

            // if (intersectionOpposee.size() == 2 ) {
            //     if( Vector2d(intersectionOpposee.at(0).x() - xBirdMoreInside,
            //                  intersectionOpposee.at(0).y() - yBirdMoreInside ).norm()
            //         < Vector2d(intersectionOpposee.at(1).x() - xBirdMoreInside,
            //                    intersectionOpposee.at(1).y() - yBirdMoreInside).norm()) {
            //         xInterOp = intersectionOpposee.at(1).x();
            //         yInterOp = intersectionOpposee.at(1).y();
            //     } else {
            //         xInterOp = intersectionOpposee.at(0).x();
            //         yInterOp = intersectionOpposee.at(0).y();
            //     }
            // } else {
            //     xInterOp = intersectionOpposee.at(0).x();
            //     yInterOp = intersectionOpposee.at(0).y();
            // }



            Effect enterAgain = enterAgainEffect(date,
                                                 message->getAttributeValue("sender").toString().value(),
                                                 xInterOp,
                                                 yInterOp);


            if (!mScheduler.exists(enterAgain))
                mScheduler.addEffect(enterAgain);
            else
                mScheduler.update(enterAgain);
        } else if (subject == "birdPosition") {
            double x = message->getDoubleAttributeValue("x");
            double y = message->getDoubleAttributeValue("y");
            double dx = message->getDoubleAttributeValue("dx");
            double dy = message->getDoubleAttributeValue("dy");

            //update the voisinage
            std::map< std::string, BirdInfo* >::const_iterator it;
            it = mVoisinage.find(message->getAttributeValue("sender").toString().value());

            //TODO
            if (it != mVoisinage.end() )
            {
                ((*it).second)->mX = x;
                ((*it).second)->mY = y;
                ((*it).second)->mXDirection = dx;
                ((*it).second)->mYDirection = dy;
            }

            // first we need when and if a collision will occur

            Point p(x,y);
            Vector2d d(dx, dy);
            Circle c(p,0);

            Circle voisinage(getCurrentCircle().getCenter(), 5);

            if(voisinage.inCollision(mDirection,c,d)) {
                CollisionPoints cp = voisinage.collisionPoints(mDirection,
                                                               c,
                                                               d);

                double distance = bg::distance(cp.object1CollisionPosition,
                                               getCurrentCircle().getCenter());


                double date = (distance / mDirection.norm()) + mCurrentTime;

                Effect enterOrLeaveNeighborhood = enterOrLeaveNeighborhoodEffect(date,
                                                                                 message->getAttributeValue("sender").toString().value(),
                                                                                 x, y, dx, dy);

                if (!mScheduler.exists(enterOrLeaveNeighborhood))
                    mScheduler.addEffect(enterOrLeaveNeighborhood);
                else
                    mScheduler.update(enterOrLeaveNeighborhood);

            } else {
                if (mVoisinage.find((message->getAttributeValue("sender").toString().value())) != mVoisinage.end()) {
                    mVoisinage.erase(message->getAttributeValue("sender").toString().value());
                }

                Effect enterOrLeaveNeighborhood = enterOrLeaveNeighborhoodEffect(vd::infinity,
                                                                                 message->getAttributeValue("sender").toString().value(),
                                                                                 x, y, dx, dy);

                if (!mScheduler.exists(enterOrLeaveNeighborhood))
                    mScheduler.addEffect(enterOrLeaveNeighborhood);
                else
                    mScheduler.update(enterOrLeaveNeighborhood);
            }
        } else if (subject == "askBirdPosition") {
            sendCurrentBirdInformation();
        }
    }

    vv::Value* observation(const vd::ObservationEvent& event) const
    {
        vd::Time delta_t = event.getTime() - mLastUpdate;

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
        if (event.onPort("coordinates&Headings")) {
            std::string output;
            output = str(boost::format("(%1%;%2%;%3%;%4%;%5%)")
                         % vu::toScientificString(x)
                         % vu::toScientificString(y)
                         % vu::toScientificString(mCircle.getRadius())
                         % vu::toScientificString(mDirection.x())
                         % vu::toScientificString(mDirection.y()));
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

    void sendBirdInformation()
    {
        Message m(getModelName(),Message::BROADCAST,"birdPosition");

        m.add("x",vv::Double::create(mCircle.getCenter().x()));
        m.add("y",vv::Double::create(mCircle.getCenter().y()));
        m.add("dx",vv::Double::create(mDirection.x()));
        m.add("dy",vv::Double::create(mDirection.y()));
        m.add("radius",vv::Double::create(mCircle.getRadius()));

        sendMessage(m);
    }

    void sendCurrentBirdInformation()
    {
        Message m(getModelName(),Message::BROADCAST,"birdPosition");

        m.add("x",vv::Double::create(getCurrentCircle().getCenter().x()));
        m.add("y",vv::Double::create(getCurrentCircle().getCenter().y()));
        m.add("dx",vv::Double::create(mDirection.x()));
        m.add("dy",vv::Double::create(mDirection.y()));
        m.add("radius",vv::Double::create(mCircle.getRadius()));

        sendMessage(m);
    }

    void sendAskForInformation(const std::string& to)
    {
        Message m(getModelName(),to,"askBirdPosition");

        sendMessage(m);
    }

    /*************************** Effect functions *****************************/
    /* enterAgain : What do this effect ?
     * - It does enable to renter the sky*/
    void enterAgain(const Effect& e)
    {
        double x =  toDouble(e.get("newX"));
        double y =  toDouble(e.get("newY"));

        mCircle.getCenter() = Point(x,y);

        sendBirdInformation();
        sendAskForInformation(Message::BROADCAST);
    }

    void updateAccordingNeighborhood(const Effect& e)
    {
        if (mVoisinage.size() != 0) { // if any flockmates

            //find-nearest-neighbor

            std::map< std::string, BirdInfo* >::const_iterator it, minIt;

            it = minIt = mVoisinage.begin();
            it++;

            while ( it != mVoisinage.end()) {
                double x = ((*it).second)->mX;
                double y = ((*it).second)->mY;
                double minX = ((*minIt).second)->mX;
                double minY = ((*minIt).second)->mY;
                double cx= mCircle.getCenter().x();
                double cy= mCircle.getCenter().y();

                if (Vector2d(x - cx, y - cy).norm() <
                    Vector2d(minX - cx, minY - cy).norm()) {
                    minIt = it;
                }
                ++it;
            }

            double minX = ((*minIt).second)->mX;
            double minY = ((*minIt).second)->mY;
            double cx= mCircle.getCenter().x();
            double cy= mCircle.getCenter().y();

            if (Vector2d(minX - cx, minY - cy).norm() < mSeparation) {
                //separate
                double dx = ((*minIt).second)->mXDirection;
                double dy = ((*minIt).second)->mYDirection;

                double tmpangle = angle(Vector2d(dx, dy), mDirection);

                if (abs(tmpangle / M_PI * 180) > mMaxSeparateTurn) {
                    if (tmpangle / M_PI * 180 < 0) {
                        tmpangle = - (mMaxSeparateTurn/ 180 * M_PI);
                    } else {
                        tmpangle = (mMaxSeparateTurn/ 180 * M_PI);
                    }
                }

                mCircle = getCurrentCircle();
                //mDirection = Vector2d(dx, dy);

                mDirection.rotate(tmpangle);

            } else {
                // align ...cohere

                double dx = 0;
                double dy = 0;

                std::map< std::string, BirdInfo* >::const_iterator it;

                for(it = mVoisinage.begin(); it != mVoisinage.end(); ++it){
                    dx += ((*it).second)->mXDirection;
                    dy += ((*it).second)->mYDirection;
                }

                dx /= mVoisinage.size();
                dy /= mVoisinage.size();

                double tmpangle = angle(mDirection, Vector2d(dx, dy));

                if (abs(tmpangle / M_PI * 180) > mMaxAlignTurn) {
                    if (tmpangle / M_PI * 180 < 0) {
                        tmpangle = - (mMaxAlignTurn/ 180 * M_PI);
                    } else {
                        tmpangle = (mMaxAlignTurn/ 180 * M_PI);
                    }
                }

                //tmpangle /=10;

                double andegre = tmpangle / M_PI * 180; // reverse x / 180 * M_PI

                mCircle = getCurrentCircle();
                //mDirection = Vector2d(dx, dy);

                Vector2d mPrevious = mDirection;

                mDirection.rotate(tmpangle);

            }

            sendBirdInformation();
            sendAskForInformation(Message::BROADCAST);
        } else {
             mCircle = getCurrentCircle();
        }


        Effect update = updateAccordingNeighborhoodEffect(mCurrentTime + 1.5,
                                                          getModelName()+"toto");

        mScheduler.update(update);
    }

    void enterOrLeaveNeighborhood(const Effect& e)
    {
        mCircle = getCurrentCircle();
        Effect effect(vd::infinity,
                      e.getName(),
                      e.getOrigin());
        this->mScheduler.update(effect);

        double x = toDouble(e.get("x"));
        double y = toDouble(e.get("y"));
        double dx = toDouble(e.get("dx"));
        double dy = toDouble(e.get("dy"));

        if (mVoisinage.find((e.getOrigin())) != mVoisinage.end()) {
            mVoisinage.erase(e.getOrigin());
        } else {
            mVoisinage[e.getOrigin()] = new BirdInfo(x, y, dx, dy);
        }
    }

    /**************************** Effect "factory" ****************************/
    Effect enterAgainEffect(double t,const std::string& source, double x, double y)
    {
        Effect effect(t,"enterAgain",source);

        effect.add("newX",vv::Double::create(x));
        effect.add("newY",vv::Double::create(y));

        return effect;
    }

    Effect enterOrLeaveNeighborhoodEffect(double t,const std::string& source,
                                          double x, double y,
                                          double dx, double dy)
    {
        Effect effect(t,"enterOrLeaveNeighborhood",source);

        effect.add("x",vv::Double::create(x));
        effect.add("y",vv::Double::create(y));
        effect.add("dx",vv::Double::create(dx));
        effect.add("dy",vv::Double::create(dy));

        return effect;
    }

    /**************************** Effect "factory" ****************************/
    Effect updateAccordingNeighborhoodEffect(double t,const std::string& source)
    {
        Effect effect(t,"updateAccordingNeighborhood",source);

        return effect;
    }

    double trunc_doub(double val, int precision)
    {
        return floorf(val * pow(10.0f,precision) + .5f)/pow(10.0f,precision);
    }

private:

    Circle   mCircle;
    Vector2d mDirection;
    std::map< std::string, BirdInfo*> mVoisinage;

    double mSeparation;
    double mMaxSeparateTurn;
    double mMaxAlignTurn;
};

}}} //namespace mas test dynamics
DECLARE_DYNAMICS_DBG(mas::test::dynamics::Bird)

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

#include <boost/lexical_cast.hpp>

#include <vle/value/Value.hpp>
#include <vle/devs/Executive.hpp>
#include <vle/value/Map.hpp>
#include <vle/utils/Rand.hpp>

#include<math.h>

#include <vle/extension/mas/collision/Vector2d.hpp>

namespace vd = vle::devs;
namespace vv = vle::value;
namespace vp = vle::vpz;
namespace vu = vle::utils;

namespace mas
{
namespace test
{
namespace dynamics
{

class God : public vd::Executive
{
public:
    typedef std::vector<std::string> Strings;
    God(const vd::ExecutiveInit& init,
                     const vd::InitEventList& events)
    : vd::Executive(init, events),mBirdNumbers(0)
    {

        // Les limites du ciel

        mNorth = events.exist("north") ? events.getDouble("north") : 0;
        mSouth = events.exist("south") ? events.getDouble("south") : 70;
        mEast = events.exist("east") ? events.getDouble("east") : 0;
        mWest = events.exist("west") ? events.getDouble("west") : 70;

        // Le rayon du voisinage des oiseaux

        mRadiusMin = events.exist("radiusMin") ? events.getDouble("radiusMin") : 5;
        mRadiusMax = events.exist("radiusMax") ? events.getDouble("radiusMax") : 5;

        // La vitesse des oiseaux

        mSpeedMin  = events.exist("speedMin") ? events.getDouble("speedMin") : 1;
        mSpeedMax = events.exist("speedMax") ? events.getDouble("speedMax") : 1;

        // Et la taille de la population

        mPopulation = events.exist("population") ? events.getDouble("population") : 10;

        mView = "view1";

        vp::Dynamic dyn("dynBird");
        dyn.setPackage("vle.extension.mas");
        dyn.setLibrary("Bird");
        dynamics().add(dyn);

        vp::Dynamic dyns("dynSky");
        dyns.setPackage("vle.extension.mas");
        dyns.setLibrary("Sky");
        dynamics().add(dyns);
    }

    vd::Time init(const vd::Time&)
    {
        createSky(mNorth, mSouth, mEast, mWest);

        for(int i = 1; i <= mPopulation; ++i){

            double x = mRand.getDouble(mEast + mRadiusMax, mWest - mRadiusMax);
            double y = mRand.getDouble(mNorth + mRadiusMax, mSouth - mRadiusMax);

            while (inANeigborhood(x, y)) {
                x = mRand.getDouble(mEast + mRadiusMax, mWest - mRadiusMax);
                y = mRand.getDouble(mNorth + mRadiusMax, mSouth - mRadiusMax);

            }

            double radius =  (mRadiusMin == mRadiusMax) ? mRadiusMin :
                mRand.getDouble(mRadiusMin, mRadiusMax);
            double speed =  (mSpeedMin == mSpeedMax) ? mSpeedMin :
                mRand.getDouble(mSpeedMin, mSpeedMax);

            double angle = mRand.getDouble(0, 360);

            Vector2d heading = Vector2d(0, 1).rotate(angle/180 * M_PI);

            createBird(x, y, heading.x()*speed, heading.y()*speed, radius);
        }

        return vd::infinity;
    }

    void createSky(double n, double s, double e, double w)
    {

        vp::Condition wall_cond("condSky");
        wall_cond.add("north");
        wall_cond.add("south");
        wall_cond.add("esat");
        wall_cond.add("west");
        wall_cond.addValueToPort("north", vv::Double::create(n));
        wall_cond.addValueToPort("south", vv::Double::create(s));
        wall_cond.addValueToPort("east", vv::Double::create(e));
        wall_cond.addValueToPort("west", vv::Double::create(w));
        conditions().add(wall_cond);

        createModel("Sky",
                    Strings({"agent_input"}),
                    Strings({"agent_output"}),
                    "dynSky",
                    Strings({wall_cond.name()}),
                    "");
        connect("Sky");
        mModels.push_back("Sky");
    }

    void createBird(double x, double y, double dx, double dy, double radius)
    {
        std::string id = boost::lexical_cast<std::string>(mBirdNumbers);
        std::string obs_ports[] = {"coordinates"};
        std::map<std::string,vv::Value*> cond_map =
            {{"x",vv::Double::create(x)},
             {"y",vv::Double::create(y)},
             {"dx",vv::Double::create(dx)},
             {"dy",vv::Double::create(dy)},
             {"radius",vv::Double::create(radius)}};
        //Create experimental conditions
        vp::Condition condBird("condBird"+id);
        for(const auto& it : cond_map) {
            condBird.add(it.first);
            condBird.addValueToPort(it.first,it.second);
        }
        conditions().add(condBird);

        //Create observables
        vp::Observable obsBird("bird"+id+"_position");
        for(const auto& it : obs_ports)
            obsBird.add(it);
        observables().add(obsBird);

        //Create vle model
        createModel("bird" + id,
                    Strings({"agent_input"}),
                    Strings({"agent_output"}),
                    "dynBird",
                    Strings({condBird.name()}),
                    obsBird.name());

        //Attach observables
        for(const auto& it : obs_ports)
            addObservableToView("bird" + id, it, mView);

        //Connect to others
        connect("bird" + id);
        mModels.push_back("bird" + id);

        // keep the coordinates also
        mKeepCoordinates["bird" + id] = std::array< double, 3 > {x,y,radius};

        ++mBirdNumbers;
    }

    bool inANeigborhood(double x, double y)
    {
        if (mKeepCoordinates.size() == 0) {
            return false;
        }

        for(const auto& it : mKeepCoordinates) {
            double xOther = it.second[0];
            double yOther = it.second[1];
            double radius = it.second[2];

            double distC2C = Vector2d(xOther - x, yOther - y).norm();

            if (distC2C <= radius) {
                return true;
            }
        }
        return false;
    }


    void connect(const std::string& new_agent)
    {
        for (auto existing_agent : mModels)
        {
            addConnection(new_agent,
                          "agent_output",
                          existing_agent,
                          "agent_input");
            addConnection(existing_agent,
                          "agent_output",
                          new_agent,
                          "agent_input");
        }
    }
private:
    int mBirdNumbers;
    Strings mModels;
    std::string mView;

    typedef std::map< std::string, std::array<double, 3>> KeepCordinates;

    KeepCordinates mKeepCoordinates;

    double mNorth;
    double mSouth;
    double mEast;
    double mWest;

    vu::Rand mRand;

    double mRadiusMin;
    double mRadiusMax;
    double mSpeedMin;
    double mSpeedMax;
    double mPopulation;

};

}
}
} // namespace vle example
DECLARE_EXECUTIVE(mas::test::dynamics::God)

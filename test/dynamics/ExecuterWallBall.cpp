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
#include <vle/extension/mas/Agent.hpp>

namespace vd = vle::devs;
namespace vv = vle::value;
namespace vp = vle::vpz;

namespace mas
{
namespace test
{
namespace dynamics
{

class ExecuterWallBall : public vd::Executive
{
public:
    typedef std::vector<std::string> Strings;
    ExecuterWallBall(const vd::ExecutiveInit& init,
                     const vd::InitEventList& events)
    : vd::Executive(init, events),mWallNumbers(0),mBallNumbers(0)
    {
        mView = "view1";

        vp::Dynamic dyn("dyn_ball");
        dyn.setPackage("vle.extension.mas");
        dyn.setLibrary("Ball");
        dynamics().add(dyn);

        vp::Dynamic dynw("dyn_wall");
        dynw.setPackage("vle.extension.mas");
        dynw.setLibrary("SimpleWall");
        dynamics().add(dynw);
    }

    vd::Time init(const vd::Time&)
    {
        createWall(0, 0, 0, 4);
        createWall(0, 4, 10, 4);
        createWall(10, 0, 10, 4);
        createWall(0, 0, 10, 0);
        createWall(7, 1, 7, 3);
        createBall(1, 1, 1, 1);
        createBall(1, 1, 0, 1);
        createBall(8, 3, 1, 0);

        for (auto it : mModels)
            std::cout << it << std::endl;

        return vd::infinity;
    }

    void createWall(double x1, double y1, double x2, double y2)
    {
        std::string id = boost::lexical_cast<std::string>(mWallNumbers);

        vp::Condition wall_cond("wall_cond"+id);
        wall_cond.add("x1");
        wall_cond.add("x2");
        wall_cond.add("y1");
        wall_cond.add("y2");
        wall_cond.addValueToPort("x1", vv::Double::create(x1));
        wall_cond.addValueToPort("y1", vv::Double::create(y1));
        wall_cond.addValueToPort("x2", vv::Double::create(x2));
        wall_cond.addValueToPort("y2", vv::Double::create(y2));
        conditions().add(wall_cond);

        createModel("wall" + id,
                    Strings({"agent_input"}),
                    Strings({"agent_output"}),
                    "dyn_wall",
                    Strings({wall_cond.name()}),
                    "");
        connect("wall" + id);
        mModels.push_back("wall" + id);
        ++mWallNumbers;
    }

    void createBall(double x, double y, double dx, double dy)
    {
        std::string id = boost::lexical_cast<std::string>(mBallNumbers);

        //Create experimental conditions
        vp::Condition ball_cond("ball_cond"+id);
        ball_cond.add("x");
        ball_cond.add("y");
        ball_cond.add("dx");
        ball_cond.add("dy");
        ball_cond.addValueToPort("x", vv::Double::create(x));
        ball_cond.addValueToPort("y", vv::Double::create(y));
        ball_cond.addValueToPort("dx", vv::Double::create(dx));
        ball_cond.addValueToPort("dy", vv::Double::create(dy));
        conditions().add(ball_cond);

        //Create observables
        vp::Observable ball_obs("ball"+id+"_position");
        ball_obs.add("x");
        ball_obs.add("y");
        observables().add(ball_obs);

        //Create vle model
        createModel("ball" + id,
                    Strings({"agent_input"}),
                    Strings({"agent_output"}),
                    "dyn_ball",
                    Strings({ball_cond.name()}),
                    ball_obs.name());

        //Attach observables
        addObservableToView("ball" + id, "x", mView);
        addObservableToView("ball" + id, "y", mView);

        //Connect to others
        connect("ball" + id);
        mModels.push_back("ball" + id);

        ++mBallNumbers;
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
    int mWallNumbers;
    int mBallNumbers;
    Strings mModels;
    std::string mView;
};

}
}
} // namespace vle example
DECLARE_EXECUTIVE(mas::test::dynamics::ExecuterWallBall)

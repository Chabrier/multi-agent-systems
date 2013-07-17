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

#include <boost/geometry/geometries/point_xy.hpp>

#include <vle/extension/mas/ActiveAgent.hpp>
#include <vle/extension/mas/Events.hpp>

namespace vd = vle::devs;
namespace vv = vle::value;
namespace bg = boost::geometry;

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
        : ActiveAgent(init, events)
    {
        mmDirectionChanged = true;
        mPosition.x((events.exist("x")) ? events.getDouble("x") : -1);
        mPosition.y((events.exist("y")) ? events.getDouble("y") : -1);
        mDirection.x((events.exist("dx")) ? events.getDouble("dx") : -1);
        mDirection.y((events.exist("dy")) ? events.getDouble("dy") : -1);
    }

    void init() { }

    vd::Time init(const vd::Time& time)
    {
        mLastUpdate = time;
        return 0;
    }

    vd::Time timeAdvance() const
    {
        if (!mScheduler.empty()) {
            return mScheduler.next_event()["time"]->toDouble().value();
        }

        if (mmDirectionChanged)
            return 0;

        return vd::infinity;
    }

    void internalTransition(const vd::Time& time)
    {
        mmDirectionChanged = false;
        if (!mScheduler.empty()) {
            double delta_t = time - mLastUpdate;
            Event next_event = mScheduler.next_event();

            mScheduler.remove_next_event();

            mPosition.x((mDirection.x() * delta_t) + mPosition.x());
            mPosition.y((mDirection.y() * delta_t) + mPosition.y());

            if (next_event["type"]->toString().value() == "collision") {
                double n_dx = mDirection.x(), n_dy = mDirection.y();
                bool isInCorner = false;

                for (auto event : mScheduler.elements()) {
                    if (event.property("time")->toDouble().value() ==
                        next_event["time"]->toDouble().value()) {
                        if (event.property("type")->toString().value()
                            == "collision") {
                            isInCorner = true;
                        }
                    }
                }

                if (!isInCorner) {
                    n_dx = next_event["new_dx"]->toDouble().value();
                    n_dy = next_event["new_dy"]->toDouble().value();
                } else {
                    n_dx *= -1;
                    n_dy *= -1;
                }


                mDirection.x(n_dx);
                mDirection.y(n_dy);
                mmDirectionChanged = true;
            }

            while (!mScheduler.empty()) {
                mScheduler.remove_next_event();
            }

            mLastUpdate = time;
        }
    }

    void externalTransition(const vd::ExternalEventList& events,
                            const vd::Time&)
    {
        for (auto event : events) {
            if (event->getPortName() == "agent_input") {
                Event new_e;
                new_e.add_property("type", new vv::String("collision"));

                for (auto attribute : event->getAttributes()) {
                    new_e.add_property(attribute.first,
                                       attribute.second->clone());
                }
                if(new_e.property("type")->toString().value() == "collision")
                    mScheduler.add_event(new_e);
            }
        }
    }

    void output(const vd::Time&,
                vd::ExternalEventList& output) const
    {
        if (mmDirectionChanged) {
            vd::ExternalEvent* event = new vd::ExternalEvent("agent_output");
            event << vd::attribute("x", mPosition.x())
                  << vd::attribute("y", mPosition.y())
                  << vd::attribute("dx", mDirection.x())
                  << vd::attribute("dy", mDirection.y())
                  << vd::attribute("type", "ball_position");
            output.push_back(event);
        }
    }

    vv::Value* observation(const vd::ObservationEvent& event) const
    {
        vd::Time delta_t = event.getTime() - mLastUpdate;

        if (event.onPort("x")) {
            return new vv::Double((mDirection.x() * delta_t) + mPosition.x());
        }
        if (event.onPort("y")) {
            return new vv::Double((mDirection.y() * delta_t) + mPosition.y());
        }
        return 0;
    }

private:
    bool mmDirectionChanged;
    bg::model::d2::point_xy<double> mPosition;
    bg::model::d2::point_xy<double> mDirection;
};

}
}
} // namespace vle example
DECLARE_DYNAMICS(mas::test::dynamics::Ball)

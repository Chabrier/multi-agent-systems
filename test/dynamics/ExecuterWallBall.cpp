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
    ExecuterWallBall(const vd::ExecutiveInit& init, const vd::InitEventList& events)
        : vd::Executive(init, events)
    { }

    vd::Time init(const vd::Time&)
    {
        vp::Conditions& cond_l = conditions(); //référence aux conditions du VPZ
        vp::Condition& cond_Mur = cond_l.get("param");
        cond_Mur.setValueToPort("x1", vv::Double(1));
        cond_Mur.setValueToPort("y1", vv::Double(0));
        cond_Mur.setValueToPort("x2", vv::Double(1));
        cond_Mur.setValueToPort("y2", vv::Double(1));


        createModelFromClass(
            "toto",
            "wall");
        cond_Mur.setValueToPort("x1", vv::Double(-3));
        cond_Mur.setValueToPort("y1", vv::Double(0));
        cond_Mur.setValueToPort("x2", vv::Double(-3));
        cond_Mur.setValueToPort("y2", vv::Double(20));

        createModelFromClass("toto", "walli");


        return vd::infinity;
    }
};

}
}
} // namespace vle example
//DECLARE_NAMED_EXECUTIVE(ExecuterWallBall, mas::test::dynamics::ExecuterWallBall)
DECLARE_EXECUTIVE(mas::test::dynamics::ExecuterWallBall)

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
            bg::model::d2::point_xy<double> xy_ball;//Ball position
            bn::ublas::vector<double> v_ball(2);//Ball direction vector

            xy_ball.x(event.property("x")->toDouble().value());
            xy_ball.y(event.property("y")->toDouble().value());
            v_ball[0] = (event.property("dx")->toDouble().value());
            v_ball[1] = (event.property("dy")->toDouble().value());

            check_collision(xy_ball, v_ball);
        }
    }

    /* CUSTOM FUNCTIONS */
    void check_collision(bg::model::d2::point_xy<double> xy_ball,
                         bn::ublas::vector<double> v_ball)
    {
        bn::ublas::vector<double> wall_vect(2), wall_vect2(2), norm_wall(2);
        bg::model::d2::point_xy<double> xy_collision;
        double A, B, D, H, R, dotprod, dotprod2;
        wall_vect[0] = x1.x() - x2.x();
        wall_vect[1] = x1.y() - x2.y();

        wall_vect2[0] = wall_vect[0];
        wall_vect2[1] = wall_vect[1];

        //Compute normal vector of wall
        norm_wall[0] = wall_vect[1];
        norm_wall[1] = -wall_vect[0];

        //reverse normal vector if it is in wrong direction
        dotprod = norm_wall[0] * (xy_ball.x() - x2.x()) +
                  norm_wall[1] * (xy_ball.y() - x2.y());
        if (dotprod <= 0)
            norm_wall = -1 * norm_wall;

        A = norm_wall[0];
        B = norm_wall[1];

        H = 0;
        D = H - (A * x1.x()) - (B * x1.y());
        H = A * xy_ball.x() + B * xy_ball.y() + D;
        R = H / (norm_wall[0] * v_ball[0] + norm_wall[1] * v_ball[1]);

        xy_collision.x(xy_ball.x() - (v_ball[0] * R));
        xy_collision.y(xy_ball.y() - (v_ball[1] * R));

        // Ball on wall norm dot prod
        dotprod =  v_ball[0] * norm_wall[0] +  v_ball[1] * norm_wall[1];
        // Collision point on wall norm dot prod
        dotprod2 = (x1.x() - xy_collision.x()) * (x2.x() - xy_collision.x()) +
                   (x1.y() - xy_collision.y()) * (x2.y() - xy_collision.y());
        if (dotprod <= 0 && dotprod2 <= 0) {
            double collision_distance, collision_time;
            collision_distance = bg::distance(xy_ball, xy_collision);
            collision_time =  collision_distance / bn::ublas::norm_2(v_ball);

            if (collision_distance > 0) {
                bn::ublas::vector<double> new_vector(2);
                Event new_collision(collision_time);

                new_vector = compute_new_vector(wall_vect, norm_wall, v_ball);
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
                mEventsToSend.push_back(new_collision);
            }
        }
    }

    bn::ublas::vector<double> compute_new_vector(
        const bn::ublas::vector<double>& wall_vect,
        const bn::ublas::vector<double>& norm_wall_vect,
        const bn::ublas::vector<double>& ball_vect)
    {
        bn::ublas::vector<double> p_vc(2), p_vn(2);
        bn::ublas::vector<double> wall_vect_unity(3), norm_vect_unit(3);
        double dotprod;

        /* compute unity vectors */
        wall_vect_unity = wall_vect / bn::ublas::norm_2(wall_vect);
        norm_vect_unit = norm_wall_vect / bn::ublas::norm_2(norm_wall_vect);

        /* compute dotproduct of direction vector/wall vector */
        dotprod = wall_vect_unity[0] * ball_vect[0]
                  + wall_vect_unity[1] * ball_vect[1];

        /* vector projection */
        p_vc[0] = dotprod * wall_vect_unity[0];
        p_vc[1] = dotprod * wall_vect_unity[1];

        /* compute dotproduct of direction vector/normal vector */
        dotprod = norm_vect_unit[0] * ball_vect[0]
                  + norm_vect_unit[1] * ball_vect[1];

        /* vector projection + inversing */
        p_vn[0] = -1 * dotprod * norm_vect_unit[0];
        p_vn[1] = -1 * dotprod * norm_vect_unit[1];

        return p_vc + p_vn;
    }

private:
    bg::model::d2::point_xy<double> x1;
    bg::model::d2::point_xy<double> x2;
};
}
}
} // namespace vle example
DECLARE_DYNAMICS(mas::test::dynamics::SimpleWall)


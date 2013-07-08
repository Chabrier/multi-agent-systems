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

#include <vle/extension/mas/Agent.hpp>
#include <vle/extension/mas/Events.hpp>

using namespace vle::extension::mas;
namespace vd = vle::devs;
namespace vv = vle::value;
namespace bg = boost::geometry;
namespace bn = boost::numeric;

namespace mas { namespace test { namespace dynamics {

class SimpleWall : public Agent
{
public:
    SimpleWall(const vd::DynamicsInit& init, const vd::InitEventList& events)
        : Agent(init, events)
    {
        x1.x( (events.exist("x1"))
            ? events.getDouble("x1") : -1);
            
        x2.x( (events.exist("x2"))
            ? events.getDouble("x2") : -1);
            
        x1.y( (events.exist("y1"))
            ? events.getDouble("y1") : -1);

        x2.y( (events.exist("y2"))
            ? events.getDouble("y2") : -1);
    }

    /* DEVS DYNAMICS FUNCTIONS */
    vd::Time init(const vd::Time& time)
    {
        std::cout << "init wall" << std::endl << std::flush;
        mLastUpdate = time;
        return vd::infinity;
    }

    vd::Time timeAdvance() const
    {
        std::cout << "Time advance wall" << std::endl<< std::flush;
        if (mEventsToSend.size() > 0)
            return 0.0;
        
        return vd::infinity;
    }

    void internalTransition(const vd::Time& /*time*/)
    {
        std::cout << "internalTransition wall" << std::endl<< std::flush;
        mEventsToSend.clear();
    }
    
    void externalTransition(const vd::ExternalEventList& event,
                            const vd::Time& /*time*/)
    {
        std::cout << "External Transition wall" << std::endl<< std::flush;
        for (size_t i = 0; i < event.size(); ++i)
        {
            if(event[i]->getPortName() == "positions")
            {
                bg::model::d2::point_xy<double> xy_ball;//Ball position
                bn::ublas::vector<double> v_ball(2);//Ball direction vector
                for(vv::MapValue::const_iterator it = event[i]->getAttributes().begin();
                  it != event[i]->getAttributes().end(); ++it)
                {
                    if(it->first == "x")
                        xy_ball.x(it->second->toDouble().value());
                    else if (it->first == "y")
                        xy_ball.y(it->second->toDouble().value());
                    else if(it->first == "dx")
                        v_ball[0] = (it->second->toDouble().value());
                    else if (it->first == "dy")
                        v_ball[1] = (it->second->toDouble().value());
                }
                std::cout << "Event listened : "<<xy_ball.x() <<" "<<xy_ball.y()
                    <<" "<<v_ball[0]<<" "<<v_ball[1]<<std::endl;

                check_collision(xy_ball, v_ball);
            }
        }
    }

    /* CUSTOM FUNCTIONS */
    void check_collision(bg::model::d2::point_xy<double> xy_ball,
        bn::ublas::vector<double> v_ball)
    {
        bn::ublas::vector<double> wall_vect(3), wall_vect2(3), norm_wall(3);
        bg::model::d2::point_xy<double> xy_collision;

        double A, B, D, H, R, dotprod;
        wall_vect[0] = x1.x() - x2.x();
        wall_vect[1] = x1.y() - x2.y();
        wall_vect[2] = 0;
        wall_vect2[0] = wall_vect[0];
        wall_vect2[1] = wall_vect[1];
        wall_vect2[2] = 2;

        // norm_wall = wall_vect * wall_vect2 (produit vectoriel))
        norm_wall[0]= wall_vect[1]*wall_vect2[2] - wall_vect[2]*wall_vect2[1];
        norm_wall[1]= wall_vect[2]*wall_vect2[0] - wall_vect[0]*wall_vect2[2];
        norm_wall[2]= wall_vect[0]*wall_vect2[1] - wall_vect[1]*wall_vect2[0];

        A = norm_wall[0];
        B = norm_wall[1];

        H = 0;
        D = H - (A*x1.x()) - (B*x1.y());
        H = A*xy_ball.x() + B*xy_ball.y() + D;
        R = H / (norm_wall[0] * v_ball[0] + norm_wall[1] * v_ball[1]);
        
        xy_collision.x(xy_ball.x() - ( v_ball[0] * R));
        xy_collision.y(xy_ball.y() - ( v_ball[1] * R));

        bn::ublas::vector<double> a(2), b(2);
        a[0] = x1.x() - xy_collision.x();
        a[1] = x1.y() - xy_collision.y();
        b[0] = x2.x() - xy_collision.x();
        b[1] = x2.y() - xy_collision.y();
        
        dotprod =  a[0] * b[0] +  a[1] * b[1];

        if (dotprod <= 0)
        {
            double collision_distance, collision_time;
            collision_distance = bg::distance(xy_ball, xy_collision);
            collision_time =  collision_distance / bn::ublas::norm_2(v_ball);

            if(collision_distance > 0)
            {
                bn::ublas::vector<double> new_vector(2);
                Event new_collision(collision_time);
 
                new_vector = compute_new_vector(wall_vect,norm_wall,v_ball);
                new_collision.add_property("new_dx", new vle::value::Double(new_vector[0]));
                new_collision.add_property("new_dy", new vle::value::Double(new_vector[1]));
                new_collision.add_property("type,", new vle::value::String("collision"));
                mEventsToSend.push_back(new_collision);

                std::cout << "New vector : " << new_vector[0]
                    <<" "<< new_vector[1]<< std::endl;
            }
        }
        else
        {
            std::cout << "Pas de collision!!!!!!" << std::endl;
        }
    }

    bn::ublas::vector<double> compute_new_vector(
        const bn::ublas::vector<double>& wall_vect,
        const bn::ublas::vector<double>& norm_wall_vect,
        const bn::ublas::vector<double>& ball_vect)
    {
        bn::ublas::vector<double> p_vc(2), p_vn(2);
        bn::ublas::vector<double> wall_vect_unity(3),norm_vect_unit(3);
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

}}} // namespace vle example
DECLARE_DYNAMICS(mas::test::dynamics::SimpleWall)

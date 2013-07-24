#include <vle/extension/mas/Utils.hpp>

namespace vle
{
namespace extension
{
namespace mas
{

std::tuple<bool,point> collision_point(point w1,
                                       point w2,
                                       point ball,
                                       vector ball_v)
{
    point collision_point;
    vector wall_v(2), wall_norm(2);
    double dot_prod, det;
    bool exists = true;

    wall_v(0) = w2.x() - w1.x();
    wall_v(1) = w2.y() - w1.y();

    //Compute normal vector of wall
    wall_norm(0) = wall_v(1);
    wall_norm(1) = -wall_v(0);

    //reverse normal vector if it is in wrong direction
    dot_prod = wall_norm(0) * (ball.x() - w1.x()) +
               wall_norm(1) * (ball.y() - w1.y());
    if (dot_prod < 0)
        wall_norm = -1 * wall_norm;

    /* Compute collision point
     * ax + by + c = 0 */
    matrix ab(2,2);
    vector c(2);

    //wall equation
    ab(0,0) = -wall_v(1); //a
    ab(0,1) =  wall_v(0); //b
    c(0) = wall_v(1)*w1.x() - wall_v(0)*w1.y(); //c

    //ball direction equation
    ab(1,0) = -ball_v(1); //a
    ab(1,1) =  ball_v(0); //b
    c(1) = ball_v(1)*ball.x() - ball_v(0)*ball.y(); //c

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
    dot_prod = wall_norm(0) * ball_v(0) + wall_norm(1) * ball_v(1);
    if(dot_prod >= 0)
        exists = false;

    return std::make_tuple(exists,collision_point);
}

vector new_direction(point w1,point w2,point ball,vector ball_v)
{
    vector p_vn(2),p_vc(2),wall_v(2),wall_norm(2);
    double dot_prod;

    wall_v(0) = w2.x() - w1.x();
    wall_v(1) = w2.y() - w1.y();
    wall_v /= bn::ublas::norm_2(wall_v);

    //Compute normal vector of wall
    wall_norm(0) = wall_v(1);
    wall_norm(1) = -wall_v(0);
    wall_norm /= bn::ublas::norm_2(wall_norm);

    //reverse normal vector if it is in wrong direction
    dot_prod = wall_norm(0) * (ball.x() - w1.x()) +
               wall_norm(1) * (ball.y() - w1.y());
    if (dot_prod < 0)
        wall_norm = -1 * wall_norm;

    //ball_v /= bn::ublas::norm_2(ball_v);

    /* compute dotproduct of direction vector/wall vector */
    dot_prod = wall_v[0] * ball_v[0]
              + wall_v[1] * ball_v[1];

    /* vector projection */
    p_vc[0] = dot_prod * wall_v[0];
    p_vc[1] = dot_prod * wall_v[1];

    /* compute dotproduct of direction vector/normal vector */
    dot_prod = wall_norm[0] * ball_v[0]
              + wall_norm[1] * ball_v[1];

    /* vector projection + inversing */
    p_vn[0] = -1 * dot_prod * wall_norm[0];
    p_vn[1] = -1 * dot_prod * wall_norm[1];

    return p_vc + p_vn;
}

bool between(point w1,point w2,point c1)
{
    vector c1_w1(2), c1_w2(2);
    double dot_prod;
    c1_w1(0) = w1.x() - c1.x();
    c1_w1(1) = w1.y() - c1.y();

    c1_w2(0) = w2.x() - c1.x();
    c1_w2(1) = w2.y() - c1.y();

    dot_prod = c1_w1(0)*c1_w2(0) + c1_w1(1)*c1_w2(1);

    return dot_prod <= 0;
}

bool inSegment(point w1,point w2,point c1)
{
    vector w1_c1(2), w1_w2(2);
    w1_c1(0) = c1.x() - w1.x();
    w1_c1(1) = c1.y() - w1.y();

    w1_w2(0) = w2.x() - w1.x();
    w1_w2(1) = w2.y() - w1.y();

    return collinear(w1_c1,w1_w2);
}

bool collinear(vector v1,vector v2)
{
    return v1(0)*v2(1) - v2(0)*v1(1) == 0;
}

}}} // namespace vle extension mas

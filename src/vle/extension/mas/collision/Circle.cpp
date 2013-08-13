#include <vle/extension/mas/collision/Circle.hpp>

double Circle::getRadius() const
{ return mRadius; }

const Point& Circle::getCenter() const
{ return mCenter; }

/* Circle/Segment collisions */
bool Circle::inCollision(Segment segment,const Vector2d& directionVector) const
{
    Vector2d wallDirectionVector, n_wallDirectionVector;
    Vector2d n_directionVector;
    Point intersectionPoint;

    /* Init variables */
    wallDirectionVector.x() = segment.getEnd1().x() - segment.getEnd2().x();
    wallDirectionVector.y() = segment.getEnd1().y() - segment.getEnd2().y();

    n_wallDirectionVector = wallDirectionVector;
    n_wallDirectionVector.normalize();

    n_directionVector = directionVector;
    n_directionVector.normalize();

    /* Extends segment  */
    segment.extends(mRadius);

    /* Check if wall vector and direction vector are not collinear */
    if ((n_wallDirectionVector == n_directionVector) ||
        (n_wallDirectionVector == -1 * n_directionVector)) {
        return false;
    }

    /* Compute intersection point */
    Point directionBis = Point(mCenter.x()+directionVector.x(),
                               mCenter.y()+directionVector.y());
    intersectionPoint = intersection(segment.getEnd1(),segment.getEnd2(),
                                     mCenter,directionBis);
    /* Compute intersection/wall point vectors */
    Vector2d a;
    Vector2d b;
    a.x() = intersectionPoint.x() - segment.getEnd1().x();
    a.y() = intersectionPoint.y() - segment.getEnd1().y();
    b.x() = intersectionPoint.x() - segment.getEnd2().x();
    b.y() = intersectionPoint.y() - segment.getEnd2().y();

    /* Compute dot products */
    double dotProdA = a.dot_prod(wallDirectionVector * -1);
    double dotProdB = b.dot_prod(wallDirectionVector);

    if(!((dotProdA >= 0) && (dotProdB >= 0)))
        return false;

    return true;
}

CollisionPoints Circle::collisionPoints(Segment segment,
                                        const Vector2d& directionVector) const
{
    Point intersectionPoint;
    Point collisionPoint;
    Vector2d n_directionVector;

    /* Extends segment  */
    segment.extends(mRadius);

    /* Compute line intersection point */
    Point directionBis = Point(mCenter.x()+directionVector.x(),
                               mCenter.y()+directionVector.y());
    intersectionPoint = intersection(segment.getEnd1(),segment.getEnd2(),
                                     mCenter,directionBis);
    n_directionVector = directionVector;
    n_directionVector.normalize();

    /* Compute ball collision point (center of the circle)*/
    collisionPoint.x(intersectionPoint.x()
                     + -1*n_directionVector.x()*mRadius);
    collisionPoint.y(intersectionPoint.y()
                     + -1*n_directionVector.y()*mRadius);

    CollisionPoints cp;
    cp.object1CollisionPosition = collisionPoint;
    cp.collisionPoint = intersectionPoint;

    return cp;
}

Vector2d Circle::newDirection(const Segment& segment,
                              const Vector2d& directionVector) const
{
    Vector2d wallDirectionVector, n_wallDirectionVector;
    Vector2d n_wallNormVector;

    wallDirectionVector.x() = segment.getEnd1().x() - segment.getEnd2().x();
    wallDirectionVector.y() = segment.getEnd1().y() - segment.getEnd2().y();
    n_wallDirectionVector = wallDirectionVector;
    n_wallDirectionVector.normalize();

    /* Compute normal vector */
    n_wallNormVector.x() = n_wallDirectionVector.y();
    n_wallNormVector.y() = -n_wallDirectionVector.x();

    /* reverse normal vector if it is in wrong direction */
    double dot_prod;
    Vector2d tmp(mCenter.x() - segment.getEnd1().x(),
                 mCenter.y() - segment.getEnd1().y());
    dot_prod = n_wallNormVector.dot_prod(tmp);

    if (dot_prod < 0)
        n_wallNormVector = -1 * n_wallNormVector;

    /* Compute new direction */
    Vector2d p_vc, p_vn;
    /* compute dotproduct of direction vector/wall vector */
    dot_prod = n_wallDirectionVector.dot_prod(directionVector);

    /* vector projection */
    p_vc = dot_prod * n_wallDirectionVector;

    /* compute dotproduct of direction vector/normal vector */
    dot_prod = n_wallNormVector.dot_prod(directionVector);

    /* vector projection + inversing */
    p_vn = -1 * dot_prod * n_wallNormVector;

    return p_vc + p_vn;
}

Point Circle::intersection(const Point& p1,const Point& p2,
                           const Point& p3,const Point& p4) const
{
    double x, y, div;

    div = (p1.x()-p2.x())*(p3.y()-p4.y()) - (p1.y()-p2.y())*(p3.x()-p4.x());

    if (div == 0)
        throw std::runtime_error("Divide by 0");

    x = (p1.x()*p2.y() - p1.y()*p2.x())*(p3.x() - p4.x()) -
        (p1.x()-p2.x()) * (p3.x()*p4.y()-p3.y()*p4.x());
    x /= div;

    y = (p1.x()*p2.y() - p1.y()*p2.x())*(p3.y()-p4.y()) -
        (p1.y() - p2.y()) * (p3.x()*p4.y()-p3.y()*p4.x());
    y /= div;
    return Point(x,y);
}

/**
 ** a(t) = pa(t) + t*va(t)
 ** b(t) = pb(t) + t*vb(t)
 ** d(t) = abs(a(t) - b(t)) - (a.radius + b.radius)
 ** pab = pa - pb
 ** vab = va - vb
 **
 ** t^2(vab.vab) + 2t(pab.vab) + (pab.pab) - (radius(a) + radius(b))^2
 **/
bool Circle::inCollision(const Vector2d& myVelocity,
                         const Circle& otherCircle,const Vector2d& v2) const
{
    Vector2d vab = myVelocity - v2;
    Vector2d pab;
    pab.x() = mCenter.x() - otherCircle.getCenter().x(),
    pab.y() = mCenter.y() - otherCircle.getCenter().y();

    double a = vab.dot_prod(vab);
    double b = 2 * pab.dot_prod(vab);
    double c = pab.dot_prod(pab) - (mRadius + otherCircle.getRadius())
               * (mRadius + otherCircle.getRadius());

    double discriminant = b * b - 4 * a * c;

    if (a == 0) /* div by 0 */
        return false;

    if (discriminant < 0) {
        /* Collision will never occure
         * Average of roots is time of closest approach, but we don't need it'*/
        return false;
    } else {
        double t0 = (-b + (double)sqrt(discriminant)) / (2 * a);
        double t1 = (-b - (double)sqrt(discriminant)) / (2 * a);
        double t = std::min(t0, t1);

        if (t < 0)/* Collision occured in the past */
            return false;
    }

    return true;
}

CollisionPoints Circle::collisionPoints(const Vector2d& myVelocity,
                                        const Circle& otherCircle,
                                        const Vector2d& v2) const
{
    Vector2d vab = myVelocity - v2;
    Vector2d pab;
    pab.x() = mCenter.x() - otherCircle.getCenter().x(),
    pab.y() = mCenter.y() - otherCircle.getCenter().y();

    double a = vab.dot_prod(vab);
    double b = 2 * pab.dot_prod(vab);
    double c = pab.dot_prod(pab) - (mRadius + otherCircle.getRadius())
               * (mRadius + otherCircle.getRadius());

    double discriminant = b * b - 4 * a * c;
    double t;

    if (discriminant == 0) {
        t = -b/(2*a);
    } else {
        double t0 = (-b + (double)sqrt(discriminant)) / (2 * a);
        double t1 = (-b - (double)sqrt(discriminant)) / (2 * a);
        t = std::min(t0, t1);
    }

    Vector2d collisionA, collisionB, intersectionV;

    collisionA.x() = mCenter.x() + myVelocity.x() * t;
    collisionA.y() = mCenter.y() + myVelocity.y() * t;
    collisionB.x() = otherCircle.getCenter().x() + v2.x() * t;
    collisionB.y() = otherCircle.getCenter().y() + v2.y() * t;

    intersectionV = (collisionA - collisionB) *
              (otherCircle.getRadius()
                                   / (mRadius + otherCircle.getRadius()))
              + collisionB;


    CollisionPoints cp;
    cp.object1CollisionPosition = Point(collisionA.x(),collisionA.y());
    cp.object2CollisionPosition = Point(collisionB.x(),collisionB.y());
    cp.collisionPoint = Point(intersectionV.x(),intersectionV.y());

    return cp;
}

Vector2d Circle::newDirection(const Vector2d& myVelocity,
                              const Circle& otherCircle,
                              const Vector2d& v2) const
{
    Vector2d balltoballVector(otherCircle.getCenter().x() - mCenter.x(),
                              otherCircle.getCenter().y() - mCenter.y());
    Vector2d n_balltoballVector = balltoballVector;
    n_balltoballVector.normalize();

    Vector2d n_balltoballNormVector(n_balltoballVector.y(),
                                    -n_balltoballVector.x());
    if(n_balltoballNormVector.dot_prod(myVelocity) < 0)
        n_balltoballNormVector *= -1;

    Vector2d projection1, projection2;
    projection1 = n_balltoballNormVector.dot_prod(myVelocity) * n_balltoballNormVector;
    projection2 = -1*n_balltoballVector.dot_prod(myVelocity) * n_balltoballVector;

    return projection1 + projection2;
}

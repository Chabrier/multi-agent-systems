#include <vle/extension/mas/collision/Circle.hpp>

double Circle::getRadius() const
{ return mRadius; }

const Point& Circle::getCenter() const
{ return mCenter; }

double& Circle::getRadius()
{ return mRadius; }

Point& Circle::getCenter()
{ return mCenter; }

/* Circle/Segment collisions */
bool Circle::inCollision(Segment segment,const Vector2d& directionVector) const
{
    Vector2d wallDirectionVector, n_wallDirectionVector;
    Vector2d n_directionVector, n_wallNormVector;
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

    /* Compute normal vector */
    n_wallNormVector.x() = n_wallDirectionVector.y();
    n_wallNormVector.y() = -n_wallDirectionVector.x();

    /* reverse normal vector if it is in wrong direction */
    Vector2d tmp(mCenter.x() - segment.getEnd1().x(),
                 mCenter.y() - segment.getEnd1().y());

    if (n_wallNormVector.dot_prod(tmp) < 0)
        n_wallNormVector = -1 * n_wallNormVector;

    /* if ball direction is not to here */
    if(n_wallNormVector.dot_prod(directionVector) >= 0)
        return false;

    /* Compute intersection point */

    Point directionBis = Point(mCenter.x()+n_directionVector.x(),
                               mCenter.y()+n_directionVector.y());
    Point segmouveEnd1 = Point(
        segment.getEnd1().x() + n_wallNormVector.x() * mRadius,
        segment.getEnd1().y() + n_wallNormVector.y() * mRadius);

    Point segmouveEnd2 = Point(
        segment.getEnd2().x() + n_wallNormVector.x() * mRadius,
        segment.getEnd2().y() + n_wallNormVector.y() * mRadius);

    Vector2d movedSegment(segmouveEnd2.x() - segmouveEnd1.x(),
                          segmouveEnd2.y() - segmouveEnd1.y());

    /* Check if wall vector and direction vector are not collinear */
    if (movedSegment.normalize() == n_directionVector ||
        movedSegment.normalize() == -1 * n_directionVector) {
        return false;
    }

    double div = (segmouveEnd1.x() - segmouveEnd2.x())*
        ( mCenter.y() - directionBis.y()) - (segmouveEnd1.y() - segmouveEnd2.y())*( mCenter.x() - directionBis.x());

    if (div == 0) {
        return false;
    }

    intersectionPoint = intersection(segmouveEnd1, segmouveEnd2,
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
       Point collisionPoint;


    Vector2d wallDirectionVector, n_wallDirectionVector;
    Vector2d n_directionVector, n_wallNormVector, n_directionNormVector;

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

    /* Compute normal vector */
    n_wallNormVector.x() = n_wallDirectionVector.y();
    n_wallNormVector.y() = -n_wallDirectionVector.x();

    /* reverse normal vector if it is in wrong direction */
    Vector2d tmp(mCenter.x() - segment.getEnd1().x(),
                 mCenter.y() - segment.getEnd1().y());

    if (n_wallNormVector.dot_prod(tmp) < 0)
        n_wallNormVector = -1 * n_wallNormVector;


    /* Compute intersection point */
    Point directionBis = Point(mCenter.x()+directionVector.x(),
                               mCenter.y()+directionVector.y());
    Point segmouveEnd1 = Point(segment.getEnd1().x() + n_wallNormVector.x() * mRadius,
                               segment.getEnd1().y() + n_wallNormVector.y() * mRadius);
    Point segmouveEnd2 = Point(segment.getEnd2().x() + n_wallNormVector.x() * mRadius,
                               segment.getEnd2().y() + n_wallNormVector.y() * mRadius);



    intersectionPoint = intersection( segmouveEnd1, segmouveEnd2,
                                      mCenter,directionBis);

    /* Compute ball collision point (center of the circle)*/
    collisionPoint.x(intersectionPoint.x());
    collisionPoint.y(intersectionPoint.y());

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
 **
 ** seems to be very clean, and could be used to check if a bird is
 ** leaving the neighborhood
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

    if (a == 0) // div by 0, les vescteurs sont identiques
        return false;

    if (discriminant < 0) {
        /* Collision will never occure
         * Average of roots is time of closest approach, but we don't need it'*/
        return false;
    } else if (discriminant == 0) { // frontière in out du point de
                                    // vue du voisinage
        return false;
    } else {
        double t0 = (-b + (double)sqrt(discriminant)) / (2 * a);
        double t1 = (-b - (double)sqrt(discriminant)) / (2 * a);

        // 2 cas
        // si un oiseau est dans le voisnage de l'autre.

        if (Vector2d(mCenter.x() - otherCircle.getCenter().x(),
                     mCenter.y() - otherCircle.getCenter().y()).norm() <= mRadius) {
            double t = std::max(t0, t1);
        } else {
            double t = std::min(t0, t1);
            if (t < 0)/* Collision occured in the past */
            return false;
        }
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
        if (Vector2d(mCenter.x() - otherCircle.getCenter().x(),
                     mCenter.y() - otherCircle.getCenter().y()).norm() <= mRadius) {
            t = std::max(t0, t1);
        } else {
            t = std::min(t0, t1);
        }
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
                              const Circle& c2,
                              const Vector2d& v2) const
{

    CollisionPoints cp = collisionPoints(myVelocity,c2,v2);
    Point myFutureCenter = cp.object1CollisionPosition;
    Point otherCircleCenter = cp.object2CollisionPosition;

    Vector2d balltoballVector(otherCircleCenter.x() - myFutureCenter.x(),
                              otherCircleCenter.y() - myFutureCenter.y());

    Vector2d n_balltoballVector = balltoballVector;
    n_balltoballVector.normalize();

    Vector2d n_balltoballNormVector(n_balltoballVector.y(),
                                    -n_balltoballVector.x());

    Vector2d projection1, projection2;
    if(n_balltoballNormVector.dot_prod(myVelocity) < 0) {
        n_balltoballNormVector *= -1;}

    if(n_balltoballVector.dot_prod(myVelocity) < 0) {
        n_balltoballVector *= -1;
        projection2 = n_balltoballVector.dot_prod(myVelocity)
            * n_balltoballVector;
    } else {
        projection2 = -1* n_balltoballVector.dot_prod(myVelocity)
            * n_balltoballVector;
    }
    projection1 = n_balltoballNormVector.dot_prod(myVelocity)
        * n_balltoballNormVector;

    Vector2d newProjection = projection1 + projection2;
    Vector2d newProjectionN = newProjection;

    newProjectionN.normalize();

    if (determinant(myVelocity, balltoballVector) == 0 &&
        determinant(balltoballVector, v2) == 0 &&
        determinant(v2, newProjection) == 0) {
        Vector2d v2n = v2;

        Vector2d myVelocityn = myVelocity;

        if (myVelocityn.normalize() != -1 * v2n.normalize()) {
            if (balltoballVector.normalize() == v2n.normalize()) {
                if (myVelocityn.normalize() != -1 * newProjectionN) {
                    newProjection *= -1;
                }
            }
        }
    }

    return newProjection;
}

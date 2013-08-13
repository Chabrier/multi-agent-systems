#ifndef CIRCLE_HPP
#define CIRCLE_HPP

#include <vle/extension/mas/collision/Types.hpp>
#include <vle/extension/mas/collision/Segment.hpp>
class Circle
{
public:
    Circle(const Point& center,double radius)
    :mCenter(center),mRadius(radius)
    {}

    double getRadius() const;
    const Point& getCenter() const;

    /* Circle/Segment collisions */
    bool inCollision(Segment,const Vector2d&) const;
    CollisionPoints collisionPoints(Segment,const Vector2d&) const;
    Vector2d newDirection(const Segment&,const Vector2d&) const;

    /* Circle/circle collisions */
    bool inCollision(const Vector2d&,const Circle&,const Vector2d&) const;
    CollisionPoints collisionPoints(const Vector2d&,
                                    const Circle&,const Vector2d&) const;
    Vector2d newDirection(const Vector2d&,const Circle&,const Vector2d&) const;
private:
    Point intersection(const Point&,const Point&,
                       const Point&,const Point&) const;

protected:
    Point  mCenter;
    double mRadius;
};

#endif

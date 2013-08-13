#include <vle/extension/mas/collision/Segment.hpp>

void Segment::extends(end e,double d)
{
    Vector2d wallDirectionVector;
    Point *end;
    /* Init variables */
    wallDirectionVector.x() = mEnd1.x() - mEnd2.x();
    wallDirectionVector.y() = mEnd1.y() - mEnd2.y();
    Vector2d n_wallDirectionVector = wallDirectionVector;
    n_wallDirectionVector.normalize();

    switch (e)
    {
        case END1:
            end = &mEnd1;
            break;
        case END2:
            end = &mEnd2;
            n_wallDirectionVector *= -1;
            break;
    }

    end->x(end->x() + n_wallDirectionVector.x() * d);
    end->y(end->y() + n_wallDirectionVector.y() * d);
}

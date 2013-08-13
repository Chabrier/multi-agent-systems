#ifndef SEGMENT_HPP
#define SEGMENT_HPP

#include <vle/extension/mas/collision/Types.hpp>

class Segment
{
public:
    Segment(const Point& p1,const Point& p2)
    :mEnd1(p1),mEnd2(p2)
    {}

    const Point& getEnd1() const
    { return mEnd1; }

    const Point& getEnd2() const
    { return mEnd2; }

    void setEnd1(const Point& p)
    { mEnd1 = p; }

    void setEnd2(const Point& p)
    { mEnd2 = p; }

    typedef enum { END1, END2 } end;

    void extends(end,double);
    void extends(double d)
    {
        extends(END1,d);
        extends(END2,d);
    }

private:
    Point mEnd1;
    Point mEnd2;
};

#endif

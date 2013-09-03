#include <vle/extension/mas/collision/Vector2d.hpp>
#include <cmath>

Vector2d& Vector2d::rotate(double angle)
{
    double sinus = std::sin(angle);
    double cosinus = std::cos(angle);

    mX = cosinus * mX - sinus * mY;
    mY = sinus * mX + cosinus * mY;

    return *this;
}

Vector2d& Vector2d::normalize()
{
    double norm2 = std::sqrt(mX*mX+mY*mY);
    mX /= norm2;
    mY /= norm2;
    return *this;
}

double Vector2d::norm() const
{ return std::sqrt(mX*mX+mY*mY); }

double Vector2d::dot_prod(const Vector2d& v) const
{ return mX* v.mX + mY*v.mY; }


Vector2d& Vector2d::operator+=(const Vector2d &v)
{
    mX += v.mX;
    mY += v.mY;
    return *this;
}

Vector2d& Vector2d::operator-=(const Vector2d &v)
{
    mX -= v.mX;
    mY -= v.mY;
    return *this;
}

Vector2d& Vector2d::operator*=(const double &d)
{
    mX *= d;
    mY *= d;
    return *this;
}

Vector2d& Vector2d::operator/=(const double &d)
{
    mX /= d;
    mY /= d;
    return *this;
}

Vector2d Vector2d::operator-()
{ return Vector2d(-1 * mX,-1 *mY); }

/* Friend functions */

Vector2d operator+(const Vector2d &v1, const Vector2d &v2)
{ return Vector2d(v1.mX+v2.mX,v1.mY+v2.mY); }

Vector2d operator-(const Vector2d &v1, const Vector2d &v2)
{ return Vector2d(v1.mX-v2.mX,v1.mY-v2.mY); }

Vector2d operator/(const Vector2d &v1, const double &d)
{ return Vector2d(v1.mX/d,v1.mY/d); }

Vector2d operator*(const Vector2d &v1, const double &d)
{ return Vector2d(v1.mX*d,v1.mY*d); }

Vector2d operator*(const double &d,const Vector2d &v1)
{ return v1 * d; }

bool operator==(const Vector2d &v1, const Vector2d &v2)
{ return (v1.mX == v2.mX) && (v1.mY == v2.mY); }

bool operator!=(const Vector2d &v1, const Vector2d &v2)
{  return (v1.mX != v2.mX) || (v1.mY != v2.mY); }

double angle(const Vector2d &v1, const Vector2d &v2)
{
    double normv1 = std::sqrt(v1.mX * v1.mX + v1.mY * v1.mY);
    double normv2 = std::sqrt(v2.mX * v2.mX + v2.mY * v2.mY);
    double C = (v1.mX * v2.mX +  v1.mY * v2.mY)/(normv1 * normv2);
    double S = v1.mX * v2.mY - v1.mY * v2.mX;
    double sS = S/fabs(S);
    return( sS * std::acos(C));
}

double determinant(const Vector2d &v1, const Vector2d &v2)
{
    return (v1.mX * v2.mY - v1.mY * v2.mX);
}

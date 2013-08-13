#ifndef VECTOR2D_HPP
#define VECTOR2D_HPP

class Vector2d
{
public:
    Vector2d(double x,double y):mX(x),mY(y)
    {}
    Vector2d() {}

    /* Getter/Setter */
    double x() const {return mX;}
    double y() const {return mY;}
    double& x() {return mX; }
    double& y() {return mY;}

    /* member functions */
    double dot_prod(const Vector2d& v) const;
    double norm() const;
    Vector2d& normalize();
    Vector2d& rotate(double angle);

    /* Operator redefinitions */
    friend Vector2d operator+(const Vector2d &v1, const Vector2d &v2);
    friend Vector2d operator-(const Vector2d &v1, const Vector2d &v2);
    friend Vector2d operator/(const Vector2d &v1, const double &d);
    friend Vector2d operator*(const Vector2d &v1, const double &d);
    friend Vector2d operator*(const double &d,const Vector2d &v1);
    friend bool operator==(const Vector2d &v1, const Vector2d &v2);
    friend bool operator!=(const Vector2d &v1, const Vector2d &v2);
    Vector2d& operator+=(const Vector2d &v);
    Vector2d& operator-=(const Vector2d &v);
    Vector2d& operator*=(const double &d);
    Vector2d& operator/=(const double &d);
    Vector2d operator-();
protected:
    double mX;
    double mY;
};


#endif

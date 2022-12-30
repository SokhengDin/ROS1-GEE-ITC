#include "Circle1.h"

Circle::Circle(double rds)
:radius(rds)
{
    if (radius < 0.0)
    {
        assert(false);
    }
}
Circle::Circle()
:radius(1)
{
}

Circle::Circle(const Circle& circle)
:radius(circle.radius)
{
}

Circle::~Circle()
{
}
void Circle::setRadius(double value)
{
    radius = value;
    if (radius < 0.0)
    {
        assert(false);
    }
}
double Circle::getRadius()const
{
    return radius;
}
double Circle::getArea()const
{
    const double PI = 3.14;
    return (2*PI*radius);
}
double Circle::getPerimter()const
{
    const double PI = 3.14;
    return (2*PI*radius);
}


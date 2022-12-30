#ifndef CIRCLE_H
#define CIRCLE_H
#include <iostream>
#include <cassert>
using namespace std;

// Class defintion
class Circle
{
    private:
            double radius;
    public:
            Circle(double radius);
            Circle();
            Circle(const Circle& cricle);
            ~Circle();
            void setRadius(double radius);
            double getRadius()const;
            double getArea()const;
            double getPerimter()const;
};
#endif
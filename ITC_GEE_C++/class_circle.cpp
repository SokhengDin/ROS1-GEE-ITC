#include <iostream>

using namespace std;

class Circle
{
    private: // Artibute
        double radius; // Data Member
    public:
        double getRadius()const; // Member function
        double getArea()const; // Member function
        double getPerimeter()const; // Member function 
        void setRadius(double radius); // Member function
};

// Definition of functiion getRadius
double Circle::getRadius()const
{
    return radius;
}
// Definition of function getArea
double Circle::getArea()const
{
    const double PI = 3.14;
    double area;
    area = PI*radius*radius;
    return area;
}
// Definition of function getPerimeter
double Circle::getPerimeter()const
{
    const double PI=3.14;
    double perimeter;
    perimeter = 2*PI*radius;
    return perimeter;
}
// Definition of function setRadius
void Circle::setRadius(double r)
{
    radius = r;
}
int main()
{
    double area_circle1;
    double perimeter_circle1;
    double area_circle2;
    double perimeter_circle2;
    Circle circle1;
    circle1.setRadius(5.0); // We set radius of circle1= 5.0cm
    area_circle1 = circle1.getArea();
    perimeter_circle1 = circle1.getPerimeter();
    cout << "The area of circle1: " << area_circle1 << endl;
    cout << "The perimeter of circle1: " << perimeter_circle1 << endl;
    Circle circle2;
    circle2.setRadius(20.0); // We set radius of circle2 = 20.0cm
    area_circle2 = circle2.getArea();
    perimeter_circle2 = circle2.getPerimeter();
    cout << "The area of circle2: " << area_circle2 << endl;
    cout << "The perimeter of cirlce2: " << perimeter_circle2 << endl;
    return 0;
}
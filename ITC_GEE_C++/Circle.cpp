/***********************************************************************************
 * A program to use a class in object-oriented programming 
***********************************************************************************/

#include <iostream>
using namespace std;

class Circle
{   /*** Data member ***/
    private:
            double radius;
    /*** Member function ***/
    public:
            double getRadius()const; // Can't modify
            double getArea()const;  // Can't modify
            double getPerimeter()const; // Can't modify
            void setRadius(double value); // Can modify
};

// Defintion of getRadius member function
double Circle::getRadius()const
{
    return radius;
}

// Definition of getArea() member funciton
double Circle::getArea()const
{
    const double PI = 3.14;
    return (PI * radius * radius);
}
// Definiton of getPerimeter memeber function
double Circle::getPerimeter()const
{
    const double PI = 3.14;
    return (2 * PI * radius);
}
// Definition of setRadius memeber function
void Circle::setRadius(double value)
{
    radius = value;
}

// Application //

int main()
{
    // Creating object name circle1
    cout << "Circle 1:" << endl;
    Circle circle1;
    circle1.setRadius(9.0);
    cout << "Radius:" << circle1.getRadius() << endl;
    cout << "Area:" << circle1.getArea() << endl;
    cout << "Perimeter:" << circle1.getPerimeter() << endl;
    // Creating object name circle2
    cout << "Circle 2:" << endl;
    Circle circle2;
    circle2.setRadius(27.0);
    cout << "Radius:" << circle2.getRadius() << endl;
    cout << "Area:" << circle2.getArea() << endl;
    cout << "Perimeter:" << circle2.getPerimeter() << endl;
    return 0;
}
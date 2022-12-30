#include <iostream>
using namespace std;


class Circle
{
    private:
            double radius;
    public:
            Circle(double radius);  // Parameter constructor
            Circle();              // Default constructor // 0,0
            Circle(const Circle& circle);      // Copy Constructor
            ~Circle();         // Destructor
            void setRadius(double radius); // Mutator=Modify
            double getRadius()const; // Accessor
            double getArea()const; // Accessor
            double getPerimeter()const; // Accessor
};

// Definition of parameter constructor
Circle::Circle(double r)
:radius(r)
{
    cout << "The parameter constructor was called." << endl;
}
// Defintiion of default parameter
Circle::Circle()
:radius(5.0)
{
    cout << "The default constructor was called." << endl;
}
// Defintion of copy constructor
Circle::Circle(const Circle& circle)
:radius(circle.radius)
{
    cout << "The copy constructor was called." << endl;
}
// Definiton of default constructor
Circle::~Circle()
{
    cout << "The destructor was called.";
    cout << endl;
}

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

// Application
int main()
{
    // Instantiation of circle1 and do the operation
    Circle circle1(10.0);
    cout << "Radius:" << circle1.getRadius() << endl;
    cout << "Area:" << circle1.getArea() << endl;
    cout << "Perimeter:" << circle1.getPerimeter() << endl;
    // Instantiation of circle2 and do the operation
    Circle circle2(circle1);
    cout << "Radius:" << circle2.getRadius() << endl;
    cout << "Area:" << circle2.getArea() << endl;
    cout << "Perimeter:" << circle2.getPerimeter() << endl;
    // Instantiation of circle3 and do the operation
    Circle circle3;
    cout << "Radius:" << circle3.getRadius() << endl;
    cout << "Area:" << circle3.getArea() << endl;
    cout << "Perimeter:" << circle3.getPerimeter() << endl << endl;
    // Calls to destructors occur here
    return 0;
}
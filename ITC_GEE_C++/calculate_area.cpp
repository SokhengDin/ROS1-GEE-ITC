#include <iostream>
#include <cmath>
using namespace std;

const double PI = 3.1456; //Global variable 

double area_triangle(double length, double height); // Prototype function
double area_circle(double radius); // Prototype function
double area_square(double length); // Prototype function
double area_rectangle(double length, double width); // Prototype function


int main()
{
    double length_1;
    double height_1;
    double area_1;
    cout << "Enter the length of triangle: ";
    cin >> length_1;
    cout << "Enter the height of triangle:  ";
    cin >> height_1;
    area_1 = area_triangle(length_1, height_1);
    cout << "The area of triangle is: " << area_1 << endl;
    double radius;
    double area_2;
    cout << "Enter the radius of the circle" ;
    cin >> radius;
    area_2 = area_circle(radius);
    cout << "The area of circle is: " << area_2 << endl;
    return 0;
}

double area_triangle(double length, double height)
{
    double area; // Local variable
    area = (0.5)*(length)*(height);
    return area;
}

double area_circle(double radius)
{
    double area; //Local variable
    // area = 2*PI*(radius**2);
    area = 2*PI*pow(radius, 2);
    return area;
}
double area_square(double length)
{
    double area;
    // area = length**2
    area = pow(length, 2);
    return area;
}

double area_rectangle(double length, double width)
{
    double area;
    area = length * width;
    return area;
}
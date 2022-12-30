#include <iostream>
using namespace std;

class Triangle
{
    private:
        double length; // Data member
        double height; // Data member
    public: 
        double calculateArea()const; // Member function
        void setValueDimension(double length, double height); // Member function
};   

double Triangle::calculateArea()const
{
    double area;
    area = (0.5)*length*height;
    return area;
}

void Triangle::setValueDimension(double len, double hei)
{
    length = len;
    height = hei;
}

int main()
{   
    double area_triangle1;
    double area_triangle2;
    cout << "Triangle1" << endl;
    Triangle triangle1;
    triangle1.setValueDimension(4, 10);
    area_triangle1 = triangle1.calculateArea();
    cout << "The area of triangle1: " << area_triangle1 << endl;
    Triangle triangle2;
    triangle2.setValueDimension(10, 20);
    area_triangle2 = triangle2.calculateArea();
    cout << "The area of triangle2: " << area_triangle2 << endl;
    return 0;

}
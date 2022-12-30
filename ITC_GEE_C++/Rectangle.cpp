/***************************************************************
 * A program to declare, define, and use a Rectangle class
 * ***************************************************************/
#include <iostream>
#include <cassert>
using namespace std;

// Class definition
class Rectangle
{
    private:
            double length; // Data member
            double height; // Data member
    public:
            Rectangle(double length, double height); // Parameter Constructor
            Rectangle(const Rectangle& rect); // Copy constructor
            ~Rectangle(); // Destructor
            void print()const; // Accessor member
            double getArea()const; // Access memeber
            double getPerimeter()const; // Access memebr
};

// Parameter constructor
Rectangle::Rectangle(double len, double hgt)
:length(len), height(hgt)
{
    if ((length <= 0.0) || (height <= 0.0))
    {
        cout << "No rectangle can be made!" << endl;
        assert(false);
    }
}
// Copy constructor
Rectangle::Rectangle(const Rectangle& rect)
:length(rect.length), height(rect.height)
{
}
// Destructor 
Rectangle::~Rectangle()
{
}

// Access memeber function: print length and height
void Rectangle::print()const
{
    cout << "A rectangle of " << length << " by " << height << endl;
}
// Access memeber fucntion: Get area
double Rectangle::getArea()const
{
    return (length * height);
}
// Access memeber function: Get perimeter
double Rectangle::getPerimeter()const
{
    return (2 * (length+height));
}

// Application
int main()
{
    // Instatiation of three objects
    Rectangle rect1(3.0, 4.2);
    Rectangle rect2(5.1, 10.2);
    Rectangle rect3(rect2);
    // Operation on frist rectangle
    cout << "Rectangle 1 :";
    rect1.print();
    cout << "Area: " << rect1.getArea() << endl;
    cout << "Perimeter: " << rect2.getPerimeter() << endl;
    // Operation on second rectangle
    cout << "Rectangle 2 :";
    rect2.print(); 
    cout << "Area: " << rect2.getArea() << endl;
    cout << "Perimeter: " << rect2.getPerimeter() << endl;
    // Operation on third rectangle
    cout << "Rectangle 3 : ";
    rect3.print(); 
    cout << "Area: " << rect3.getArea() << endl;
    cout << "Perimeter: " << rect3.getPerimeter() << endl;
    return 0;
}
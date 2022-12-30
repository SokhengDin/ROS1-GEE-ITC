/***********************************************************
 * A Program to create objects and count them.
 **********************************************************/
#include <iostream>
using namespace std;
// Defintion of rectangle class
class Rectangle
{
    private:
            double length; // Data member
            double height; // Data member
            static int count; // Static data member
    public:
            Rectangle(double length, double height); // Parameter constructor
            Rectangle(); // Default constructor
            ~Rectangle(); // Destructor for cleaning memory
            Rectangle(const Rectangle& rect); // Copy constructor
            static int getCount(); // Static member function
};
// Initialization of static data member
int Rectangle::count = 0;

// Definition of all constructor
Rectangle::Rectangle(double len, double hgt) // Parameter constructor
:length(len), height(hgt)
{
    count ++;
    cout << "The parameter constructor was called" << endl;
}

Rectangle::Rectangle() // Default constructor
:length(0.0), height(0.0)
{
    count ++;
    cout << "The default constructor was called" << endl;
}

Rectangle::Rectangle(const Rectangle& rect) // Copy constructor
:length(rect.length), height(rect.height)
{
    count ++;
    cout << "The copy constructor was called" << endl;
}

Rectangle::~Rectangle()
{
    count --;
    cout << "The destructor was called." <<  endl;
}

// Definition of static int member function
int Rectangle::getCount()
{
    return count;
}

// Applications
int main()
{
    Rectangle rect1(3.2, 1.2);
    Rectangle rect2(1.5, 2.1);
    Rectangle rect3;
    Rectangle rect4(rect1);
    Rectangle rect5(rect2);
    cout << "Count objects:" << rect5.getCount() << endl;
    cout << "Count objects:" << Rectangle::getCount() << endl;
    return 0;
}
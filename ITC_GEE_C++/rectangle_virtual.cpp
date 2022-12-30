/***************************************************************
 * The implementation file the Rectangle class
 * ***************************************************************/
#include "rectangle_virtual.h"

// Constructor
Rectangle::Rectangle(double lg, double wd)
:length(lg), width(wd)
{
    if (!isValid())
    {
        cout << "Invalid rectangle!";
        assert(false);
    }
}
// Destructor
Rectangle::~Rectangle()
{
}
// Definition of print function
void Rectangle::print()const
{
    cout << "Rectangle of " << length << "X" << width << endl;
}
// Finding the area
double Rectangle::getArea()
{
    return length * width;
}
// Finding the perimeter
double Rectangle::getPerimeter()
{
    return 2*(length+width);
}
// Private isValid function
bool Rectangle::isValid()const
{
    return (length > 0.0 && width > 0.0);
}
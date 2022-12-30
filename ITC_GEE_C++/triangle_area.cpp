#include <iostream>
using namespace std;

int main()
{
    // Declaration variables
    double height;
    double length;
    double area;

    // Inputting value
    cout << "Enter the value of height of triangle: ";
    cin >> height;
    cout << "Enter the value of length of triangle: ";
    cin >> length;
    // If condition
    if (height < 0)
    {
        cout << "Height of this triangle does not exists!";
    }
    if (length < 0)
    {
        cout << "Length of this triangle does not exists!";
    }

    // Calculating area.
    area = 0.5*(height)*(length);
    cout << "The area of the triangle is :" << area;

    return 0;
}
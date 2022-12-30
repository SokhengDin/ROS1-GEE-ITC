#include <iostream>

using namespace std;

int main()
{
    // Declaration
    int number;
    // Getting input
    cout << "Enter an integer: ";
    cin >> number;
    // Find the absolute value
    if (number < 0)
    {
        number = -number;
    }
    // Printting the value
    cout << "ABsolute value of the number you entered is: " << number;
    return 0;
}
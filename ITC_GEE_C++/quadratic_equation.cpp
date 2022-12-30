#include <iostream>
#include <cmath>

using namespace std;

int main()
{
    // Declaration of variables ax^2+bx+c=0
    int a, b, c;
    double delta;
    // Inputting the value of three coefficients
    cout << "Enter the value of coefficient a: ";
    cin >> a;
    cout << "Enter the value of coefficient b: ";
    cin >> b;
    cout << "Enter the value of coefficient c: ";
    cin >> c;
    // Calculating the value of term (b^2-4ac)
    delta = pow(b,2) - 4*a*c;
    if (delta < 0)
    {
        cout << "There is no root !" << endl;
    }
    else if (delta == 0)
    {
        cout << "The two roots are equal."  << endl;
        cout << "x1 = x2 =" << -b/(2*a) << endl; 
    }
    else
    {
        cout << "There are two distinct roots: " << endl;
        cout << "x1 = " << (-b+sqrt(delta))/(2*a) << endl;
        cout << "x2 = " << (-b-sqrt(delta))/(2*a) << endl;
    }
    return 0;
}
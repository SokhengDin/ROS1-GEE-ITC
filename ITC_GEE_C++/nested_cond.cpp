#include <iostream>

using namespace std;

int main()
{
    // Declaration
    int num1;
    int num2;
    // Get input values
    cout << "Enter the value of number1: ";
    cin >> num1;
    cout << "Enter the value of number2: ";
    cin >> num2;
    // Decision using nested-if-else
    if (num1 >= num2)
    {
        if (num1 > num2)
        {
            cout << "The value of number 1" << " > " << "number2";
        }
        else
        {
            cout << "The value of number 1" << " = " << "number2";
        }
    }
    else
    {
        cout << "The value of number 1 " << " < " << "number2 ";
    }


    return 0;
}
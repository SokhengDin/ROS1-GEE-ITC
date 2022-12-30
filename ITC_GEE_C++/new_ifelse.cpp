#include <iostream>

using namespace std;

int main()
{
    // Declaration
    int num1;
    int num2;
    int larger;
    // Input
    cout << "Enter the value of number1: ";
    cin >> num1;
    cout << "Enter the value of number2: ";
    cin >> num2;

    // Decision making
    larger = (num1 >= num2)? num1: num2;
    // Output
    cout << "The larger is: " << larger << endl;
    return 0;
}
#include <iostream>

using namespace std;

int main()
{
    // Declarations of variables
    int age;
    bool eligible;

    // Getting input
    cout << "Enter your age: ";
    cin >> age;
    // Setting the condition
    eligible = (age >= 25) && (age < 100);
    // Testing the condtion and output
    if (eligible)
    {
        cout << "You are good to rent the car! ";
    }
    else
    {
        cout << "Sorry! you can't rent a car! ";
    }
    return 0;
}
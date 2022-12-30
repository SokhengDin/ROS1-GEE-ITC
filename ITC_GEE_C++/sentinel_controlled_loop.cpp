#include <iostream>

using namespace std;

int main()
{
    // Declaration
    int sum = 0;
    int num;
    // Loop including the first input
    cout << "Enter the integer (-1 to stop while loop): ";
    cin >> num;
    while (num != -1)
    {
        sum = sum + num;
        cout << "Enter an integer (-1 to stop): ";
        cin >> num;
    }
    // Ouputting result
    cout << "The sum is: " << sum;
    return 0;
}
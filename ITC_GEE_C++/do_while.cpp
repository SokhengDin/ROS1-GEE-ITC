#include <iostream>

using namespace std;

int main()
{
    // Declaration 
    int num;
    short leftDigit;
    // Input
    cout << "Enter a non-negative integer: ";
    cin >> num;
    // Loop
    do
    {
        leftDigit = num % 10;
        num = num / 10;
    }while(num > 0);
    // Output
    cout << "The leftmost digit is: " << leftDigit << endl;
    return 0;
}
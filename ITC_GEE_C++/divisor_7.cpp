#include <iostream>
#include <iomanip>
using namespace std;

int main()
{
    // Declaration and initialization
    int lower = 1;
    int higher = 300;
    int divisor = 7;
    // For loop
    for (int i = lower; i < higher; i++)
    {
        if (i % divisor == 0)
        // Even number
        // if (i % 2 == 0)
        // Odd number
        // else { cout << i ;}
        {
            cout << setw(4) << i << endl;
        }
    }
    return 0;
}
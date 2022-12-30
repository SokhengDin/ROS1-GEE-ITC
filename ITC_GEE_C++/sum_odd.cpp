#include <iostream>
using namespace std;


int sum = 0;
int num_odd;

int main()
{   
    for (num_odd = 100; num_odd < 200; num_odd++)
    {
        if (num_odd % 2 !=0)
        {
            sum = sum + num_odd;
        }
    }
    cout << "The sum of the odd number in that bigger than 100 and smaller than 200 is :" << sum << endl;
    return 0;
}
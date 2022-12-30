#include <iostream>

using namespace std;


// The function compare two number
int larger(int a, int b)
{
    int max;
    if (a > b)
    {
        cout << "The larger value is value a: ";
        max = a;
    }
    else if( b > a)
    {   
        cout << "The larger value is value b: ";
        max = b;
    }
    return max;
}

int main()
{   
    int num1;
    int num2;
    cout << "Enter the value of number1: ";
    cin >> num1;
    cout << "Enter the value of number2: "; 
    cin >> num2;
    int compare = larger(num1, num2); // compare = max
    cout << compare << endl;
    return 0;
}
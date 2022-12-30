#include <iostream>

using namespace std;

void pattern(int size) // Take parameter
{
    for (int i = 0; i < size; i++)
    {
        for (int j = 0; j < size; j++)
        {
            cout << "*" ;
        }
        cout << endl;
    }
    return;
}

int main()
{
    // Declaration
    int patternSize; // Paramter to input inside the function
    cout << "Enter the value of pattern size: ";
    cin >> patternSize;
    pattern(patternSize);
    return 0;
}
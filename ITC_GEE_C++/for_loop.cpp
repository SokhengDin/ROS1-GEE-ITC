#include <iostream>

using namespace std;

int main()
{
    // Declaration
    int n;
    // Get input variables
    cout << "Enter the number of the variable: " ;
    cin >> n;
    // For loop 
    for (int counter = 0; counter < n; counter+=5)
    {
        // cout << counter << " ";
        cout << "Hello world!" << endl;
        
    }
    return 0;
}
#include <iostream>

using namespace std;

int main()
{
    // Declaration
    int rows;   // Number of rows
    int cols;   // Number of coloumns
    // Inputs
    cout << "Enter the rows: ";
    cin >> rows ;
    cout << "Enter the columns: ";
    cin >> cols;
    // Nested-loop
    for (int i = 1; i <= rows; i++)
    {
        for (int j = i; j <= i+cols-1; j++)
        {
            cout << j << " ";
        }// End inner loop
        cout << endl;
    }
    return 0;

}
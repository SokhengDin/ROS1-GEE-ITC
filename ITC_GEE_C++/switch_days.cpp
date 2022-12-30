#include <iostream>

using namespace std;

int main()
{
    // Declaration
    int day;
    // Input 
    cout << "Enter the a number between 0 and 6: ";
    cin >> day;
    // Switch statement (decision and output)
    switch(day)
    {
        case 0: cout << "Monday" << endl;
                break;
        case 1: cout << "Tuesday" << endl;
                break;
        case 2: cout << "Wednesday" << endl;
                break;
        case 3: cout << "Thurdays" << endl;
                break;
        case 4: cout << "Friday" << endl;
                break;
        case 5: cout << "Saturday" << endl;
                break;
        case 6: cout << "Sunday" << endl; 
                break;
    }

    return 0;
}
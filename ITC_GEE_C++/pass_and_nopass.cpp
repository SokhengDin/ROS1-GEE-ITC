#include <iostream>
using namespace std;


int main()
{
    // Declaration
    int score;

    // Input
    cout << "Enter a score between 0 and 100: ";
    cin >> score;
    // Decision
    if (score >= 50)
    {
        cout << "Grade is pass" << endl;
    }
    else{
        cout << "Grade is nopass" << endl;
    }

    return 0;
}
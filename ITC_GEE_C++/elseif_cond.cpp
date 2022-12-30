#include <iostream>

using namespace std;

int main()
{
    // Declaration
    int score;
    char grade;
    // Get input
    cout << "Enter the score of the student: ";
    cin >> score;

    // Multi-way decision
    if (score >= 90)
    {
        grade = 'A';
    }
    else if (score >= 80)
    {
        grade = 'B';
    }
    else if (score >= 70)
    {
        grade = 'C';
    }
    else if (score >= 60)
    {
        grade = 'D';
    }
    else if (score >= 50)
    {
        grade = 'E';
    }
    else
    {
        grade = 'F';
    }

    cout << "The grade of the student is :" << grade << endl;
    return 0;
}
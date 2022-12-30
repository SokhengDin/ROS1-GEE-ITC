#include <iostream>
#include <iomanip>
using namespace std;

int main()
{
    // Declaration
    int score;
    int sum = 0;
    double average;
    // int average
    // Loop
    int counter = 0;
    while(counter < 4)
    {
        cout << "Enter the next score between 0 and 100: " << endl;
        cin >> score;
        sum += score;
        // sum = sum + score;
        counter++;
        // counter = counter + 1
    }

    // Result
    average = static_cast<double>(sum)/4;
    // average = sum / 4
    cout << fixed << setprecision(4) << showpoint; 
    cout << "The average of scores is: " << average << endl;
    return 0;
}
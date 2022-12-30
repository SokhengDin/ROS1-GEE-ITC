#include <iostream>
using namespace std;

int main()
{
    // Variable Declaration
    unsigned long duration, hours, minutes, seconds;
    // Prompt and input
    cout << "Enter a positive integer for the number of seconds: ";
    cin >> duration;
    // Processing
    hours = duration / 3600L;
    minutes = (duration -(hours * 3600L)) / 60L;
    seconds = duration - (hours * 3600L) - (minutes*60L);
    // Output
    cout << "Given Duration in seconds: " << duration << endl;
    cout << "Result: ";
    cout << hours << "hours, ";
    cout << minutes << "minutes, and";
    cout << seconds << "seconds.";
    return 0;
}
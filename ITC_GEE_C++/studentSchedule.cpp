/***************************************************************
 * The implementation file for the class StudentSchedule
 * ***************************************************************/
#include "studentSchedule.h"


// Constructor
StudentSchedule::StudentSchedule()
:size(0)
{
    courseNames = new string[5];
}
// Destructor
StudentSchedule::~StudentSchedule()
{
    delete[] courseNames;
}
// Definitions of addCourse function
void StudentSchedule::addCourse(string name)
{
    courseNames[size] = name;
    size++;
}
// Definition of print function
void StudentSchedule::print()const
{
    cout << "List of Courses" <<  endl;
    for (int i = 0; i < size; i++)
    {
        cout << courseNames[i] << endl;
    }
    cout << endl;
}
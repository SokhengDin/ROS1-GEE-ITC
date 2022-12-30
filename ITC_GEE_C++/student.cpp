/********************************************************************
 * The implementation file for the class Studnet
 * *****************************************************************/

#include "student.h"
// Constructor
Student::Student(string nm)
:name(nm)
{
    schedule = new StudentSchedule;
}
// Destructor
Student::~Student()
{
}
// Defintion of getName function
string Student::getName()const
{
    return name;
}
// Definition of getSchedule function
StudentSchedule* Student::getSchedule()const
{
    return schedule;
}
// Definition of addCourse function
void Student::addCourse(string name)
{
    schedule -> addCourse(name);
}
// Definition of print function
void Student::print()const
{
    cout << "Student name:" << name << endl;
    schedule -> print();
}
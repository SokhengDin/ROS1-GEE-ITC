/***************************************************************
 * The implementation file for class CourseRoster
 * ***************************************************************/

#include "courseRoster.h"
#pragma once
// Constructor
CourseRoster::CourseRoster()
:size(0)
{
    stdNames = new string[20];
}
// Destructor
CourseRoster::~CourseRoster()
{
    delete[] stdNames;
}
// Definitions of addStudent function
void CourseRoster::addStudent(string studentName)
{
    stdNames[size] = studentName;
    size++;
}
// Definition of print function
void CourseRoster::print()const
{
    cout << "List of Students" << endl;
    for (int i = 0; i < size; i++)
    {
        cout << stdNames[i] << endl;
    }
    cout << endl;
}
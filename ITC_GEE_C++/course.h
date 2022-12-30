/***************************************************************
 * The interface file for the class Course
 * ***************************************************************/
#ifndef COURSE_H
#define COURSE_H
#include <cassert>
#include <string>
#include <iostream>
#include "courseRoster.h"
using namespace std;

// Class definition
class Course
{
    private:
            string name;
            int units;
            CourseRoster* roster;
    public:
            Course(string name, int units);
            ~Course();
            string getName()const;
            CourseRoster* getRoster()const;
            void addStudent(string name);
            void print()const;
};
#endif
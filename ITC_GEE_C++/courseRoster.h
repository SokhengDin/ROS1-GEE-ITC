/***************************************************************
 * The interface file for the class CourseRoster
 * ***************************************************************/
#ifndef COURSEROSTER_H
#define COURSEROSTER_H
#pragma once

#include <string>
#include <iostream>
#include <cassert>
using namespace std;

// Class definition
class CourseRoster
{
    private:
            int size;
            string* stdNames;
    public:
            CourseRoster();
            ~CourseRoster();
            void addStudent(string studentName);
            void print()const;
};
#endif
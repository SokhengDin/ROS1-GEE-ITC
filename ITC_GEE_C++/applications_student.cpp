/***************************************************************
 * The application file to test classes in Registration project
 * ***************************************************************/
#include "registrar.h"
#include "registrar.cpp"
#include "student.h"
#include "student.cpp"
#include "course.h"
#include "course.cpp"
#include "studentSchedule.h"
#include "studentSchedule.cpp"
#include "courseRoster.h"
#include "courseRoster.cpp"
int main()
{
    // Instantiation of Registrar object
    Registrar registrar;
    // Instantiation of three student objects
    Student student1("John");
    Student student2("Mary");
    Student student3("Ann");
    // Instantiation of three course objects
    Course course1("CS101", 4);
    Course course2("CS102", 3);
    Course course3("CS103", 3);
    // Let the registrar object enroll students in the courses
    registrar.enroll(student1, course1);
    registrar.enroll(student1, course2);
    registrar.enroll(student2, course1);
    registrar.enroll(student2, course3);
    registrar.enroll(student3, course1);
    // Printing the information about each student
    student1.print();
    student2.print();
    student3.print();
    // Printing information about each course
    course1.print();
    course2.print();
    course3.print();
    return 0;
}
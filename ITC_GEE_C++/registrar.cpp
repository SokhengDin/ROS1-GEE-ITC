/**************************************************************
 * The implementation file for the class Registrar
 * ***************************************************************/
#include"registrar.h"

// Constructor
Registrar::Registrar()
{
}
// Destructor
Registrar::~Registrar()
{
}
// Enroll function
void Registrar::enroll(Student student, Course course)
{
    (course.getRoster()) -> addStudent(student.getName());
    (student.getSchedule()) -> addCourse(course.getName());
}
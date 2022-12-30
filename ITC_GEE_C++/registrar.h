/***************************************************************
 * The interface file for the class Registrar
 * ***************************************************************/
#ifndef REGISTRAR_H
#define REGISTRAR_H
#include "course.h"
#include "course.cpp"
#include "student.h"
#include "course.cpp"
// Class definition
class Registrar
{
    public:
            Registrar();
            ~Registrar();
            void enroll(Student student, Course course);
};
#endif
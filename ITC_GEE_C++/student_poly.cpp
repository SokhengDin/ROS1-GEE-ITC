#include "student_poly.h"

// Definition of constructor for student class
Student::Student(string nm, double gp)
:Person(nm), gpa(gp)
{
}
// Definition of virtual print function for student class
void Student::print()const
{
    Person::print();
    cout << "GPA:" << gpa << endl;
}
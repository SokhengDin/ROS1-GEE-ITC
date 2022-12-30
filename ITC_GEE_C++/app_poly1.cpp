/***************************************************************
 * Modification of application file to show the actual use of
 * polymorphism with an array of pointers.
 * ***************************************************************/
#include "student_poly.h"
#include "student_poly.cpp"
#include "person_poly.h"
#include "person_poly.cpp"
int main()
{
    // Declaration of an array of polymorphic variables (pointers)
    Person* ptr[4];
    // Instantiation of four objects in the heap memory
    ptr[0] = new Student("Joe", 3.7);
    ptr[1] = new Student("John", 3.9);
    ptr[2] = new Student("Bruce");
    ptr[3] = new Student("Sue");
    // Calling the virual print function for each objects
    for (int i = 0; i < 4; i++)
    {
        ptr[i] -> print();
        cout << endl;
    }
    // Deleting the objects in the heap
    for (int i = 0; i < 4; i++)
    {
        delete ptr[i];
    }
    return 0;
}
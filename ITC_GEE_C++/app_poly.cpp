/***************************************************************
 * The application file to test Person and Student classes
 * ***************************************************************/
#include "student_poly.h"
#include "student_poly.cpp"
#include "person_poly.h"
#include "person_poly.cpp"
int main()
{
    // Creating of ptr as polynomial variable
    Person* ptr;
    // Instantiation Person object in the heap
    ptr = new Person("Lucie");
    cout << "Person Information";
    ptr -> print();
    cout << endl;
    delete ptr;
    // Instantiation Student object in the heap
    ptr = new Student("John", 3.9);
    cout << "Student Information";
    ptr -> print();
    cout << endl;
    delete ptr;
    return 0;
}   

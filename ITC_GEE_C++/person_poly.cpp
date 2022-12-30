/***************************************************************
 * The implementation file for the Person class
 * ***************************************************************/
#include "person_poly.h"

// Definition of the Person constructor
Person::Person(string nm)
:name(nm)
{
}
// Definition of the Person destructor (virtual)
Person::~Person()
{
}
// Definition of the print function (virtual)
void Person::print()const
{
    cout << "Name:" << name << endl;
}
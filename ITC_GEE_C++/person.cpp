/***************************************************************
 * The implementation of functions in the Date class
 * ***************************************************************/
#include "person.h"

// Constructor
Person::Person(long id, Date bd)
:identity(id), birtDate(bd)
{
    assert(identity > 111111111 && identity < 999999999); 
}
// Destructor
Person::~Person()
{
}
// Print function
void Person::print()const
{
    cout << "Person Identity:" << identity << endl;
    cout << "Person date of birth:" ;
    birtDate.print();
    cout << endl << endl;
}
/***************************************************************
 * A simple program to show the first two conditions for
 * polymorphism
 * ***************************************************************/
#include <iostream>
#include <string>
using namespace std;

// Definition of Base class and in-line print function
class Base
{
    public:
            void print()const{cout << "In the Base" << endl;}
};
// Definition of Derivd class and in-line print function
class Derived:public Base
{
    public:
            void print()const{cout << "In the Derive" << endl;}
};

int main()
{
    // Creation of a pointer to the Base class (simulating object)
    Base* ptr;
    // Let ptr point to an object of the Base class
    ptr = new Base();
    ptr -> print();
    delete ptr;
    // Let ptr poing to an object of the Derivd class
    ptr = new Derived();
    ptr -> print();
    delete ptr;
    return 0;
}
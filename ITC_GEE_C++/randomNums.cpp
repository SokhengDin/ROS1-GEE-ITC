/***************************************************************
 * A program to declare, define, and use a class that generates
 * a random-number integer between any given range defined in
 * the constructor of the class.
 * ***************************************************************/
#include <iostream>
#include <cstdlib>
#include <ctime>
using namespace std;

// Class Definition // 
class RandomInteger
{
    private:
            int low; // Data member
            int high; // Data member
            int value; // Data member
    public:
            RandomInteger(int high, int low); // Parameter Constructor
            RandomInteger(); 
            ~RandomInteger(); // Destructor
            void print()const; // Accessor member function
};


// Defintion of constructor, destructor, and accessor member function
RandomInteger::RandomInteger(int lw, int hh) // Parameter constructor
:high(hh), low(lw)
{
    srand(time(0));
    int temp = rand();
    value = temp % (high - low + 1 ) + low;
    cout << "This is the parameter constructor" << endl;
}
// Definition of default constructor
RandomInteger::RandomInteger()
:high(100), low(1)
{
    cout << "This is the default constructor" << endl;
}
// Definition of destructor
RandomInteger::~RandomInteger()
{
    cout << "This is the destructor" << endl;
    cout << endl;
}
// Accessor memeber function

void RandomInteger::print()const
{
    cout << value << endl;
}
// Application to instantiate number
int main()
{
    // Generating a random integer between 100 and 200
    RandomInteger rand1(1000, 2000);
    cout << "Random number between 1000 and 2000:" << endl;
    rand1.print();
    // Generating a random integer between 200 and 1333
    RandomInteger rand2(200, 1333);
    cout << "Random number between 200 and 1333:" << endl;;
    rand2.print();
    // Generating a radnom integer by default value
    RandomInteger rand3;
    cout << "Random number by default value:" << endl;
    rand3.print();
    return 0;
}
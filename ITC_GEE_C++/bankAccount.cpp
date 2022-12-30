/*******************************************************************
 efine and use a bank account c* A program to declare, dlass
 * ****************************************************************/
#include <iostream>
#include <cassert>
using namespace std;

// Class definition
class Account
{
    private:
            long accNumber; // Data member
            double balance; // Data member
            static int base;  // Static data member
    public:
            Account(double balance); // Parameter constructor
            Account(); // Defaualt constructor
            ~Account(); // Destructor
            void checkBalance()const; // Accessor
            void deposit(double amount); // Mutator
            void withdraw(double ammount); // Mutator
};
// Initialization of static data member
int Account::base = 0;

// Definition of constructor
Account::Account(double bal) // Parameter constructor
:balance(bal)
{
    if (bal < 0.0)
    {
        cout << "Balance is negative; program terminates";
        assert(false);
    }
    base ++;
    accNumber = 1000000000 + base;

    cout << "Account: " << accNumber << "is opened." << endl;
    cout << "Balance #" << balance << endl << endl;
}
Account::Account()
:balance(1000)
{
    if (balance < 0.0)
    {
        cout << "Balance is negative, program terminates";
        assert(false);
    }
    base ++;
    accNumber = 1000000000 + base;

    cout << "Account: " << accNumber << "is opened." << endl;
    cout << "Balance #" << balance << endl << endl;
}
// Destructor
Account::~Account()
{
    cout << "Account # : " << accNumber << "is closed." << endl;
    cout << " $" << balance << "was send to the customer" << endl << endl;
}
// Accessor member function
void Account::checkBalance()const
{
    cout << "Account # : " << accNumber << endl <<  endl;
    cout << "Transaction: balance check" << endl;
    cout << "Balance $:" << balance << endl << endl;
}
// Mutator member function
void Account::deposit(double amount)
{
    if (amount > 0.0)
    {
        balance += amount;
        cout << "Account #:" << accNumber << endl <<  endl;
        cout << "Transaction: deposit of $" << amount << endl;
        cout << "New balance $:" << balance << endl << endl;
    }
    else{
        cout << "Transaction is aborted: " << endl; 
    }
}
// Mutator member function
void Account::withdraw(double amount)
{
    if (amount > balance)
    {
        amount = balance;
    }
    balance -= amount;
    cout << "Account #:" << accNumber << endl <<  endl;
    cout << "Transaction: withdraw of $" << amount << endl;
    cout << "New balance $:" << balance << endl << endl;
}

// Application
int main()
{
    // Creation of three accounts
    Account acc1(5000);
    Account acc2(10000);
    Account acc3(200000);
    Account acc4;
    // Transaction
    acc1.deposit(1000);
    acc2.checkBalance();
    acc3.withdraw(800);
    acc1.withdraw(2000);
    acc2.deposit(30000);
    acc4.checkBalance();
    return 0;
}
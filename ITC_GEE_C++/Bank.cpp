/***************************************************************
 * A program to declare, define, and use a bank account class
 * ***************************************************************/
#include <iostream>
#include <cassert>
using namespace std;

// Class definition
class Account
{
    private:
            long accNumber; // Data member
            double balance; // Data member
            static int base; // Static data member
    public:
            Account(double bal); // Constructor
            ~Account(); // Destructor
            void checkBalance()const; // Accessor
            void deposit(double amount); // Mutator
            void withdraw(double amount); // Mutator

};
// Initialization of static data member
int Account::base = 0;
// Parameter Constructor
Account::Account(double bal)
:balance(bal)
{
    if (bal < 0.0)
    {
        cout << " Balance is negative; program terminates";
        assert(false);
    }
    base ++;
    accNumber = 100000 + base;
    
    cout << "Account " << accNumber << " is opened." << endl;
    cout << "Balance $"  << balance << endl << endl;
}
// Destructor
Account::~Account()
{
    cout << "Account #:" << accNumber << " is closed." << endl;
    cout << "$" << balance << " was sent to the customer." << endl << endl;
}
// Accessor member function
void Account::checkBalance()const
{
    cout << "Account #:" << accNumber <<endl;
    cout << "Transaction: balance check" << endl;
    cout << "Balance: $" << balance << endl << endl;
}
// Mutator Member function
void Account::deposit(double amount)
{
    if (amount > 0.0)
    {
        balance += amount;
        cout << "Amount #:" << accNumber << endl;
        cout << "Transaction: deposit of $" << amount << endl;
        cout << "New balance: $" << balance << endl << endl;
    }
    else
    {
        cout << "Transaction aborted." << endl;
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
    cout << "Account #:" << accNumber << endl;
    cout << "Transaction: withdraw of $" << amount << endl;
    cout << "New balace: $" << balance << endl << endl;
}

// Application
int main()
{
    // Creation of three accounts
    Account acc1(2000);
    Account acc2(5000);
    Account acc3(1000);
    // Transaction
    acc1.deposit(150);
    acc2.checkBalance();
    acc1.checkBalance();
    acc3.withdraw(800);
    acc1.withdraw(1000);
    acc2.deposit(120);
    return 0;
}
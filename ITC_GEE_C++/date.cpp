/***************************************************************
 * The implementation of functions in the Date class
 * ***************************************************************/
#include "date.h"

// Paramter constructor
Date::Date(int m, int d, int y)
:month(m), day(d), year(y)
{
    if ((month < 1) || (month > 12))
    {
        cout << "Month is out of range. ";
        assert(false);
    }
    int dayInMonths[13] = {0,31,28,31,30,30,31,31,30,31,30,31};
    if ((day < 1) || (day > dayInMonths[month]))
    {
        cout << "Day out of range!";
        assert(false);
    }
    if ((year < 1900) || (year > 2099))
    {
        cout << "Year out of range!";
        assert(false);
    }
}
// Destructor
Date::~Date()
{
}
// Print member function
void Date::print()const
{
    cout << month << "/" << day << "/" << year << endl;
}
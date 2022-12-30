#include <iostream>
using namespace std;

class Kinematic
{
    private:
            double position;
            double time;
    public:
            Kinematic(double position, double time); // Parameter constructor
            Kinematic(); // Default constructor
            Kinematic(const Kinematic& kinematic);
            ~Kinematic();
            double calDist();
            double calVel();
            double calAccel();
            void print()const;
};

// Defintion of Parameter constructor
Kinematic::Kinematic(double pos, double t)
:position(pos), time(t)
{
    cout << "The parameter constructor was called" << endl;
}
// Definition of Default constructor
Kinematic::Kinematic()
:position(1), time(1)
{
    cout << "The default constructor was called." << endl;
}
// Defintion of Copy constructor
Kinematic::Kinematic(const Kinematic& kinematic)
:position(kinematic.position), time(kinematic.time)
{
    cout << "The copy constructor was called." << endl;
}

// Defintion of destructor
Kinematic::~Kinematic()
{
    cout << "The destructor was called." << endl;
}
// Defintiion of member function
double Kinematic::calDist()
{
    return position;
}
double Kinematic::calVel()
{
    double velocity;
    velocity = position * time;
    return velocity;
}
double Kinematic::calAccel()
{
    double acceleration;
    acceleration = (2*position)/(time*time);
    return acceleration;
}

// Application
int main()
{
    Kinematic motion1(100, 5); // Parameter
    cout << "Distance of motion 1 :" << motion1.calDist() << endl;
    cout << "Velcotiy of motion 1 :" << motion1.calVel() <<  endl;
    cout << "Accelertion of motion 1 :" << motion1.calAccel() << endl;
    Kinematic motion2(motion1); // Copy
    cout << "Distance of motion 2 :" << motion2.calDist() << endl;
    cout << "Velcotiy of motion 2 :" << motion2.calVel() <<  endl;
    cout << "Accelertion of motion 2 :" << motion2.calAccel() << endl;
    Kinematic motion3; // Default
    cout << "Distance of motion 3 :" << motion3.calDist() << endl;
    cout << "Velcotiy of motion 3 :" << motion3.calVel() <<  endl;
    cout << "Accelertion of motion 3 :" << motion3.calAccel() << endl;

    return 0;
}
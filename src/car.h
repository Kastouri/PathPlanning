#ifndef CAR_H
#define CAR_H
#include <vector>
#include <string>
using std::vector;
using std::string;

struct Car {
    double x;
    double y;
    double s;
    double d;
    double yaw;
    double speed; 
};

// TODO: you may need a class
/* 
class car
{
private:
    double x;
    double y;
    double vx; 
    double vy; 
    double s; 
    double d;
public
:
    car(double x, double y, double vx, double vy, double s, double d);

    ~car()
    ;
};

car::car( double x, double y, double vx, double vy, double s, double d)

{
}

car::~car()
{
} */
#endif
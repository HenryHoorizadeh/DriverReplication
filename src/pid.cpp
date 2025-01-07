#include "api.h"
// #include "auton.h"
#include "main.h"
#include "pid.h"
#include "robot.h"
#include "auton.h"

// #include "<valarray>"
// #include "<sstream>"
// #include "<string>"


using namespace pros;
using namespace c;
using namespace std;

bool mogoValues = false;
bool longValues = false;
bool stallProtection = false;
bool stalled = false;
int stallTime = 0;
int direc;
int direc2;
int hookpos;
int prevhookpos;
float view;
int stallC = 0;

double vKpl;
double vKil;
double vKdl;
float errorl; //amount from target
double prevErrorl; 
//double h;
int integrall; 
int derivativel;
int time2l;
double powerl;

//constants used for calculating power/voltage
double vKp;
double vKi;
double vKd;
float error; //amount from target
double prevError; 
//double h;
int integral; 
int derivative;
int time2;
double power; //voltage provided to motors at any given time to reach the target


//calc2
double vKp2;
double vKi2;
double vKd2;
float error2; //amount from target
double prevError2; 
double h2;
int integral2;
int derivative2;
int time22;
double power2;

//calc
double vKp3;
double vKi3;
double vKd3;
float error3; //amount from target
double prevError3; 
double h3;
int integral3;
int derivative3;
int time23;
double power3;


void setConstants(double kp, double ki, double kd) {
    vKp = kp;
    vKi = ki;
    vKd = kd;
} 

void setConstants2(double kp, double ki, double kd) {
    vKp2 = kp;
    vKi2 = ki;
    vKd2 = kd;
} 

void setConstants3(double kp, double ki, double kd) {
    vKp3 = kp;
    vKi3 = ki;
    vKd3 = kd;
} 

void resetEncoders() { //reset the chassis motors every time a target is reached
    LF.tare_position(); //or set_zero_position(0) or set_zero_position(LF.get_position()); (sets current encoder position to 0)
    LB.tare_position();
	RF.tare_position();
	RB.tare_position();
    RM.tare_position();
	LM.tare_position();
}


//setting method for driving straight or turning (pos neg voltages change directions)
void chasMove(int voltageLF, int voltageLM, int voltageLB, int voltageRF, int voltageRM, int voltageRB) { //voltage to each chassis motor
    LF.move(voltageLF);
    LM.move(voltageLM);
    LB.move(voltageLB);
    RF.move(voltageRF);
    RM.move(voltageRM);
    RB.move(voltageRB);
}

int slew = 3;
double calcPID(double target, double input, int integralKi, int maxIntegral) { //basically tuning i here

    int integral;
    
    prevError = error;
    error = target - input;
    
    if(abs(error) < integralKi) {
        integral += error;
    } else {
        integral = 0;
    }

    if(integral >= 0) {
        integral = min(integral, maxIntegral); //min means take whichever value is smaller btwn integral and maxI
        //integral = integral until integral is greater than maxI (to keep integral limited to maxI)
    } else {
        integral = max(integral, -maxIntegral); //same thing but negative max
    }

    derivative = error - prevError;

    power = (vKp * error) + (vKi * integral) + (vKd * derivative);

    return power;
} 



double calcPID2(double target, double input, int integralKi, int maxIntegral) { //basically tuning i here
    int integral2;
    prevError2 = error2;
    error2 = target - input;
    
    if(std::abs(error2) < integralKi) {
        integral2 += error2;
    } else {
        integral2 = 0;
    }

    if(integral2 >= 0) {
        integral2 = std::min(integral2, maxIntegral); //min means take whichever value is smaller btwn integral and maxI
        //integral = integral until integral is greater than maxI (to keep integral limited to maxI)
    } else {
        integral2 = std::max(integral2, -maxIntegral); //same thing but negative max
    }
    
    derivative2 = error2 - prevError2;

    

    power2 = (vKp2 * error2) + (vKi2 * integral2) + (vKd2 * derivative2);

    return power2;
}


double calcPID3(double target, double input, int integralKi, int maxIntegral) { //basically tuning i here
    int integral3;
    prevError3 = error3;
    error3 = target - input;
    
    if(std::abs(error3) < integralKi) {
        integral3 += error3;
    } else {
        integral3 = 0;
    }

    if(integral3 >= 0) {
        integral3 = std::min(integral3, maxIntegral); //min means take whichever value is smaller btwn integral and maxI
        //integral = integral until integral is greater than maxI (to keep integral limited to maxI)
    } else {
        integral3 = std::max(integral3, -maxIntegral); //same thing but negative max
    }
    
    derivative3 = error3 - prevError3;

    

    power3 = (vKp3 * error3) + (vKi3 * integral3) + (vKd3 * derivative3);

    return power3;
}

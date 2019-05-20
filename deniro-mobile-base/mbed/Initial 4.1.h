#include "string"

#define PIN2 p8 //This is Pin 8 on the Mbed/ White wire/ wheelchair control data bus
#define PIN4 p9 //This is Pin 9 on the Mbed/ Green wire/ Lights control data bus
#define PIN5 p10 //This is Pin 10 on the Mbed/ Green wire/ Lights control data bus

//Prototype for Initialization Functions
void writeInit();
void writeInitLights();

//Prototype for conversion function
string int2bin(int speed, int direction);
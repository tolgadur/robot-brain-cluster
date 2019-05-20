/*
These two functions initialize the chair's power electronics and 
the light's electronics. It was very difficult to recreate these initialization
streams, and they should be left alone unless it is ABSOLUTELY necessary 
for them to be modified.
*/

#include "mbed.h"
#include "FastIO.h"
#include "Initial.h"
#include "string"

//Create digital output
extern FastOut<PIN2> Pin2;
extern DigitalOut Pin4;
extern DigitalOut Pin5;


//Create Timer
Timer timer1;

//Initialization Sequences
string In1 = "01100101001100011010101111111111011001110111000000100011110100000001110110111100";
string green1 = "1001110011001111111111001110001101000000000000000000000000100110001001111111111011100111";
string green2 = "100111011000111111111100111000100000000000000000000000010011001110111111111101110011001";

void writeInitLights(){ //This function outputs the initialization bits for the lights electronics module on the chair
    
    Pin4.write(1);
    Pin5.write(0);
    wait_us(14);
    Pin4.write(0);
    Pin5.write(1);
    wait_ms(305);
    
    for (int j = 0; j < green1.size();j++){
        if (green1[j] == '0'){
            Pin4.write(0);
            Pin5.write(1);
            wait_us(25);
        }
        else{
            Pin4.write(1);
            Pin5.write(0);
            wait_us(25);
        }
    }
    
    Pin4.write(0);
    Pin5.write(1);
    wait_us(7760);
    
    for (int j = 0; j < green1.size();j++){
        if (green1[j] == '0'){
            Pin4.write(0);
            Pin5.write(1);
            wait_us(25);
        }
        else{
            Pin4.write(1);
            Pin5.write(0);
            wait_us(25);
        }
    }
    Pin4.write(0);
    Pin5.write(1);
    wait_ms(806);
}

void writeInit(){ // This outputs the initialization bits for the power electtronics for the chair
    
    //Magic number! x was calculated to be 4 but was found by experimentation to
    //be 4.04 This number defines the multiplier for the loop; i.e. how many times
    //to wait for each bit
    
    wait_ms(100);
    
    float x = 4.04;
    
    
    //Brute force method used here to recreate the correct timing for the initialization bits! The Mbed is not capable of outputing 
    //such short durations ~ 0.X us using conventional methods! DO NOT ALTER THIS CODE!
    
    for (int i = 0; i < int(5*x);i++){
    Pin2.write(1);
    }
    for (int i = 0; i < int(47*x);i++){
       Pin2.write(0);
    }
    for (int i = 0; i < int(6*x);i++){
       Pin2.write(1);
    }
    for (int i = 0; i < int(47*x);i++){
       Pin2.write(0);
    }
    for (int i = 0; i < int(6*x);i++){
       Pin2.write(1);
    }
    for (int i = 0; i < int(46*x);i++){
       Pin2.write(0);
    }
    for (int i = 0; i < int(7*x);i++){
       Pin2.write(1);
    }
    for (int i = 0; i < int(46*x);i++){
       Pin2.write(0);
    }
    for (int i = 0; i < int(8*x);i++){
       Pin2.write(1);
    }
    for (int i = 0; i < int(45*x);i++){
       Pin2.write(0);
    }
    for (int i = 0; i < int(8*x);i++){
       Pin2.write(1);
    }
    for (int i = 0; i < int(44*x);i++){
       Pin2.write(0);
    }
    for (int i = 0; i < int(9*x);i++){
       Pin2.write(1);
    }
    for (int i = 0; i < int(44*x);i++){
       Pin2.write(0);
    }
    for (int i = 0; i < int(10*x);i++){
       Pin2.write(1);
    }
    for (int i = 0; i < int(43*x);i++){
       Pin2.write(0);
    }
    for (int i = 0; i < int(10*x);i++){
       Pin2.write(1);
    }
    for (int i = 0; i < int(42*x);i++){
       Pin2.write(0);
    }
    for (int i = 0; i < int(11*x);i++){
       Pin2.write(1);
    }
    for (int i = 0; i < int(42*x);i++){
       Pin2.write(0);
    }
    for (int i = 0; i < int(11*x);i++){
       Pin2.write(1);
    }
    for (int i = 0; i < int(41*x);i++){
       Pin2.write(0);
    }
    for (int i = 0; i < int(12*x);i++){
       Pin2.write(1);
    }
    for (int i = 0; i < int(19*x);i++){
       Pin2.write(0);
    }
    for (int i = 0; i < int(6*x);i++){
       Pin2.write(1);
    }
    for (int i = 0; i < int(15*x);i++){
       Pin2.write(0);
    }
    for (int i = 0; i < int(13*x);i++){
       Pin2.write(1);
    }
    for (int i = 0; i < int(17*x);i++){
       Pin2.write(0);
    }
    for (int i = 0; i < int(8*x);i++){
       Pin2.write(1);
    }
    for (int i = 0; i < int(14*x);i++){
       Pin2.write(0);
    }
    for (int i = 0; i < int(14*x);i++){
       Pin2.write(1);
    }
    for (int i = 0; i < int(15*x);i++){
       Pin2.write(0);
    }
    for (int i = 0; i < int(9*x);i++){
       Pin2.write(1);
    }
    for (int i = 0; i < int(14*x);i++){
       Pin2.write(0);
    }
    for (int i = 0; i < int(14*x);i++){
       Pin2.write(1);
    }
    for (int i = 0; i < int(15*x);i++){
       Pin2.write(0);
    }
    for (int i = 0; i < int(9*x);i++){
       Pin2.write(1);
    }
    for (int i = 0; i < int(13*x);i++){
       Pin2.write(0);
    }
    for (int i = 0; i < int(15*x);i++){
       Pin2.write(1);
    }
    for (int i = 0; i < int(14*x);i++){
       Pin2.write(0);
    }
    for (int i = 0; i < int(11*x);i++){
       Pin2.write(1);
    }
    for (int i = 0; i < int(12*x);i++){
       Pin2.write(0);
    }
    for (int i = 0; i < int(15*x);i++){
       Pin2.write(1);
    }
    for (int i = 0; i < int(13*x);i++){
       Pin2.write(0);
    }
    for (int i = 0; i < int(12*x);i++){
       Pin2.write(1);
    }
    for (int i = 0; i < int(12*x);i++){
       Pin2.write(0);
    }
    for (int i = 0; i < int(16*x);i++){
       Pin2.write(1);
    }
    for (int i = 0; i < int(12*x);i++){
       Pin2.write(0);
    }
    for (int i = 0; i < int(14*x);i++){
       Pin2.write(1);
    }
    for (int i = 0; i < int(11*x);i++){
       Pin2.write(0);
    }
    for (int i = 0; i < int(17*x);i++){
       Pin2.write(1);
    }
    for (int i = 0; i < int(10*x);i++){
       Pin2.write(0);
    }
    for (int i = 0; i < int(15*x);i++){
       Pin2.write(1);
    }
    for (int i = 0; i < int(10*x);i++){
       Pin2.write(0);
    }
    for (int i = 0; i < int(18*x);i++){
       Pin2.write(1);
    }
    for (int i = 0; i < int(9*x);i++){
       Pin2.write(0);
    }
    for (int i = 0; i < int(17*x);i++){
       Pin2.write(1);
    }
    for (int i = 0; i < int(9*x);i++){
       Pin2.write(0);
    }
    for (int i = 0; i < int(19*x);i++){
       Pin2.write(1);
    }
    for (int i = 0; i < int(7*x);i++){
       Pin2.write(0);
    }
    for (int i = 0; i < int(17*x);i++){
       Pin2.write(1);
    }
    for (int i = 0; i < int(9*x);i++){
       Pin2.write(0);
    }
    for (int i = 0; i < int(21*x);i++){
       Pin2.write(1);
    }
    for (int i = 0; i < int(5*x);i++){
       Pin2.write(0);
    }
    for (int i = 0; i < int(18*x);i++){
       Pin2.write(1);
    }
    for (int i = 0; i < int(8*x);i++){
       Pin2.write(0);
    }
    for (int i = 0; i < int(45*x);i++){
       Pin2.write(1);
    }
    for (int i = 0; i < int(8*x);i++){
       Pin2.write(0);
    }
    for (int i = 0; i < int(46*x);i++){
       Pin2.write(1);
    }
    for (int i = 0; i < int(7*x);i++){
       Pin2.write(0);
    }
    for (int i = 0; i < int(46*x);i++){
       Pin2.write(1);
    }
    for (int i = 0; i < int(6*x);i++){
       Pin2.write(0);
    }
    for (int i = 0; i < int(47*x);i++){
       Pin2.write(1);
    }
    
    
    //Separation
    Pin2.write(0);
    wait_us(109470);
    
    
    //In1 Bits
    timer1.reset();
    timer1.start();
    for (int i = 0; i < In1.size();i++){
        if (In1[i] == '0'){
            Pin2.write(0);
            wait_us(25);
        }
        else{
            Pin2.write(1);
            wait_us(25);
        }
    }
    
    timer1.stop();
    Pin2.write(1);
    wait_us(119500 - 109470 - timer1.read_us());
}
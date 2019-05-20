/*
This is the main function. In the main while loop, we will continue to receive messages from the Rpi
and interpret them into the bit string required to send to the chair.

Also, we have two functions that are attached to tickers that interrupt the main loop to actually output 
the string to the chair
*/

#include "mbed.h"
#include "string"
#include "stdlib.h"
#include "fstream"
#include "iostream"
#include "vector"
#include "FastIO.h" //Use FastIO.h that is uploaded on the box as it has been slightly edited as compared to the source code
#include "Initial.h"
#include "sstream"
#include "MODSERIAL.h"
#include "time.h"

using namespace std;

//Define Strings for different commands
string Stationary =     "00101001011100000000001100000010101100000000001100000000001101010100011111111111110011111111110001010101110000000000110000001010110100001011110100011010110100111011111111111111111"; //Stationary
string StationaryAct1 = "00101001011100001000011100000000001101000000011100100000011100101010101111111111001111111111000101010111000000100111000000101011010000101111000001000111001011100011111111111111111111"; //Switch to Act1
string StationaryAct2 = "0010100101110000010001110000000000110 1000000011100100000011100100010111111111111111001111111111000101010111000000010111000000101011010000101111000001000111001011000111111111111111111"; //Switch to Act2

const int NumOfBits = 182; //Number of bits in the regular output string

//Define strings for lights
string no_lights = "100111011000111111111100111000100000000000000000000000010011001110111111111101110011001"; //All off
string lights_on = "1001110110001110011111001111101000000000000000000000000010011001110111111111101110011001"; //Headlights on
string lights_flashing = "1001110110001001111111001011001000000000000000000000000010011001110111111111101110011001"; //Blinkers on
string flashing_r = "100111011000101111111000100100100100000000000000000000010011001110111111111101110011001"; //Right blinkers on
string flashing_l = "100111011000110111111000110100100000000000000000000000010011001110111111111101110011001";//Left blinkers on

//Create i2c communication wih Rpi
I2CSlave s1(p28,p27); //SDA, SCL

//Create digital output
FastOut<PIN2> Pin2;
DigitalOut Pin4(PIN4);
DigitalOut Pin5(PIN5);

//Create Timer
Timer timer;

//Create Tickers for interrupt routines to output bits to chair
Ticker bits;
Ticker chair;

//Timeout for wasteSomeTime function
Timeout timeout;

//Initialize global commands to output to the VR2
//ocmd and olights are used so that if the interrupt to output bits occurs while the string is interpreted - using int2bin - we wont 
//output gibberish.
string cmd = Stationary;
string ocmd = Stationary;
string lights = no_lights;
string olights = no_lights;

//Variables to store parameters for int2bin()
int speed;
int direction;
int light;

//Timeout counter - Currently not used!
int timeout_ctr = 0;

//Indexes
unsigned i = 0; //ocmd index
unsigned j = 0; //olights index

//Buffers to read into using I2C
char buff [5];

//Misc
extern "C" void mbed_reset(); //Soft reset
bool valid = true; //Bool to verify validity of command
bool isRunning = false; //To prevent reentry

//Function prototypes
void wasteSomeTime();
void outputBits();
void reset();

//Create Local File System to write to memory - for debugging
LocalFileSystem local("local");

int main(){
    
    //Files for debugging
    //freopen("/local/cerr.txt", "w", stderr);
    //freopen("/local/cout.txt", "w", stdout);
    
    //Set up I2C communication parameters
    s1.address(0x60); //Address is actually 0x30 on the Pi due to left shift by mbed
    s1.frequency (100010);
    
    //Initialization code
    writeInitLights(); //Initialization sequence for the lights
    writeInit(); //Initialization sequence for the chair
    
    //Tickers
    bits.attach_us(&outputBits, 25); //This is a ticker that flips the digital outputs every 25us 
    chair.attach_us(&reset, 10000); //This is a ticker that resets the output every 10ms (100Hz)
    
    while(true){
        
        //Receive new information
        s1.read(buff, 5); //Read into buffer
        if (buff[0] == 0 && buff[1] == 83){ //buff[0] is always zero, buff[1] is signal1 from Pi - This is just for verification
            speed = buff[2];
            direction = buff[3];
            light = buff[4];
        }
    
        else if (buff[0] == 0 && buff[1] == 73) { //buff[0] is always zero, buff[1] is signal1 from Pi - This is to switch modes of operation between drive and actuators
            cmd = StationaryAct1;
            valid = true;
            continue;
        }
        
        else if (buff[0] == 0 && buff[1] == 63) {
            mbed_reset(); //software reset    
        }
        
        else if (timeout_ctr < 10){
            timeout_ctr++; //timeout counter to break if no valid messages received ten times in a row!
        }
        
        else{ //After 10 timed out iterations, for safety stop moving
            speed = 0;
            direction = 0;
            
        }
        
        cmd = int2bin (speed, direction); //Change into string
        valid = true; //This is to verify that a valid string is stored in cmd so that it can be transferred to ocmd for output
    }
    
    //fclose(stderr);
    //fclose(stdout);
}

//First Ticker function - This function outputs the bits to the chair by interrupting the main function and switching the logic on pins 8,9,10 accordingly
//Period = 25us
void outputBits (){
    
    //Chair output
    if (i < NumOfBits){
        if (ocmd[i] == '0'){
            Pin2.write(0);
        }
        else if (ocmd[i] == '1'){
            Pin2.write(1);
        }
        i++;
    }
    else {
        Pin2.write(1);
    }
    
    //Lights output
    if (j < olights.size()){
        if (olights[j] == '0'){
            Pin4.write(0);
            Pin5.write(1);
        }
        else{
            Pin4.write(1);
            Pin5.write(0);
        }
        j++;
    }
    else {
        Pin4.write(0);
        Pin5.write(1);
    }
}


//Second Ticker function - This function resets the frame that is output (ocmd/olights) such that the frequency is 100Hz, the chair frequency.
void reset (){
    if (valid){
        while(i < NumOfBits);
        ocmd = cmd;
        olights = lights;
        valid = false;
    }
    
    while(i < NumOfBits);
    //Reset indexe
    i = 0;
    j = 0;   
}
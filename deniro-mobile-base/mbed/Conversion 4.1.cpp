/*
This function generates the bit string that needs to be sent to the chair by piecing together parts of the string 
into the final string.
part1Drive = part1Drive + byte1 + parity1 + "110" + byte2 + parity2 + "110" + byte3 + parity3 + part2Act
*/

#include "mbed.h"
#include "FastIO.h"
#include "Initial.h"
#include "string"
#include "bitset"
#include "stdlib.h"
#include "sstream"
#include "iostream"

using namespace std;


//Prototype
void bitgen (int number, int (&byte)[8], char &par);

string int2bin(int speed, int direction){
    
    //string command = "0010100101110000000000110000001010110 011100111 110 000000000 110 111000100 1111111111110011111111110001010101110000000000110000001010110100001011110100011010110100111011111111111111111";
    //                                                         1stByte       2ndByte       3rdByte
    string part1Drive = "0010100101110000000000110000001010110"; //37 bits
    string part1Act =   "0010100101110000100001110000001010110";
    string part2 =    "1111111111110011111111110001010101110000000000110000001010110100001011110100011010110100111011111111111111111";
    string part2Act = "1111111111111001111111111000101010111000000010111000000101011010000101111000001000111001011000111111111111111111"; //112 bits
    
    int checksum = 21 - speed - direction;
    
    int byte1 [8];
    int byte2 [8];
    int byte3 [8];

    char par1;
    char par2;
    char par3;
    
    //Another 33 bits

    //Speed Byte
    bitgen(speed, byte1, par1);
    string temp1;
    for (int i = 0;i< 8;i++){
        string String = static_cast<ostringstream*>( &(ostringstream() << byte1[i]) )->str();
        temp1 = temp1 + String;
    }
    part1Drive = part1Drive + temp1 + par1 + "110";
    
    //Direction
    bitgen(direction, byte2, par2);
    string temp2;
    for (int i = 0;i< 8;i++){
        string String = static_cast<ostringstream*>( &(ostringstream() << byte2[i]) )->str();
        temp2 = temp2 + String;
    }
    part1Drive = part1Drive + temp2 + par2 + "110";
    
    //Checksum 
    bitgen(checksum, byte3, par3);
    string temp3;
    for (int i = 0;i< 8;i++){
        string String = static_cast<ostringstream*>( &(ostringstream() << byte3[i]) )->str();
        temp3 = temp3 + String;
    }
    part1Drive = part1Drive + temp3 + par3 + part2Act; 

    return part1Drive;
}

//This is the function that converts signed decimal to twos complement binary and also returns the parity of the byte
void bitgen (int number, int (&byte)[8], char &par){
    if (number >=0){ //If the number is positive, then it is an easy task
        for (int count = 0;count < 8;count++){
            if (number % 2 == 0){
                byte[count] = 0;
            }
            else{
                byte[count] = 1;
            }
            number = number/2;
        }
    }
    
    /*
    For negative numbers we need to invert the number to positive, change it into binary, invert the bits and
    then add 1 to the number
    */
    else{ // For two's complement conversion for negative speeds
        number = -number; 
        for (int count = 0;count < 8;count++){
            byte[count] = number % 2;
            number = number/2;
        }
        
        for(int i = 0;i <= 7 ;i++){
            byte[i] = 1 - byte[i];
        }
        
        for(int i = 0;i <=7 ;i++){ 
            if (byte[i] == 0){
                byte[i] = 1;
                break;
            }
            else {
                byte[i] = 0;
            }
        }
    }
    
    //parity generation
    int parity  = 0;
    for (int i = 0; i <8; i++){
        if (byte[i] == 1){
            parity++;
        }
    }
    
    if (parity % 2 !=0)
        par = '1';
    else
        par = '0';
    
}

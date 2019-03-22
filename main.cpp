#include "mbed.h"
#include "SHA256.h"
#include "Crypto.h"
#include "PwmOut.h"
#include <sstream>
#include <string.h>

//Photointerrupter input pins
#define I1pin D3
#define I2pin D6
#define I3pin D5

//Incremental encoder input pins
#define CHApin   D12
#define CHBpin   D11

//Motor Drive output pins   //Mask in output byte
#define L1Lpin D1           //0x01
#define L1Hpin A3           //0x02
#define L2Lpin D0           //0x04
#define L2Hpin A6           //0x08
#define L3Lpin D10          //0x10
#define L3Hpin D2           //0x20

#define PWMpin D9
PwmOut controlPWM(PWMpin);

//Motor current sense
#define MCSPpin   A1
#define MCSNpin   A0

//define serial pins as pc
RawSerial pc(SERIAL_TX, SERIAL_RX);

//Buffer incoming characters
Queue<void,8> inCharQ; 

#define MAXTRQ 1000

/**************** Declaring Global Variables *****************/
// volative keyword to use safely in input thread
volatile uint64_t newKey = 0.0;
Mutex newKey_mutex;

volatile float newPwm = 0.0;
Mutex newPwm_mutex;

volatile float maxSpeed = 0.0;
Mutex maxSpeed_mutex;

volatile float noOfRotation = 0.0;
Mutex noOfRotation_mutex;

int inputArraySize = 64;

// counter to find hashing frequency
uint64_t countHashes = 0;

// timer to calculate hashing frequency
Timer btcTimer;

//Variables for motor control
volatile float motorPos;

volatile int32_t motorInitPos;

volatile int32_t trqVal = MAXTRQ; //defining the torque value

/*************************************************************/
//Mapping from sequential drive states to motor phase outputs
/*
State   L1  L2  L3
0       H   -   L
1       -   H   L
2       L   H   -
3       L   -   H
4       -   L   H
5       H   L   -
6       -   -   -
7       -   -   -
*/
//Drive state to output table
const int8_t driveTable[] = {0x12,0x18,0x09,0x21,0x24,0x06,0x00,0x00};

//Mapping from interrupter inputs to sequential rotor states. 0x00 and 0x07 are not valid
const int8_t stateMap[] = {0x07,0x05,0x03,0x04,0x01,0x00,0x02,0x07};  
//const int8_t stateMap[] = {0x07,0x01,0x03,0x02,0x05,0x00,0x04,0x07}; //Alternative if phase order of input or drive is reversed

//Phase lead to make motor spin
int8_t lead = 2;            //2 for forwards, -2 for backwards

//global variable orstate
int8_t orState = 0;         //Rotot offset at motor state 0

uint32_t prevPos = 0;       //Previous motor position used in motorCtrlFn

uint8_t printVelCtr = 1;    //Counter for speed printing


/************** Defining Threads *************/
Thread outputThread;
Thread inputThread;

/************** Defining Tickers *************/
Ticker motorTicker;
Thread motorCtrlT (osPriorityNormal,1024);

void motorCtrlTick(){
    motorCtrlT.signal_set(0x1);
}


/********************* Defining the Structure of Input Thread   **************************/
// structure used to send data through mail
typedef struct{
    int identifier;
    uint64_t sentData;
    float sentDataFloat;
    } info_t;
    
    
void serialISR(){
 uint8_t newChar = pc.getc();
 inCharQ.put((void*)newChar);
 }
/*****************************************************************************************/

/******************** Defining the Structure of the Output Thread  ***********************/
// define the mail
Mail<info_t,16> sentInfo;

// putMessage as defined in the rtos documentation
void putMessage(int identifier, uint64_t sentData){
    info_t* infoPtr = sentInfo.alloc();
    infoPtr -> identifier = identifier;
    infoPtr -> sentData = sentData;
    sentInfo.put(infoPtr);
}
// putMessage as defined in the rtos documentation
void putMessage(int identifier, float sentData){
    info_t* infoPtr = sentInfo.alloc();
    infoPtr -> identifier = identifier;
    infoPtr -> sentDataFloat = sentData;
    sentInfo.put(infoPtr);
}
/*****************************************************************************************/
/****************************** Motor Control Function ***********************************/
void motorCtrlFn(){
    Ticker motorCtrlTicker;
    motorCtrlTicker.attach_us(&motorCtrlTick,100000);
    int32_t ctr = 0;
    
    float prevMotorPos = motorPos;
    float speed = 0;
    float k_ps = 25.0;
    
    float e_r = 0.0;
    float prevE_r = 0.0;
    float e_rDiff = 0.0;
        
    int32_t T_s;
    int32_t T_r;
    
    while(1){
        ctr++;
        motorCtrlT.signal_wait(0x1);
        
        //Calculating the Position
        if(ctr == 10){
            putMessage(5, (motorPos - motorInitPos)/6.0f);
        }
        
        //Calculating the Speed
        speed = (float) (motorPos - prevMotorPos)/(6*0.1);
        if(ctr == 10){
            putMessage(4, speed);
            ctr = 0;
        }
        prevMotorPos = motorPos;       
        
        //Controlling the Speed of the Motor
        maxSpeed_mutex.lock();        
        T_s = k_ps * (maxSpeed - abs(speed));
        if(maxSpeed == 0){ 
            T_s = MAXTRQ;
        }
        maxSpeed_mutex.unlock();
        
        //Contolling the Position of the Motor
        noOfRotation_mutex.lock();
        e_r = noOfRotation - (motorPos - motorInitPos);
        if(noOfRotation == 0){ 
            e_r = MAXTRQ;
        }
        noOfRotation_mutex.unlock();

        e_rDiff = (e_r - prevE_r)/0.1;
        T_r = 25 * e_r + 20 * e_rDiff;
        
        //Choosing the output torque
        if(speed < 0) { //if the sign is negative
            if(T_s > T_r){
                trqVal = T_s;
                }
            else{
                trqVal = T_r;
            }
        } 
        else { //if the sign is positive
            if(T_s > T_r){
                trqVal = T_r;
            }
            else{
                trqVal = T_s;
            }
        }
    }
}

/*****************************************************************************************/

/************** Handling the Output Thread  ****************/
// implementing the everlasting loop for the output thread
void outputThreadLoop(){
    while (1) {
        osEvent event = sentInfo.get();
        info_t* infoPtr = (info_t*)event.value.p;

        // to reduce message size, we only send 1 field at a time, and decode the message type by using an identifier field        
        if(infoPtr->identifier == 0){
            pc.printf("The current hashrate is: %llu\n\r", infoPtr->sentData);
        }
        else if(infoPtr->identifier == 1){
            pc.printf("The last found nonce is: 0x%llx\n\r", infoPtr->sentData);
        }
        else if(infoPtr->identifier == 2){
            pc.printf("The new key for the sequence is: 0x%016x\n\r", infoPtr->sentData);
        }
        else if(infoPtr->identifier == 3){
            pc.printf("PWM duty cycle set to: %llu\n\r", infoPtr->sentData);
        }
        else if(infoPtr->identifier == 4){
            pc.printf("Current speed is %f\n\r", infoPtr->sentDataFloat);
        }
        else if(infoPtr->identifier == 5){
            pc.printf("Current position is %f\n\r", infoPtr->sentDataFloat);
        }
        else if(infoPtr->identifier == 6){
            pc.printf("The maximum speed is %f\n\r", infoPtr->sentDataFloat);
        }
        else if(infoPtr->identifier == 7){
            pc.printf("The number of rotation set to %f\n\r", infoPtr->sentDataFloat);
        }
        else{   // if the identifier is unknown, we report it as an error
            pc.printf("Sorry, wrong identifier, it does not correspond to a message...");
        }
        sentInfo.free(infoPtr);
    }
}
/*******************************************************/

/************** Handling the Input Thread  *************/
void inputThreadLoop(){
    // array to hold each command
    char ipCommand[inputArraySize];
    uint8_t indexCounter = 0;
    pc.attach(&serialISR);
    while(1) {
        osEvent newEvent = inCharQ.get();
        uint8_t newChar = (uint8_t) newEvent.value.p;
        ipCommand[indexCounter] = newChar;
        if(indexCounter == inputArraySize-1) {
            pc.printf("Input string is getting out of bounds.\n\r");
            indexCounter = 0;
        } 
        else {
            indexCounter++;
        }

        if(newChar == '\r'){
             ipCommand[indexCounter] = '\0';
             indexCounter = 0;
             if (ipCommand[0] == 'K'){
                newKey_mutex.lock();
                sscanf(ipCommand, "K%x", &newKey); //Decode the command
                putMessage(2,newKey);
                newKey_mutex.unlock();
            }
             if (ipCommand[0] == 'P'){
                newPwm_mutex.lock();
                sscanf(ipCommand, "P%f", &newPwm); //Decode the command
                putMessage(3,newPwm);
                controlPWM.write(newPwm);
                newPwm_mutex.unlock();
            }
            if (ipCommand[0] == 'V'){
                maxSpeed_mutex.lock();
                sscanf(ipCommand, "V%f", &maxSpeed); //Decode the command
                putMessage(6,maxSpeed);
                maxSpeed_mutex.unlock();
            }
            if (ipCommand[0] == 'R'){
                noOfRotation_mutex.lock();
                sscanf(ipCommand, "R%f", &noOfRotation); //Decode the command
                putMessage(7, noOfRotation);
                noOfRotation = noOfRotation * 6;
                noOfRotation_mutex.unlock();
                motorInitPos = motorPos;
            }
            
        }
    }
}
/*******************************************************/

//Status LED
DigitalOut led1(LED1);

//Photointerrupter inputs
InterruptIn I1(I1pin);
InterruptIn I2(I2pin);
InterruptIn I3(I3pin);

//Motor Drive outputs
DigitalOut L1L(L1Lpin);
DigitalOut L1H(L1Hpin);
DigitalOut L2L(L2Lpin);
DigitalOut L2H(L2Hpin);
DigitalOut L3L(L3Lpin);
DigitalOut L3H(L3Hpin);

//Set a given drive state
void motorOut(int8_t driveState){
    
    //Lookup the output byte from the drive state.
    int8_t driveOut = driveTable[driveState & 0x07];
      
    //Turn off first
    if (~driveOut & 0x01) L1L = 0;
    if (~driveOut & 0x02) L1H = 1;
    if (~driveOut & 0x04) L2L = 0;
    if (~driveOut & 0x08) L2H = 1;
    if (~driveOut & 0x10) L3L = 0;
    if (~driveOut & 0x20) L3H = 1;
    
    //Then turn on
    if (driveOut & 0x01) L1L = 1;
    if (driveOut & 0x02) L1H = 0;
    if (driveOut & 0x04) L2L = 1;
    if (driveOut & 0x08) L2H = 0;
    if (driveOut & 0x10) L3L = 1;
    if (driveOut & 0x20) L3H = 0;
    }
    
    //Convert photointerrupter inputs to a rotor state
    inline int8_t readRotorState(){
    return stateMap[I1 + 2*I2 + 4*I3];
    }

//Basic synchronisation routine    
int8_t motorHome() {
    //Put the motor in drive state 0 and wait for it to stabilise
    motorOut(0);
    wait(2.0);
    
    //Get the rotor state
    return readRotorState();
}

//Interrupt Service Routine for driving motor field
void field_ISR(){
    int8_t currState = readRotorState();
    static int prevState;
    
    int32_t torque = 0;
    int32_t prevTrqVal = 0;
    
    if (prevTrqVal != trqVal) {
        torque = trqVal;
        // Setting the lead based on the direction of the lead
        if(torque < 0) {
            torque = -torque;
            lead = -2;
        } else {
            lead = 2;
        }
        // Set to maximum if troque exceeds it
        if(torque > MAXTRQ){
            torque = MAXTRQ;
        }
    }
    
    motorOut((currState-orState+lead+6)%6); //+6 to make sure the remainder is positive
    prevTrqVal = trqVal;
    
    if (currState - prevState == 5){ motorPos --;}  // as per the table given in the Lab3 instructions
    else if (currState - prevState == -5){ motorPos ++;}
    else {motorPos += (currState - prevState);}
    prevState = currState;
}

int main() {
    
    /************************** BITCOIN BLOCK ***************************/
    SHA256 dwarf;
    uint8_t sequence[] = {0x45,0x6D,0x62,0x65,0x64,0x64,0x65,0x64,
    0x20,0x53,0x79,0x73,0x74,0x65,0x6D,0x73,
    0x20,0x61,0x72,0x65,0x20,0x66,0x75,0x6E,
    0x20,0x61,0x6E,0x64,0x20,0x64,0x6F,0x20,
    0x61,0x77,0x65,0x73,0x6F,0x6D,0x65,0x20,
    0x74,0x68,0x69,0x6E,0x67,0x73,0x21,0x20,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
    uint64_t* key = (uint64_t*)((int)sequence + 48);
    uint64_t* nonce = (uint64_t*)((int)sequence + 56);
    uint8_t hash[32];
    /********************************************************************/

    // introduction message
    pc.printf("Hello-coders\n\r");
    
    // initializing the period and the duty cycle    
    controlPWM.period(0.2f);
    controlPWM.write(1.0f);
    
    //Run the motor synchronisation
    orState = motorHome();
    pc.printf("Rotor origin: %x\n\r",orState);
    motorPos = orState;
    //orState is subtracted from future rotor state inputs to align rotor and motor states
    
    //Setting the Interrupt Service Routine
    I1.rise(&field_ISR);
    I2.rise(&field_ISR);
    I3.rise(&field_ISR); 
    
    I1.fall(&field_ISR);
    I2.fall(&field_ISR);
    I3.fall(&field_ISR);


    // this is the output thread that handles all printing to serial output
    outputThread.start(outputThreadLoop);
    inputThread.start(inputThreadLoop);
    motorCtrlT.start(motorCtrlFn);

    // starts the counter for hashing frequency
    btcTimer.start();
    
    // infinite loop
    while (1) {
        newKey_mutex.lock();
        *key = newKey;
        newKey_mutex.unlock();
        
        // dwarves are miners in world of warcraft :)
        dwarf.computeHash(hash, sequence, 64);
        countHashes++;
        // when the counter reaches one second, print the number of hashes and reset everything
        if(btcTimer.read() >= 1){
            putMessage(0, countHashes);            
            btcTimer.reset();
            countHashes = 0;
        }
        // when a nonce is found, print it
        if(hash[0] == 0 && hash[1] == 0){
            putMessage(1, *nonce);            
        }
        *nonce = *nonce + 1; // then increment
    }
}

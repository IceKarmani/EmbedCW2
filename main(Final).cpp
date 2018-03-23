#include "mbed.h"
#include "SHA256.h"
#include "rtos.h"
#include "Queue.h"
#include "Thread.h" //do we need this?
//#include "rtos.h"

//Photointerrupter input pins
#define I1pin D2
#define I2pin D11
#define I3pin D12

//Incremental encoder input pins
#define CHA   D7
#define CHB   D8  

//Motor Drive output pins   //Mask in output byte
#define L1Lpin D4           //0x01
#define L1Hpin D5           //0x02
#define L2Lpin D3           //0x04
#define L2Hpin D6           //0x08
#define L3Lpin D9           //0x10
#define L3Hpin D10          //0x20

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
const int8_t lead = 2;  //2 for forwards, -2 for backwards

//Status LED
DigitalOut led1(LED1);

//Photointerrupter inputs
InterruptIn I1(I1pin);
InterruptIn I2(I2pin);
InterruptIn I3(I3pin);

//Motor Drive outputs
PwmOut L1L(L1Lpin);
DigitalOut L1H(L1Hpin);
PwmOut L2L(L2Lpin);
DigitalOut L2H(L2Hpin);
PwmOut L3L(L3Lpin);
DigitalOut L3H(L3Hpin);

//Set a given drive state
void motorOut(int8_t driveState, uint32_t mP){
    
    //Lookup the output byte from the drive state.
    int8_t driveOut = driveTable[driveState & 0x07];
      
    //Turn off first
    if (~driveOut & 0x01) L1L.pulsewidth_us(0);
    if (~driveOut & 0x02) L1H = 1;
    if (~driveOut & 0x04) L2L.pulsewidth_us(0);
    if (~driveOut & 0x08) L2H = 1;
    if (~driveOut & 0x10) L3L.pulsewidth_us(0);
    if (~driveOut & 0x20) L3H = 1;
    
    //Then turn on
    if (driveOut & 0x01) L1L.pulsewidth_us(mP);
    if (driveOut & 0x02) L1H = 0;
    if (driveOut & 0x04) L2L.pulsewidth_us(mP);
    if (driveOut & 0x08) L2H = 0;
    if (driveOut & 0x10) L3L.pulsewidth_us(mP);
    if (driveOut & 0x20) L3H = 0;
    }
    
    //Convert photointerrupter inputs to a rotor state
inline int8_t readRotorState(){
    return stateMap[I1 + 2*I2 + 4*I3];
    }
    
//declare global variable
int8_t orState;
int8_t intState;
int8_t intStateOld;
int8_t newlead;
int32_t motorPower; 
int32_t motorPosition, motorPositionRead, ys, yr, y, maxspeed, velocity, oldmotorPosition, rotorSet, Er, dEr, oldEr;
int32_t kp = 25;
int32_t kd = 15;
Queue<void, 8> inCharQ;
volatile uint64_t newKey; 
volatile uint32_t newM;
volatile uint64_t newMaxV, newRotor;
Mutex newKey_mutex, newMaxV_mutex, newRotor_mutex;
Thread commOutT(osPriorityNormal,1024), decCommT(osPriorityNormal,1024), motorCtrlT(osPriorityNormal,1024);
Timer timerHash, timerRotor;

//Basic synchronisation routine    
int8_t motorHome() {
    //Put the motor in drive state 0 and wait for it to stabilise
    motorOut(0, 1000);
    wait(1.0);
    
    //Get the rotor state
    return readRotorState();
}


//Poll the rotor state and set the motor outputs accordingly to spin the motor
void ISR() {
    intState = readRotorState();
    if (intState != intStateOld) {
        intStateOld = intState;
        motorOut((intState-orState+lead+6)%6, motorPower); //+6 to make sure the remainder is positive
    }
}

typedef struct{
 uint8_t code;
 uint32_t data;
 } message_t ;

Mail<message_t,16> outMessages;
//Initialise the serial port
RawSerial pc(SERIAL_TX, SERIAL_RX);

void commOutFn(){
    while(1) {
        osEvent newEvent = outMessages.get();
        message_t *pMessage = (message_t*)newEvent.value.p;
        pc.printf("Message %d with data 0x%016x\n\r",
        pMessage->code,pMessage->data);
        outMessages.free(pMessage);
    }
}

void putMessage(uint8_t code, uint32_t data){
 message_t *pMessage = outMessages.alloc();
 pMessage->code = code;
 pMessage->data = data;
 outMessages.put(pMessage);
}
 
void serialISR(){
 uint8_t newChar = pc.getc();
 inCharQ.put((void*)newChar);
}
 
void motorISR(){
    //motorPower = newM;
    if(y < 0){
        newlead = -lead;   //change direction for negative velocity
    }
    else{
        newlead = lead;   
    }
    motorPower = abs(y);
    if(motorPower > 1000){
        motorPower = 1000;           
    }
    static int8_t oldRotorState;
    int8_t rotorState = readRotorState();
    motorOut((rotorState-orState+newlead+6)%6,motorPower);
    if (rotorState - oldRotorState == 5) motorPosition--;
    else if (rotorState - oldRotorState == -5) motorPosition++;
    else motorPosition += (rotorState - oldRotorState);
    oldRotorState = rotorState;
}

void decCommFn(){
    pc.attach(&serialISR);
    char Cmd[256];
    int i=0;
    while(1) {
        osEvent newEvent = inCharQ.get();
        uint8_t newChar = (uint8_t)newEvent.value.p;
        Cmd[i]= newChar;
        if(Cmd[i] == '\r'){
            Cmd[i] = '\0';
            i = -1;
            if (Cmd[0] == 'K'){
                newKey_mutex.lock();
                sscanf(Cmd, "K%x", &newKey); //Decode the command
                newKey_mutex.unlock();
                putMessage(3, newKey);
            }
            if (Cmd[0] == 'T') {
                sscanf(Cmd, "T%x", &newM); //Decode the command
                putMessage(5, newM);
            }
            if (Cmd[0] == 'V'){
                newMaxV_mutex.lock();
                sscanf(Cmd, "V%x", &newMaxV); //Decode the command
                newMaxV_mutex.unlock();
                putMessage(9, newMaxV);
            }
           if (Cmd[0] == 'R'){
                newRotor_mutex.lock();
                sscanf(Cmd, "R%x", &newRotor); //Decode the command
                newRotor_mutex.unlock();
                putMessage(8, newRotor);
            }
        }
        i++;
        if(i > 255){
            //what to do when array size exceeded
            printf("Warning: array size of 'Cmd' exceeded");   
        }
    }
}

void motorCtrlTick(){
 motorCtrlT.signal_set(0x1);
}

int32_t sgn(int32_t x){
    if(x < 0){
        return -1;   
    }    
    else{
        return 1;   
    }
}

int32_t max(int32_t x, int32_t y){
    if(x > y){
        return x;   
    }
    else{
        return y;   
    }   
}

int32_t min(int32_t x, int32_t y){
    if(x > y){
        return y;   
    }
    else{
        return x;   
    }   
}

void motorCtrlFn(){
 int v=0;
 Ticker motorCtrlTicker;
 motorCtrlTicker.attach_us(&motorCtrlTick,100000);
 while(1){
    newMaxV_mutex.lock();
    maxspeed = newMaxV;
    newMaxV_mutex.unlock();    
    if(maxspeed == 0){  //command V0 sets motor speed to maximum
        maxspeed = 1000;   
    }
    newRotor_mutex.lock();
    rotorSet = newRotor;
    newRotor_mutex.unlock(); 
    oldmotorPosition = motorPosition;
    timerRotor.start();                      //timer used to calculate rate of change in error
    oldEr = rotorSet - oldmotorPosition;
    motorCtrlT.signal_wait(0x1);
    motorPositionRead = motorPosition;      //copies motorPosition into new variable to avoid multiple access
    Er = rotorSet - motorPositionRead;
    dEr = (Er - oldEr)/timerRotor.read();   //differential term 
    yr = (kp * Er) + (kd * dEr);
    timerRotor.reset();
    velocity = (motorPosition - oldmotorPosition)* 10;
    ys = kp * (maxspeed - abs(velocity)) * sgn(Er);
    if(velocity < 0){
        y = max(ys, yr);   
    }
    else{
        y = min(ys, yr);   
    }
    v++;
    if(v == 10){
        v=0;
        //putMessage(10, y);
        //putMessage(6, velocity);
        //putMessage(7, motorPosition);
    }
 }
} 

void setup(){
    //Intialise PWM periode
    L1L.period_us(2000);  
    L2L.period_us(2000);  
    L3L.period_us(2000);  
    
    orState = 0;    //Rotor offset at motor state 0
    intState = 0;
    intStateOld = 0;
    pc.printf("Hello\n\r");
    
    //Run the motor synchronisation
    orState = motorHome();
    pc.printf("Rotor origin: %x\n\r",orState);   
    
    //orState is subtracted from future rotor state inputs to align rotor and motor states
    intState = readRotorState();
    if (intState != intStateOld) {
        intStateOld = intState;
        motorOut((intState-orState+lead+6)%6, motorPower); //+6 to make sure the remainder is positive
    };
    
    //Poll the rotor state and set the motor outputs accordingly to spin the motor
    I1.rise(&motorISR);
    I2.rise(&motorISR);
    I3.rise(&motorISR);
    I1.fall(&motorISR);
    I2.fall(&motorISR);
    I3.fall(&motorISR);
}
    
//Main
int main() {
    
    setup();
    
    //create and start threads
    commOutT.start(commOutFn);
    decCommT.start(decCommFn);
    motorCtrlT.start(motorCtrlFn);
    
    //Declare and initialise the input sequence and hash
    uint8_t sequence[] = {0x45,0x6D,0x62,0x65,0x64,0x64,0x65,0x64,
    0x20,0x53,0x79,0x73,0x74,0x65,0x6D,0x73,
    0x20,0x61,0x72,0x65,0x20,0x66,0x75,0x6E,
    0x20,0x61,0x6E,0x64,0x20,0x64,0x6F,0x20,
    0x61,0x77,0x65,0x73,0x6F,0x6D,0x65,0x20,
    0x74,0x68,0x69,0x6E,0x67,0x73,0x21,0x20,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
    //key and nonce are initialised as pointers to 64-bit integers within sequence
    uint64_t* key = (uint64_t*)((int)sequence + 48);    
    uint64_t* nonce = (uint64_t*)((int)sequence + 56);
    uint8_t hash[32];  
    
    int counter =0; //intialise the counter (number of hashes computed)
    float t;       //variable to store time in
    float hr;   //hash rate (hashes/sec)
    
    while(1) {
        timerHash.start();          //start the timer
        SHA256 jack;                //instance of the class SHA256
        newKey_mutex.lock();        //lock the volatile variable newKey, other functions can't access it
        *key = newKey;              //copy the variale newKey into another variable
        newKey_mutex.unlock();      //unlock newKey, other functions can now access it
        jack.computeHash(hash, sequence, 64); //calculate hash for the input sequence
        counter=counter+1;          //increment the counter                 
        if(timerHash.read()> 1){    //roughly every second, calculate and print the hash rate 
            t=timerHash.read();
            hr=counter/t; 
            //pc.printf("Hash Rate: %f hash/second\n\r", hr);
            //pc.printf("Time: %f seconds\n\r", t); 
            putMessage(1, hr);
            timerHash.reset();      //reset the timer
            counter=0;              //reset the counter 
        };      
        if (hash[0]==0 && hash[1]==0 ){
            //pc.printf("%#016x\n",*nonce);
            putMessage(2, *nonce);
            //putMessage(4, *key);
        };    
        *nonce=*nonce+1;    //increment value of nonce
    };
    
}

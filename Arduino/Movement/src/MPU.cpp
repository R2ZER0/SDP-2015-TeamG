/* 
 * Controller for the robot's MPU6050
 * by Rikki "R2ZER0" Guy
 * 2015, Group 9, SDP
 */

#include <Wire.h>
#include <SerialCommand.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "MPU.h"

#define MPU_DEBUG (true)

// Local functions
static void cmd_MPU();
static bool Initialise();
static void ServiceDMP();
static bool WaitForStabilisation();
static bool Calibrate();

////////////////////////////////////////////////////////////////////////////////
// Variables
////////////////////////////////////////////////////////////////////////////////

#define MPU_ADDRESS (0x68)
#define STABLE_WINDOW (2*M_PI/360)
#define STABLE_MILLIS (5000)

#define DECIMAL_PLACES (3)

// command interface
static SerialCommand* comm;

// MPU6050 utility/wrapper class
static MPU6050 mpu(MPU_ADDRESS);

// MPU control/status vars
static bool dmpReady = false;  // set true if DMP init was successful
static uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
static uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
static uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
static uint16_t fifoCount;     // count of all bytes currently in FIFO
static uint8_t fifoBuffer[64]; // FIFO storage buffer

Quaternion q;
VectorFloat gravity;
float yawPitchRoll[3];
float yawOffset = 0.0f;

////////////////////////////////////////////////////////////////////////////////
// MPU Interrupt Handler
////////////////////////////////////////////////////////////////////////////////

static volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
static void dmpDataReady() {
    mpuInterrupt = true;
}

////////////////////////////////////////////////////////////////////////////////
// Setup/Service/Command Processing
////////////////////////////////////////////////////////////////////////////////

void MPU_setup(SerialCommand* _comm) {
    // I2C should already be setup
    // Wire.begin();
    
    // Add the MPU command
    comm = _comm;
    comm->addCommand("MPU", cmd_MPU);
}

void MPU_service() {
    // We do not need to service the comm, as that is already done
    
    ServiceDMP();
}

static void cmd_MPU() {
    char* subcmd = NULL;
    subcmd = comm->next();
    
    bool success = false;
    
    if(subcmd == NULL) { 
        /* do nothing */
        
    } else if(strcmp(subcmd, "GETYAW") == 0) {        
        if(dmpReady) {            
            Serial.println(yawOffset + yawPitchRoll[0], DECIMAL_PLACES);
            success = true;
        }
        
    } else if(strcmp(subcmd, "INIT") == 0) {
        success = Initialise(); //&& WaitForStabilisation();
        
    } else if(strcmp(subcmd, "STABL") == 0) {
        success = WaitForStabilisation();
        
    } else if(strcmp(subcmd, "CALIB") == 0) {
        success = Calibrate();        
        
    } else if(strcmp(subcmd, "HOME") == 0) {
        success = WaitForStabilisation();
        if(success) {
            yawOffset = -yawPitchRoll[0];
            Serial.print("Set offset of ");
            Serial.println(yawOffset, DECIMAL_PLACES);
        }
    }
    
    Serial.println(success ? "DONE" : "FAILED");
}

////////////////////////////////////////////////////////////////////////////////
// Initialise the DMP
////////////////////////////////////////////////////////////////////////////////

#define ASSERT(what) if(!( what )) { return false; }

static bool Initialise() {
    
    // Initialise MPU
#ifdef MPU_DEBUG
    Serial.println("Initialising MPU");
#endif
    mpu.initialize();
    
    // Test MPU connection
#ifdef MPU_DEBUG
    Serial.println("Testing MPU connection");
#endif
    ASSERT( mpu.testConnection() );
    
    // Initialise DMP
#ifdef MPU_DEBUG
    Serial.println("Initialising DMP");
#endif
    devStatus = mpu.dmpInitialize();
    ASSERT( devStatus == 0 );
    
    // Set full scale range (250, 500, 1000, 2000)
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_1000);
    
    // Calibration offsets
    // TODO: dynamic calibration
    /*
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788);
    */
    
    // Enable DMP
#ifdef MPU_DEBUG
    Serial.println("Enabling DMP");
#endif
    mpu.setDMPEnabled(true);

    // Enable Arduino interrupt detection
#ifdef MPU_DEBUG
    Serial.println("Attaching MPU interrupt");
#endif
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // Enable servicing of values
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
    
    return true;
}

#undef ASSERT

////////////////////////////////////////////////////////////////////////////////
// Service the DMP FIFO/Calculate YPR
////////////////////////////////////////////////////////////////////////////////

static void ServiceDMP() {
    // Is the DMP enabled?
    if(!dmpReady) { return; }
    
    // De we currently have data to process?
    while(mpuInterrupt || fifoCount > packetSize) {
    
        // We have data, time to get it
        mpuInterrupt = false;
        mpuIntStatus = mpu.getIntStatus();
        fifoCount = mpu.getFIFOCount();
        
        // check for overflow (this should never happen unless our code is too inefficient)
        if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
            // reset so we can continue cleanly
            mpu.resetFIFO();

        // otherwise, check for DMP data ready interrupt (this should happen frequently)
        } else if (mpuIntStatus & 0x02) {
            // wait for correct available data length, should be a VERY short wait
            while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

            // read a packet from FIFO
            mpu.getFIFOBytes(fifoBuffer, packetSize);
            
            // track FIFO count here in case there is > 1 packet available
            // (this lets us immediately read more without waiting for an interrupt)
            fifoCount -= packetSize;
            
            // Calculate yaw/pitch/roll 
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(yawPitchRoll, &q, &gravity);
        }
    
    }
}

////////////////////////////////////////////////////////////////////////////////
// Wait for the Gyro to Stabilise (10 seconds or so)
////////////////////////////////////////////////////////////////////////////////

static bool WaitForStabilisation() {
    if(!dmpReady) { return false; }
    
#ifdef MPU_DEBUG
    Serial.print("Stabilising: window=+-");
    Serial.print((float)STABLE_WINDOW, 4);
    Serial.print(" millis=");
    Serial.println(STABLE_MILLIS);
#endif
    
    // Wait for the gyro values to become stable
    float stableYPR[3];
    unsigned long stableTime[3];
    bool isStable[3];
    
    stableYPR[0] = yawPitchRoll[0];
    stableYPR[1] = yawPitchRoll[1];
    stableYPR[2] = yawPitchRoll[2];
    stableTime[0] = millis();
    stableTime[1] = millis();
    stableTime[2] = millis();
    isStable[0] = false;
    isStable[1] = false;
    isStable[2] = false;    
    
    while(true) {
        if(isStable[0] && isStable[1] && isStable[2]) { break; }
        
        for(byte i = 0; i < 3; ++i) {
            if( (yawPitchRoll[i] < (stableYPR[i] - STABLE_WINDOW)) ||
                (yawPitchRoll[i] > (stableYPR[i] + STABLE_WINDOW))) {
                stableTime[i] = millis();
                stableYPR[i] = yawPitchRoll[i];
            
            } else if(stableTime[i] + STABLE_MILLIS < millis()) {
                isStable[i] = true;                
            }
        }
        
        ServiceDMP();
    }
    
    return true;
}

////////////////////////////////////////////////////////////////////////////////
// Calibration - TODO
////////////////////////////////////////////////////////////////////////////////

static bool Calibrate() {
    return false;
}
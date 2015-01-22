#include <SerialCommand.h>

const int PIN_LED   = 13;
const int PIN_RADIO = 8;

SerialCommand comm;

void setup()
{
  // Enable setting the LED
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, HIGH);
  
  // Select the radio for serial
  pinMode(PIN_RADIO, OUTPUT);    
  digitalWrite(PIN_RADIO, HIGH);
  
  // Initialise serial-over-RF
  Serial.begin(115200);
  
  // Initialse the command messenger
  comm.addCommand("PING", cmd_PING);
  comm.addCommand("RUN", cmd_RUN);
}

void cmd_PING()
{
  Serial.println("PONG");
  digitalWrite(PIN_LED, LOW);
}

void cmd_RUN()
{
  double motor1 = 0.0;
  double motor2 = 0.0;
  double motor3 = 0.0;
  char* arg;
  
  arg = comm.next();
  if(arg != NULL) {
    motor1 = atof(arg);
  }
  
  arg = comm.next();
  if(arg != NULL) {
    motor2 = atof(arg);
  }
  
  arg = comm.next();
  if(arg != NULL) {
    motor3 = atof(arg);
  }
  
  doRun();
}

void loop()
{
  comm.readSerial();
}


void doRun(double motor1, double motor2, double motor3)
{
  // TODO
}



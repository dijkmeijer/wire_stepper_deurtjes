#include <EEPROM.h>
#include <Wire.h>
#include <AccelStepper.h>

// rotowand

const int LONGSIZE = sizeof(long int);

// Define a stepper and the pins it will use
AccelStepper stepper(AccelStepper::DRIVER,10, 11); // Defaults to AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5

#define WireAdress 3
#define MaxSpeedAdress 0
#define MaxAccelerationAdress 4
#define Positie_1_Adress 8
#define Positie_2_Adress 12
 
#define SensorPin1 8
#define SensorPin2 9

unsigned long MaxSpeed = 5000;
unsigned long  MaxAcceleration = 8000;
unsigned long Position_1;
unsigned long Position_2;
byte Position;
unsigned long tmp;

char buffer[10];
String value;
long partPosition;

void setup()
{  

  // Change these to suit your stepper if you want
  Serial.begin(9600); 
  MaxSpeed = read_long_Eeprom(MaxSpeedAdress);
  if(MaxSpeed < 10) MaxSpeed = 4000;
  MaxAcceleration = read_long_Eeprom(MaxAccelerationAdress);
  if(MaxAcceleration < 10) MaxAcceleration = 8000;
  Position_1 = read_long_Eeprom(Positie_1_Adress);
  Position_2 = read_long_Eeprom(Positie_2_Adress);
  
  stepper.setMaxSpeed(MaxSpeed);
  stepper.setAcceleration(MaxAcceleration);
  pinMode(SensorPin1, INPUT_PULLUP);
  pinMode(SensorPin2, INPUT_PULLUP);
  Wire.begin(WireAdress);               
  Wire.onReceive(receiveEvent); // register event
  stepper.setPinsInverted(true, false, false);
  Serial.print("start");
  stepper.setCurrentPosition(0);
  stepper.moveTo(-60000);
}

void loop()
{
  if(((PINB & 0x03) == 0x03)) 
  
 // Serial.println(PINB & 0x03);
    stepper.run();
  else {
    stepper.setCurrentPosition(0);
    stepper.moveTo(10);
    stepper.run();
  }
    
}



void receiveEvent(int howMany)
{
  int i;
  for(i=0;i<4;i++)
    buffer[i]=0;  
  i=0;
  tmp=0;
  while(1 < Wire.available()) // loop through all but the last
  {
    buffer[i++] = Wire.read(); // receive byte as a character
  }
  memcpy(&tmp, buffer, 4);
  Serial.println(tmp);
  Position = Wire.read();
//  Serial.print(Position);
  
  if(Position != 0)
    parseCommand();

}


void write_long_Eeprom(int addr, long val){
  EEPROM.write(addr, (byte )((val >> 24) & 0xff));
  EEPROM.write(addr+1, (byte )((val >> 16) & 0xff));
  EEPROM.write(addr+2, (byte )((val >> 8) & 0xff));
  EEPROM.write(addr+3, (byte )((val) & 0xff));
}


long read_long_Eeprom(int addr){
  long val;
  val = EEPROM.read(addr);
  val = (val << 8) | EEPROM.read(addr+1);
  val = (val << 8) | EEPROM.read(addr+2);
  val = (val << 8) | EEPROM.read(addr+3);
  return val;
}

void parseCommand(){ 

  switch(Position) {
  case 0: 
    break;
  case 1: 
    stepper.moveTo(Position_1);
    Serial.print("Ga naar Positie 1: ");
    Serial.println(Position_1);
    break;
  case 2: 
    stepper.moveTo(Position_2);
    Serial.print("Ga naar Positie 2: ");
    Serial.println(Position_2);
    break;
  case 3: 
    Serial.println("Reset");
    stepper.moveTo(-6000000);    
    break;
  case 4:
    MaxSpeed=tmp;
    Serial.print("Maxspeed = ");
    Serial.println(MaxSpeed);
    stepper.setMaxSpeed(MaxSpeed);
    write_long_Eeprom(MaxSpeedAdress, MaxSpeed);
    break;
  case 5:

    MaxAcceleration=tmp;
    Serial.print("MaxAcceleration = ");
    Serial.println(MaxAcceleration);
    stepper.setAcceleration(MaxAcceleration);
    write_long_Eeprom(MaxAccelerationAdress, MaxAcceleration);
    break;
  case 6:
    Position_1=tmp;
    Serial.print("Position_1 = ");
    Serial.println(Position_1);

    write_long_Eeprom(Positie_1_Adress, Position_1);
    break;
  case 7:
    Position_2=tmp;
    Serial.print("Position_2 = ");
    Serial.println(Position_2);
    write_long_Eeprom(Positie_2_Adress, Position_2);
    break;
  case 8:
    Serial.print("Goto position = ");
    partPosition = ((Position_2 - Position_1)*tmp/1000)+Position_1;
    Serial.println(partPosition);
    stepper.moveTo(partPosition);
    break;
  }
}










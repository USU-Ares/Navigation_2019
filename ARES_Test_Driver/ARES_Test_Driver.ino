/* This program drives the test-bench robot
 * for ARES team navigation testing 
 * 
 * Author: Derek S Workman
 * Email: derek.workman@aggiemail.usu.edu
 */

//Definitions
#define MOTOR_L_CONT 6   //Left motor control at pin 6 (For PWM)
#define MOTOR_L_STAT 9   //Left motor status at pin 9 (HIGH = forward)
#define MOTOR_R_CONT 11  //Right motor control at pin 11 (For PWM)
#define MOTOR_R_STAT 10  //Right motor status at pin 10 (HIGH = forward)

#define DIR 0 //index for motor direction byte in serial data
#define ML  1 //index for left motor PWM value in serial data
#define MR  2 //index for right motor PWM value in serial data

#define DIR_LEFT 1  //bit selection for left motor direction
#define DIR_RIGHT 2 //bit selection for right motor direction

//Global Variables
char pid_stat = 0;  //contains status for serial Packet ID
const size_t N = 3; //The data index size [direction_bits, PWM_L, PWM_R]
byte data[N] = {0}; //the data for the motors (PWM values)
byte crc = 0; //Cyclic redundancy check, (will be a sum of all bits in the data portion of the packet)

bool ChecksumMatch(byte* buf, byte crc, size_t n) {

  uint8_t bitCount = 0;
  
  for(uint8_t i = 0; i < n; i++) {
    bitCount += buf[i]&0x01;  //count the first bit if set
    for(uint8_t j = 1; j < 8; j++) {
      bitCount += (buf[i] >> j)&0x01; //count the rest of the set bits
    }
  }
  //Serial.print("bitCount: "); //print statements for debug
  //Serial.println(bitCount);
  //Serial.print("crc: ");
  //Serial.println(crc);
  if(bitCount == crc) return true;
  else return false;
}

void ReadSerial() {

  byte buf[N] = {0};
  
  if(Serial.available()) {
    switch(pid_stat) {
      case 0:
        if(Serial.read() == 'G') {
          pid_stat = 'G';
        }
        break;
      case 'G':
        if(Serial.read() == 'O') {
          pid_stat = 'O';
        }
        break;
      case 'O':
        if(Serial.available() == (N+1)) {
          pid_stat = 0;
          for(size_t i = 0; i < N; i++) {
            buf[i] = Serial.read();     //read in all N data bytes
          }
          crc = Serial.read();  //get the checksum at the end
          if(ChecksumMatch(buf,crc,N)) {
            for(size_t i = 0; i < N; i++) {
              data[i] = buf[i]; //If the checksum matched, then transfer
                                //  data from the buffer to the registers
            }
          }
        }
        break;
      default: pid_stat = 0;  //In case of default, reset pid_stat
    }
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); //Serial port has baudrate of 9600
  
  pinMode(MOTOR_L_CONT, OUTPUT); //Set motor pins as outputs
  pinMode(MOTOR_L_STAT, OUTPUT);
  pinMode(MOTOR_R_CONT, OUTPUT);
  pinMode(MOTOR_R_STAT, OUTPUT);

}

void loop() {
  // put your main code here, to run repeatedly:
  
  ReadSerial();

  //Perform motor opperations
  if(data[DIR]&DIR_LEFT == DIR_LEFT) {  //Update the direction for each motor
    digitalWrite(MOTOR_L_STAT, HIGH);
  }else digitalWrite(MOTOR_L_STAT, LOW);
  if(data[DIR]&DIR_RIGHT == DIR_RIGHT) {
    digitalWrite(MOTOR_R_STAT, HIGH);
  }else digitalWrite(MOTOR_R_STAT, LOW);
  analogWrite(MOTOR_L_CONT, data[ML]);  //Update the PWM outputs
  analogWrite(MOTOR_R_CONT, data[MR]);
}

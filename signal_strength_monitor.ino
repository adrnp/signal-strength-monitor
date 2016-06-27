#include "types.h"

// --------------------------------- //
//
// some necessary constants
//
// --------------------------------- //

// 880 MHz
//const float SLOPE = 19.0;   // mV/dB
//const float X_INT = -88.8;    // dBm

// 2140 MHz (~ wifi)
const float SLOPE = 17.7;   // mV/dB
const float X_INT = -89.0;  // dBm

// loop rate
const float FREQ = 10;          // [Hz]

// calculated constants
const float B = -SLOPE * X_INT;
const float TIMEOUT = 1.0/FREQ * 1000.0;

// for sending the message
const byte SYNC_1 = 0xA0;
const byte SYNC_2 = 0xB1;

// --------------------------------- //
//
// globals
//
// --------------------------------- //

unsigned long curTime;
unsigned long lastTime;
int sensorValue;
float voltage;
float dir_dBm;
float omni_dBm;

int enablePinDir = 7;
int enablePinOmni = 2;
int ledPin = 13;


// --------------------------------- //
//
// setup
//
// --------------------------------- //

void setup() {
  Serial.begin(115200);
  lastTime = millis();
  
  // set up some pins and lighting
  pinMode(enablePinDir, OUTPUT);
  pinMode(enablePinOmni, OUTPUT);
  pinMode(ledPin, OUTPUT);
  
  // delay on turning on the enable pin, just to make sure everything going smoothly
  // the led is going to be used as an indicator to know when the chip has been enabled
  digitalWrite(enablePinDir, LOW);
  digitalWrite(enablePinOmni, LOW);
  digitalWrite(ledPin, LOW);
  delay(3000);
  digitalWrite(enablePinDir, HIGH);
  digitalWrite(enablePinOmni, HIGH);
  digitalWrite(ledPin, HIGH);
}


// --------------------------------- //
//
// main loop
//
// --------------------------------- //

void loop() {
  
  curTime = millis(); // get the current timestamp

  // rate limit
  if (curTime - lastTime >= TIMEOUT) {
    float rate = 1000.0/(curTime - lastTime);
    Serial.print("loop rate:\t");
    Serial.println(rate);
    lastTime = curTime;
  
    // read in for directional antenna - A0
    sensorValue = analogRead(A0);
    voltage = sensorValue * (5.0/1023.0) * 1000.0; // mV
    
    // convert voltage to dBm
    dir_dBm = (voltage - B)/SLOPE;
    
    // read in for omni directional antenna - A5
    sensorValue = analogRead(A5);
    voltage = sensorValue * (5.0/1023.0) * 1000.0; // mV
    
    // convert voltage to dBm
    omni_dBm = (voltage - B)/SLOPE;
    
    // do some printing out
    /*
    Serial.print("mV:\t");
    Serial.print(voltage);
    Serial.print("\t\tdbm:\t");
    Serial.println(dir_dBm);
    Serial.print("\t\t omni dbm:\t");    
    Serial.println(omni_dBm);
    */
    sendData(millis(), dir_dBm, omni_dBm);
  }
}

// --------------------------------- //
//
// helper functions
//
// --------------------------------- //


/* send bitstream out the serial port */
void sendData(unsigned long timestamp, float dir, float omni) {
  
  // do some scaling to be able to send as ints
  int dirScaled = dir*100;
  int omniScaled = omni*100;
  
  msg_buf_t msg;
  msg.signalStrength.timestamp = millis();
  msg.signalStrength.dir = dirScaled;
  msg.signalStrength.omni = omniScaled;
  
  // calculate the checksum
  checksum_t checksum = stupidChecksum(msg.buf, sizeof(msg));
  
  // put it into a stupid buffer to be able to be sent out
  chk_buf_t chk;
  chk.checksum = checksum;
  
  // send the data over the serial port
  // sync bytes first
  Serial.write(SYNC_1);
  Serial.write(SYNC_2);
  
  // the actual data
  Serial.write(msg.buf, sizeof(msg));
  
  // the checksum
  Serial.write(chk.buf, 2);
  
  return;
}


/* calculate the Fletcher checksum for a buffer of data */
checksum_t fletcher16( uint8_t const *data, int bytes )
{
  unsigned int sum1 = 0xff, sum2 = 0xff;
  int tlen;
   
  while (bytes) {
    tlen = bytes >= 20 ? 20 : bytes;
    bytes -= tlen;
    do {
            sum2 += sum1 += *data++;
    } while (--tlen);
    sum1 = (sum1 & 0xff) + (sum1 >> 8);
    sum2 = (sum2 & 0xff) + (sum2 >> 8);
  }
  /* Second reduction step to reduce sums to 8 bits */
  sum1 = (sum1 & 0xff) + (sum1 >> 8);
  sum2 = (sum2 & 0xff) + (sum2 >> 8);
  
  checksum_t checksum;
  checksum.a = sum1;
  checksum.b = sum2;
  return checksum;
}

/* my really dumb checksum, but easy to implement and it works */
checksum_t stupidChecksum( uint8_t *data, int length )
{
  uint8_t a = 0;
  uint8_t b = 0;
  
  for (int i = 0; i < length; i++) {
    a += *data++;
    b += a;
  }
  
  checksum_t checksum;
  checksum.a = a;//(a >> 8);
  checksum.b = b;//(b >> 8);
  return checksum;
}



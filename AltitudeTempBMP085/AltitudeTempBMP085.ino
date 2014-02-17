#include <LiquidCrystal.h>

// Author: Grmis, 04/2013
// Arduino Pro Mini 3.3v connected to BMP085 barometer breackout
// throught I2C
// Arduino connected to 'RX' line of FrSky D8R-2 receiver
// through digital Pin (use SoftSerial with inversion)
// Although the arduino is 3.3v, it seems that the 5v receiver understands it :-)

// code based on the original codes by:
// Rolf R Bakke
// R.Schlohan
// BMP085 Extended Example Code by: Jim Lindblom (SparkFun Electronics)


#include <Wire.h> // I2C to communiucate with the sensor
#include <SoftwareSerial.h> // To communicate with the FrSky RX
#include <Time.h> 

#define BMP085_ADDRESS 0x77  // I2C address of BMP085

const unsigned char OSS = 3;  // Oversampling Setting 3 => max precision
// Calibration values
int ac1;
int ac2; 
int ac3; 
unsigned int ac4;
unsigned int ac5;
unsigned int ac6;
int b1; 
int b2;
int mb;
int mc;
int md;

// b5 is calculated in bmp085GetTemperature(...), this variable is also used in bmp085GetPressure(...
// so ...Temperature(...) must be called before ...Pressure(...).
long b5; 

short temperature; // Temperature, in 0.1 degrees.
short max_temp; // Maximum Temperature, in 0.1 degrees.
long pressure; // in Pa
float altitude;
//uint16_t max_alt=0;
const int AverageValueCount=10;         // the number of values that will be used to calculate a new average value
int count=0;
long pressure_values[AverageValueCount];// Aray to store the previous pressure values
const float p0 = 101325;     // Pressure at sea level (Pa)


#define PIN_SerialTX 2       // the pin to transmit the serial data to the frsky telemetry enabled receiver
#define PIN_Led 13
#define PIN_VoltageCell1 0    //  Pin for measuring Voltage of cell 1 ( Analog In Pin! )
#define PIN_VoltageCell2 1    //  Pin for measuring Voltage of cell 2 ( Analog In Pin! )
#define PIN_VoltageCell3 2    //  Pin for measuring Voltage of cell 3 ( Analog In Pin! )

// Software Serial is used including the Inverted Signal option ( the "true" in the line below )
// The corresponding pin has to be connected to the 'rx' pin of the receiver
SoftwareSerial mySerial(0, PIN_SerialTX,true);

#define FRSKY_USERDATA_GPS_ALT_B    0x01
#define FRSKY_USERDATA_TEMP1        0x02
#define FRSKY_USERDATA_RPM          0x03
#define FRSKY_USERDATA_FUEL         0x04
#define FRSKY_USERDATA_TEMP2        0x05
#define FRSKY_USERDATA_CELL_VOLT    0x06

#define FRSKY_USERDATA_GPS_ALT_A    0x09
#define FRSKY_USERDATA_BARO_ALT_B   0x10
#define FRSKY_USERDATA_GPS_SPEED_B  0x11
#define FRSKY_USERDATA_GPS_LONG_B   0x12
#define FRSKY_USERDATA_GPS_LAT_B    0x13
#define FRSKY_USERDATA_GPS_CURSE_B  0x14
#define FRSKY_USERDATA_GPS_DM       0x15
#define FRSKY_USERDATA_GPS_YEAR     0x16
#define FRSKY_USERDATA_GPS_HM       0x17
#define FRSKY_USERDATA_GPS_SEC      0x18
#define FRSKY_USERDATA_GPS_SPEED_A  0x19
#define FRSKY_USERDATA_GPS_LONG_A   0x1A
#define FRSKY_USERDATA_GPS_LAT_A    0x1B
#define FRSKY_USERDATA_GPS_CURSE_A  0x1C

#define FRSKY_USERDATA_BARO_ALT_A   0x21
#define FRSKY_USERDATA_GPS_LONG_EW  0x22
#define FRSKY_USERDATA_GPS_LAT_EW   0x23
#define FRSKY_USERDATA_ACC_X        0x24
#define FRSKY_USERDATA_ACC_Y        0x25
#define FRSKY_USERDATA_ACC_Z        0x26

#define FRSKY_USERDATA_CURRENT      0x28

#define FRSKY_USERDATA_VOLTAGE_B    0x3A
#define FRSKY_USERDATA_VOLTAGE_A    0x3B

unsigned long lastMillisFrame1,lastMillisFrame2,lastMillisFrame3,startMillis;

//un-comment the line below if you want some messages to be transmitted on the serial line (to console/PC)
#define DEBUG

void setup() {
  Wire.begin();
#ifdef DEBUG
  Serial.begin(9600);
#endif

  // set the data rate for the SoftwareSerial port
  // this is the speed of the UART of the FrSky  receiver
  mySerial.begin(9600);

// Read the (factory) calibration data of the pressure sensor
  bmp085Calibration();
  for (int i=0;i<AverageValueCount;i++) pressure_values[i]=p0;     
  max_temp=-999;
  //SendValue(FRSKY_USERDATA_VOLTAGE_B,0);
  //SendValue(FRSKY_USERDATA_VOLTAGE_A,0);
  //SendValue(FRSKY_USERDATA_CURRENT,3);
}

short n=0;
void loop() {
   altitude=getAltitude();
#ifdef DEBUG
  n++;
#endif
  // Frame to send every 300ms
  if( (lastMillisFrame1 + 300) <=millis()) {
  ledOn();
  SendAlt(altitude);
    lastMillisFrame1=millis();
   ledOff();  


  // Frame to send every 2s
  if( (lastMillisFrame2 + 2000) <=millis()) {
    //SendCellVoltage(0,ReadVoltage(PIN_VoltageCell1));
    //SendCellVoltage(1,ReadVoltage(PIN_VoltageCell2));
    //SendCellVoltage(2,ReadVoltage(PIN_VoltageCell3));
    SendSec();
#ifdef DEBUG
    Serial.print("Temperature: ");
    Serial.print(temperature, DEC);
    Serial.println(" *0.1 deg C");
    Serial.print("Pressure: ");
    Serial.print(pressure, DEC);
    Serial.println(" Pa");

    Serial.print("Altitude: ");
    Serial.print(altitude, 2);
    Serial.println(" m");
    Serial.println((millis()-lastMillisFrame2)/n);
    Serial.println(n);n=0;
    Serial.println();
#endif
    lastMillisFrame2=millis();

  // Frame to send every 5s
  if( (lastMillisFrame3 + 5000) <=millis()) {
    SendTemperature(temperature/10); 
    //SendValue(FRSKY_USERDATA_GPS_ALT_B,max_alt);
    SendHourMinutes();
    lastMillisFrame3=millis();
    
  }
  }
  }

}

void SendValue(uint8_t ID, uint16_t Value) {
  uint8_t tmp1 = Value & 0x00ff;
  uint8_t tmp2 = (Value & 0xff00)>>8;
  mySerial.write(0x5E);
  mySerial.write(ID);
  if(tmp1 == 0x5E) {
    mySerial.write(0x5D);
    mySerial.write(0x3E);
  }
  else if(tmp1 == 0x5D) {
    mySerial.write(0x5D);
    mySerial.write(0x3D);
  }
  else {
    mySerial.write(tmp1);
  }
  if(tmp2 == 0x5E) {
    mySerial.write(0x5D);
    mySerial.write(0x3E);
  }
  else if(tmp2 == 0x5D) {
    mySerial.write(0x5D);
    mySerial.write(0x3D);
  }
  else {
    mySerial.write(tmp2);
  }
  mySerial.write(0x5E);
}
uint16_t ReadVoltage(int pin)
{
  // Convert the analog reading (which goes from 0 - 1023) to a millivolt value
  return uint16_t(
  //   (float) analogRead(pin) * (5.0 / 1023.0) *1000
  (float) analogRead(pin) * (float)(readVccMv() /1023.0)
    );

}
void SendCellVoltage(uint8_t cellID, uint16_t voltage) {
  voltage /= 2;
  uint8_t v1 = (voltage & 0x0f00)>>8 | (cellID<<4 & 0xf0);
  uint8_t v2 = (voltage & 0x00ff);
  uint16_t Value = (v1 & 0x00ff) | (v2<<8);
  SendValue(FRSKY_USERDATA_CELL_VOLT, Value);
}

void SendTemperature(int16_t tempc) {
  SendValue(FRSKY_USERDATA_TEMP1, tempc);
  if (tempc>max_temp) max_temp=tempc;
  SendValue(FRSKY_USERDATA_TEMP2, max_temp);
}

void SendAlt(float alt)
{
  int16_t Meter = int16_t(alt);
  uint16_t Centimeter =  uint16_t((abs(alt) -abs(Meter)) *100);
  SendValue(FRSKY_USERDATA_BARO_ALT_B, Meter);
  SendValue(FRSKY_USERDATA_BARO_ALT_A, Centimeter);
}

void SendSec() {
  ///SendValue(FRSKY_USERDATA_GPS_SEC, second());
}
void SendHourMinutes() {
  ///const uint16_t hm=  ( ( minute()& 0x00ff)<<8 )+ (hour() & 0x00ff);
  ///SendValue(FRSKY_USERDATA_GPS_HM,hm);
}

float getAltitude() {
  temperature=bmp085GetTemperature(bmp085ReadUT());
  pressure=bmp085GetPressure(bmp085ReadUP());
  pressure_values[count++]=pressure;
  if (count>=AverageValueCount) count=0;

  float average_pressure=0;
  for (int i=0;i<AverageValueCount;i++) average_pressure+=(float) pressure_values[i];
  average_pressure/=p0*AverageValueCount;
  return ((float)44330 * (1 - pow(average_pressure, 0.190295)));
}

void ledOn() {
  digitalWrite(PIN_Led,1);
}


void ledOff() {
  digitalWrite(PIN_Led,0);
}

/* readVcc - Read the real internal supply voltageof the arduino
 
 Improving Accuracy
 While the large tolerance of the internal 1.1 volt reference greatly limits the accuracy
 of this measurement, for individual projects we can compensate for greater accuracy.
 To do so, simply measure your Vcc with a voltmeter and with our readVcc() function.
 Then, replace the constant 1125300L with a new constant:
 
 scale_constant = internal1.1Ref * 1023 * 1000
 
 where
 
 internal1.1Ref = 1.1 * Vcc1 (per voltmeter) / Vcc2 (per readVcc() function)
 
 This calibrated value will be good for the AVR chip measured only, and may be subject to
 temperature variation. Feel free to experiment with your own measurements.
 */
/*
     Meine Versorgungsspannung per Multimeter =4,87V
 1,1*4870 /5115 =1,047311827957
 */
//const long scaleConst = 1071.4 * 1000 ; // internalRef * 1023 * 1000;
//const long scaleConst = 1125.300 * 1000 ; // internalRef * 1023 * 1000;
const long scaleConst = 1156.300 * 1000 ; // internalRef * 1023 * 1000;
int readVccMv() {
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
  ADMUX = _BV(MUX5) | _BV(MUX0);
#elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
  ADMUX = _BV(MUX3) | _BV(MUX2);
#else
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#endif 

  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring

  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH 
  uint8_t high = ADCH; // unlocks both

  long result = (high<<8) | low;

  //result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  result = scaleConst / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return (int)result; // Vcc in millivolts
}

void bmp085Calibration()
{
  ac1 = bmp085ReadInt(0xAA);
  ac2 = bmp085ReadInt(0xAC);
  ac3 = bmp085ReadInt(0xAE);
  ac4 = bmp085ReadInt(0xB0);
  ac5 = bmp085ReadInt(0xB2);
  ac6 = bmp085ReadInt(0xB4);
  b1 = bmp085ReadInt(0xB6);
  b2 = bmp085ReadInt(0xB8);
  mb = bmp085ReadInt(0xBA);
  mc = bmp085ReadInt(0xBC);
  md = bmp085ReadInt(0xBE);
}

// Calculate temperature given ut.
// Value returned will be in units of 0.1 deg C
short bmp085GetTemperature(unsigned int ut) {
  long x1, x2;
  x1 = (((long)ut - (long)ac6)*(long)ac5) >> 15;
  x2 = ((long)mc << 11)/(x1 + md);
  b5 = x1 + x2;
  return ((b5 + 8)>>4);  
}

// Calculate pressure given up
// calibration values must be known
// b5 is also required so bmp085GetTemperature(...) must be called first.
// Value returned will be pressure in units of Pa.
long bmp085GetPressure(unsigned long up)
{
  long x1, x2, x3, b3, b6, p;
  unsigned long b4, b7;

  b6 = b5 - 4000;
  // Calculate B3
  x1 = (b2 * (b6 * b6)>>12)>>11;
  x2 = (ac2 * b6)>>11;
  x3 = x1 + x2;
  b3 = (((int32_t) ac1 * 4 + x3)<<OSS) >> 2; 

  // Calculate B4
  x1 = (ac3 * b6)>>13;
  x2 = (b1 * ((b6 * b6)>>12))>>16;
  x3 = ((x1 + x2) + 2)>>2;
  b4 = (ac4 * (unsigned long)(x3 + 32768))>>15;

  b7 = ((unsigned long)(up - b3) * (50000>>OSS));
  if (b7 < 0x80000000)
    p = (b7<<1)/b4;
  else
    p = (b7/b4)<<1;

  x1 = (p>>8) * (p>>8);
  x1 = (x1 * 3038)>>16;
  x2 = (-7357 * p)>>16;
  p += (x1 + x2 + 3791)>>4;

  return p;
}

// Read 1 byte from the BMP085 at 'address'
char bmp085Read(unsigned char address) {
  unsigned char data;
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(address);
  Wire.endTransmission();
  Wire.requestFrom(BMP085_ADDRESS, 1);
  while(!Wire.available());
  return Wire.read();
}

// Read 2 bytes from the BMP085
// First byte will be from 'address'
// Second byte will be from 'address'+1
int bmp085ReadInt(unsigned char address) {
  unsigned char msb, lsb;
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(address);
  Wire.endTransmission();
  Wire.requestFrom(BMP085_ADDRESS, 2);
  while(Wire.available()<2);
  msb = Wire.read();
  lsb = Wire.read();
  return (int) msb<<8 | lsb;
}

// Read the uncompensated temperature value
unsigned int bmp085ReadUT() {
  unsigned int ut;

  // Write 0x2E into Register 0xF4
  // This requests a temperature reading
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(0xF4);
  Wire.write(0x2E);
  Wire.endTransmission();

  // Wait at least 4.5ms
  delay(5);
  //delay(10);
  //delay(8);
  
  // Read two bytes from registers 0xF6 and 0xF7
  ut = bmp085ReadInt(0xF6);
  return ut;
}

// Read the uncompensated pressure value
unsigned long bmp085ReadUP()
{
  unsigned char msb, lsb, xlsb;
  unsigned long up = 0;

  // Write 0x34+(<<6) into register 0xF4
  // Request a pressure reading w/ oversampling setting
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(0xF4);
  Wire.write(0x34 + (OSS<<6));
  Wire.endTransmission();

  // Wait for conversion, delay time dependent on OSS
  delay(2 + (3<<OSS));
  //delay(4 + (6<<OSS));// For my 8MHz Arduino Pro mini ?

  // Read register 0xF6 (MSB), 0xF7 (LSB), and 0xF8 (XLSB)
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(0xF6);
  Wire.endTransmission();
  Wire.requestFrom(BMP085_ADDRESS, 3);

  // Wait for data to become available
  while(Wire.available() < 3)
    ;
  msb = Wire.read();
  lsb = Wire.read();
  xlsb = Wire.read();

  up = (((unsigned long) msb << 16) | ((unsigned long) lsb << 8) | (unsigned long) xlsb) >> (8-OSS);

  return up;
}

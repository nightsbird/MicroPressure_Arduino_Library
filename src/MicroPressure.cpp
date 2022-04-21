/*
  This is a library for the Honeywell MicroPressure Sensor, which can read from 0 to 300mmHg.
  By: Jeffrey Dunbar
  Date: April 2022 
  License: This code is public domain but you buy me a beer if you use this and 
  we meet someday (Beerware license).
  Feel like supporting our work? Buy a board from SparkFun!
 */
 
 #include "MicroPressure.h"

/* Constructor and sets default values.
   - (Optional) eoc_pin, End of Conversion indicator. Default: -1 (skip)
   - (Optional) rst_pin, Reset pin for MPR sensor. Default: -1 (skip)
   - minimumPSI, minimum range value of the sensor (in PSI). Default: 0
   - maximumPSI, maximum range value of the sensor (in pSI). Default: 25
   - deviceTYPE 1 = OUTPUT_MIN, minimum range calibration (hex of 10 percent of 2^24). Default:  0x19999A;OUTPUT_MAX, maximum range calibration (hex of 90 percent of 2^24). Default:  0xE66666
   - deviceTYPE 2 = OUTPUT_MIN, minimum range calibration (hex of 2.5 percent of 2^24). Default:  0x66666;OUTPUT_MAX, maximum range calibration (hex of 22.5 percent of 2^24). Default:  0x39999A
   - deviceTYPE 3 = OUTPUT_MIN, minimum range calibration (hex of 20 percent of 2^24). Default:  0x333333;OUTPUT_MAX, maximum range calibration (hex of 80 percent of 2^24). Default:  0xCCCCCD
*/
MicroPressure::MicroPressure(int8_t eoc_pin, int8_t rst_pin, uint8_t minimumPSI, uint8_t maximumPSI, uint8_t deviceADDRESS, unit8_t deviceTYPE)
{
  _eoc = eoc_pin;
  _rst = rst_pin;
  _minPsi = minimumPSI;
  _maxPsi = maximumPSI;
  _devADDr = deviceADDRESS;
  _devTyp = deviceTYPE;	
}

/* Initialize hardware
  - deviceAddress, I2C address of the sensor. Default: 0x18
  - wirePort, sets the I2C bus used for communication. Default: Wire
  
  - Returns 0/1: 0: sensor not found, 1: sensor connected
*/
bool MicroPressure::begin(uint8_t deviceAddress, TwoWire &wirePort)
{
  _address = deviceAddress;
  _i2cPort = &wirePort;
  
  if(_eoc != -1)
  {
    pinMode(_eoc, INPUT);
  }
  
  if(_rst != -1)
  {
    pinMode(_rst, OUTPUT);
	digitalWrite(_rst,LOW);
	delay(5);
	digitalWrite(_rst,HIGH);
	delay(5);
  }

  _i2cPort->beginTransmission(_address);

  uint8_t error = _i2cPort->endTransmission();

  if(error == 0) return true;
  else           return false;
}

/* Read the status byte of the sensor
  - Returns status byte
*/
uint8_t MicroPressure::readStatus(void)
{
  _i2cPort->requestFrom(_address,1);
  return _i2cPort->read();
}

/* Read the Pressure Sensor Reading
 - (optional) Pressure_Units, can return various pressure units. Default: mmHg
   Pressure Units available:
     - PSI: Pounds per Square Inch
	 - PA: Pascals
	 - KPA: Kilopascals
	 - TORR
	 - INHG: Inch of Mercury
	 - ATM: Atmospheres
	 - BAR
*/
float MicroPressure::readPressure(Pressure_Units units)
{
  _i2cPort->beginTransmission(_address);
  _i2cPort->write((uint8_t)0xAA);
  _i2cPort->write((uint8_t)0x00);
  _i2cPort->write((uint8_t)0x00);
  _i2cPort->endTransmission();
  
  // Wait for new pressure reading available
  if(_eoc != -1) // Use GPIO pin if defined
  {
    while(!digitalRead(_eoc))
	{
	  delay(1);
	}
  }
  else // Check status byte if GPIO is not defined
  {
    uint8_t status = readStatus();
    while((status&BUSY_FLAG) && (status!=0xFF))
    {
      delay(1);
      status = readStatus();
    }
  }
  
  _i2cPort->requestFrom(_address,4);
  
  uint8_t status = _i2cPort->read();
  
  //check memory integrity and math saturation bits
  if((status & INTEGRITY_FLAG) || (status & MATH_SAT_FLAG))
  {
    return NAN;
  }
  
  //read 24-bit pressure
  uint32_t reading = 0;
  for(uint8_t i=0;i<3;i++)
  {
    reading |= _i2cPort->read();
    if(i != 2) reading = reading<<8;
  }

  //convert from 24-bit to float psi value
  float pressure;
  pressure = (reading - OUTPUT_MIN) * (_maxPsi - _minPsi);
  pressure = (pressure / (OUTPUT_MAX - OUTPUT_MIN)) + _minPsi;

  if(units == PSI)       return pressure; //PSI
  else if(units == PA)   return pressure*6894.7573; //Pa (Pascal)
  else if(units == KPA)  return pressure*6.89476;   //kPa (kilopascal)
  else if(units == TORR) return pressure*51.7149;   //torr (mmHg)
  else if(units == INHG) return pressure*2.03602;   //inHg (inch of mercury)
  else if(units == ATM)  return pressure*0.06805;   //atm (atmosphere)
  else if(units == BAR)  return pressure*0.06895;   //bar
  else                   return pressure; //PSI
}

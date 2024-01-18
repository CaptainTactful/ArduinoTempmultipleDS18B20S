#include <OneWire.h>
#include <DallasTemperature.h>

// OneWire DS18S20, DS18B20, DS1822 Temperature Example
//
// http://www.pjrc.com/teensy/td_libs_OneWire.html
//
// The DallasTemperature library can do all this work for you!
// https://github.com/milesburton/Arduino-Temperature-Control-Library


#define ONE_WIRE_BUS 10

OneWire  ds(ONE_WIRE_BUS);  // on pin 10 (a 4.7K resistor is necessary)

DallasTemperature sensors(&ds);

int sensornumber = 0;

//romserials[20] = romserial1, romserial2, romserial3, romserial4, romserial5, romserial6, romserial7, romserial8, romserial9, romserial10, romserial11, romserial12, romserial13, romserial14, romserial15, romserial16, romserial17, romserial18, romserial19, romserial20;

void setup(void) {
  Serial.begin(9600);
      // Start up the library
  sensors.begin();
}

void loop(void) {
  // locate devices on the bus
  Serial.print("Locating devices...");
  Serial.print("Found ");
  Serial.print(sensors.getDeviceCount(), DEC);
  Serial.println(" devices.");
delay(500);
    Serial.print("Parasite power is: ");
  if (sensors.isParasitePowerMode()) Serial.println("ON");
  else Serial.println("OFF");
delay(500);

  byte i;
  byte present = 0;
  byte type_s;
  byte data[9];
  byte addr[8];
  int celsius;
  int celsiusstringlength;
//  int sensornumber = 0;
  
  String romserial;
  
  if ( !ds.search(addr)) {
    Serial.println("No more addresses.");
    Serial.print("Device count = ");
    Serial.print(sensornumber);
    Serial.println();
    ds.reset_search();
    sensornumber = 0;
    delay(400);
    return;
  }
  
 // Serial.print("ROM =");
  for( i = 0; i < 8; i++) {
    Serial.write(' ');
 //   Serial.print(addr[i], HEX);
  }

  if (OneWire::crc8(addr, 7) != addr[7]) {
  //    Serial.println("CRC is not valid!");
      return;
  }
 // Serial.println();
 
  // the first ROM byte indicates which chip
  switch (addr[0]) {
    case 0x10:
  //    Serial.println("  Chip = DS18S20");  // or old DS1820
      type_s = 1;
      break;
    case 0x28:
  //    Serial.println("  Chip = DS18B20");
      type_s = 0;
      break;
    case 0x22:
  //    Serial.println("  Chip = DS1822");
      type_s = 0;
      break;
    default:
  //    Serial.println("Device is not a DS18x20 family device.");
      return;
  } 

  ds.reset();
  ds.select(addr);
  ds.write(0x44, 1);        // start conversion, with parasite power on at the end
  
  delay(1200);     // maybe 750ms is enough, maybe not
  // we might do a ds.depower() here, but the reset will take care of it.
  
  present = ds.reset();
  ds.select(addr);    
  ds.write(0xBE);         // Read Scratchpad

  //Serial.print("  Data = ");
  //Serial.print(present, HEX);
  //Serial.print(" ");
  for ( i = 0; i < 9; i++) {           // we need 9 bytes
    data[i] = ds.read();
  //  Serial.print(data[i], HEX);
  //  Serial.print(" ");
  }
  //Serial.print(" CRC=");
  //Serial.print(OneWire::crc8(data, 8), HEX);
  //Serial.println();

  // Convert the data to actual temperature
  // because the result is a 16 bit signed integer, it should
  // be stored to an "int16_t" type, which is always 16 bits
  // even when compiled on a 32 bit processor.
  int16_t raw = (data[1] << 8) | data[0];
  if (type_s) {
    raw = raw << 3; // 9 bit resolution default
    if (data[7] == 0x10) {
      // "count remain" gives full 12 bit resolution
      raw = (raw & 0xFFF0) + 12 - data[6];
    }
  } else {
    byte cfg = (data[4] & 0x60);
    // at lower res, the low bits are undefined, so let's zero them
    if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
    else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
    else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
    //// default is 12 bit resolution, 750 ms conversion time
  }
  celsius = (raw / 16.0)*100;
romserial = String(addr[i], HEX);
  Serial.print("Temp of (");

if (sensornumber < 20) 
  {
    sensornumber++;
 //   Serial.print(raw);
  Serial.print(") sensor"); 
    char sensornumberpadded[5];
  sprintf(sensornumberpadded, "%02d", sensornumber);
  Serial.print(sensornumberpadded);
  Serial.print(" ");
  }
  Serial.print(addr[0], HEX);
    Serial.print(addr[1], HEX);
      Serial.print(addr[2], HEX);
        Serial.print(addr[3], HEX);
          Serial.print(addr[4], HEX);
            Serial.print(addr[5], HEX);
              Serial.print(addr[6], HEX);
                Serial.print(addr[7], HEX);
  Serial.print(" = ");
  char temppaddedoutput[5];
  sprintf(temppaddedoutput, "%04d", (celsius+3000));
  Serial.print(temppaddedoutput);
  Serial.println();
/*
for (uint8_t i = 0; i < sensornumber; i++) {
  DeviceAddress addr; //or use Addresses[i], if allocated properly
  sensors.getAddress(addr, i);
  printAddress(addr);
  Serial.println();
}

  delay(2000);
}
void printAddress(DeviceAddress deviceAddress)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    if (deviceAddress[i] < 16){ 
      Serial.print("+");
    }
    Serial.print(deviceAddress[i], HEX);
  //  Serial.println();
  //  Serial.print(deviceAddress[i], DEC);
 //   Serial.print("\t");
 //   Serial.print(deviceAddress[i], BIN);    
 //   Serial.print("\n");
  }
 // Serial.print();
 */
}

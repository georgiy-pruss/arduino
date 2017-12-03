// for chart

#include <OneWire.h>

OneWire  ds(2);  // on pin 2 (a 4.7K resistor is necessary)
#define NDEVS 10.0

void setup(void) { Serial.begin(9600); pinMode(13, OUTPUT); digitalWrite(13, LOW); }

float corr[10] = {-0.10856,-0.10318,0.028015,0.066306,0.064143,0.062871,0.14363,-0.10742,-0.006567,-0.039245};

int d = 0;

void loop(void)
{
  byte present = 0;
  byte addr[8], data[12];
  static float s = 0.0;

  if( !ds.search(addr) ) // No more addresses
  {
    Serial.print( " " ); Serial.print( s / NDEVS ); s = 0.0;
    ds.reset_search(); delay(250); Serial.println();
    d = 0;
    return;
  }

  if( OneWire::crc8(addr, 7) != addr[7] ) { Serial.println("CRC in addr is not valid!"); return; }
  if( addr[0] != 0x28 ) { Serial.println("Chip is not DS18B20!"); return; }

  ds.reset(); ds.select(addr); ds.write(0x44,0); // start conversion; with parasite power on at the end: ,1)
  delay(780);     // maybe 750ms is enough, maybe not

  present = ds.reset(); ds.select(addr); ds.write(0xBE); // Read Scratchpad
  if( ! present ) { for( int i = 0; i < 9; ++i ) data[i] = ds.read(); return; } // just read 9 bytes if not ready
  for( int i = 0; i < 9; ++i ) data[i] = ds.read(); // we need 9 bytes
  if( OneWire::crc8(data, 8) != data[8]) { Serial.println("CRC in data is not valid!"); return; }

  int16_t raw = (data[1] << 8) | data[0];
  byte cfg = (data[4] & 0x60);
  if(cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
  else if(cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
  else if(cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
  float t = (float)raw / 16.0 + corr[d]; s += t; ++d;
  Serial.print( t );  Serial.print(" ");
}


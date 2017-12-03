#include <OneWire.h>

OneWire  ds(2);  // on pin 2 (a 4.7K resistor is necessary)

void setup(void) { Serial.begin(9600); pinMode(13, OUTPUT); digitalWrite(13, LOW); enumdevs(); }

#define NDEVS 15 // max number of devices
#define DEVSZ 8
byte devs[NDEVS][DEVSZ];
int ndevs = 0;

float corr[] = {-0.1009, -0.0896, 0.0228, 0.0708, 0.0602, 0.0672, 0.1206, -0.1159, 0.0026, -0.0378};

int srchdev(byte* addr)
{
  for( int d = 0; d < NDEVS; ++d )
  {
    int b = 0; for( ; b < DEVSZ; ++b ) if( devs[d][b] != addr[b] ) break;
    if( b == DEVSZ ) return d;
  }
  return -1;
}

int adddev(byte* addr)
{
  if( ndevs == NDEVS ) { Serial.println("Max number of devices"); return; }
  for( int b = 0; b < DEVSZ; ++b )
    devs[ndevs][b] = addr[b];
  return ndevs++;
}

#define p( s ) Serial.print( s )

void ph( int x ) { if(x<16) p('0'); Serial.print(x,HEX); }

void printdevs()
{
  p("Found "); p(ndevs); Serial.println(" devices:");
  for( int d=0; d<ndevs; ++d )
  {
    ph( d ); p( ": " );
    for( int b=0; b<DEVSZ; ++b ) { if(b>0) p( " " ); ph( devs[d][b] ); }
    Serial.println("");
  }
}

void enumdevs()
{
  byte addr[DEVSZ];
  ds.reset_search();
  delay(250);
  while( ds.search(addr) ) // search on wire
  {
    if( OneWire::crc8(addr, 7) != addr[7] ) { Serial.println("CRC in addr is not valid!"); break; }
    // the ROM byte indicates which chip (=family)
    if( addr[0] != 0x28 ) { Serial.println("Chip is not DS18B20!"); return; }
    int dev = srchdev(addr);
    if( dev < 0 )
      dev = adddev(addr);
    delay(50);
  }
  printdevs();
}

#define SKIP -31999
#define LDATA 9

int16_t processdev(int d)
{
  byte data[12];
  byte* addr = devs[d];

  ds.reset(); ds.select(addr); ds.write(0x44); // start conversion; with parasite power on at the end: ,1)

  delay(781+(d>=6)); // 750ms is enough; 781+(0..4)/3 --> 3999..4000 for 5 devs

  byte present = ds.reset(); ds.select(addr); ds.write(0xBE); // Read Scratchpad

  if( ! present ) { for( int i = 0; i < 9; ++i ) data[i] = ds.read(); return SKIP; } // just read 9 bytes if not ready

  for( int i = 0; i < 9; ++i ) data[i] = ds.read(); // we need 9 bytes
  if( OneWire::crc8(data, 8) != data[8]) { Serial.println("CRC in data is not valid!"); return SKIP; }

  int16_t raw = (data[1] << 8) | data[0];
  byte cfg = (data[4] & 0x60);
  // at lower res, the low bits are undefined, so let's zero them
  if(cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
  else if(cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
  else if(cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
  return raw;
}

void p3( float x )
{
  if( x<0.0 ) { p( "-" ); p3( -x ); return; }
  x += 0.0005;
  int xi = (int)x;
  int xj = (int)((x - (float)xi) * 1000.0);
  p( xi ); p( '.' ); p( xj/100 ); p( (xj/10)%10 ); p( xj%10 );
}

void p4( float x )
{
  if( x<0.0 ) { p( "-" ); p4( -x ); return; }
  x += 0.00005;
  int xi = (int)x;
  int xj = (int)((x - (float)xi) * 10000.0);
  p( xi ); p( '.' ); p( xj/1000 ); p( (xj/100)%10 ); p( (xj/10)%10 ); p( xj%10 );
}

unsigned long mt = 0; // millitime

void loop()
{
  if( mt == 0 ) mt = millis();
  float s = 0.0; float ss = 0.0;
  int16_t rawmin, rawmax;
  float tmin, tmax;
  for( int d = 0; d < ndevs; ++d )
  {
    int16_t raw = processdev( d );
    if( raw == SKIP ) continue;
    float t = (float)raw / 16.0; // + corr[d];
    s += t; ss += t*t;
    if( d==0 || raw<rawmin ) rawmin = raw; if( d==0 || raw>rawmax ) rawmax = raw;
    if( d==0 || t<tmin ) tmin = t; if( d==0 || t>tmax ) tmax = t;
    if( d>0 ) p( " " ); p4( t );
  }
  float avg = s / (float)ndevs;
  p( "   " ); p4( avg );                               // average
  p( "   " ); p4( sqrt( ss/(float)ndevs - avg*avg ) ); // std
  p( "   " ); p4( tmax - tmin );                       // t.range
  // p( "   " ); p4( (float)(rawmax - rawmin) / 16.0 );   // raw.range
  unsigned long now = millis();
  p( "   " ); Serial.println( now - mt ); mt = now;
}

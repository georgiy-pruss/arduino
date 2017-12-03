#include <OneWire.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>

#define OLED_W 128
#define OLED_H 64
#define OLED_RESET 4 // ?

Adafruit_SSD1306 display(OLED_RESET);
OneWire  ds(2);  // on pin 2 (a 4.7K resistor is necessary)

#define NDEVS 10 // max number of devices
#define DEVSZ 8
byte devs[NDEVS][DEVSZ];
int ndevs = 0;

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
  if( ndevs == NDEVS ) { /* Serial.println("Max number of devices"); */ return; }
  for( int b = 0; b < DEVSZ; ++b ) devs[ndevs][b] = addr[b];
  return ndevs++;
}

void enumdevs()
{
  byte addr[DEVSZ];
  ds.reset_search();
  delay(250);
  while( ds.search(addr) ) // search on wire
  {
    if( OneWire::crc8(addr, 7) != addr[7] ) { Serial.println(F("CRC in addr is not valid!")); break; }
    // the ROM byte indicates which chip (=family)
    if( addr[0] != 0x28 ) { Serial.println(F("Chip is not DS18B20!")); return; }
    int dev = srchdev(addr);
    if( dev < 0 )
      dev = adddev(addr);
    delay(50);
  }
}

#define SKIP -31999
#define LDATA 9

int16_t processdev(int d)
{
  byte data[12];
  byte* addr = devs[d];
  ds.reset(); ds.select(addr); ds.write(0x44); // start conversion; with parasite power on at the end: ,1)
  delay(770); // maybe 750ms is enough, maybe not
  byte present = ds.reset(); ds.select(addr); ds.write(0xBE); // Read Scratchpad
  if( !present ) { for( int i = 0; i < 9; ++i ) data[i] = ds.read(); return SKIP; } // just read 9 bytes if not ready
  for( int i = 0; i < 9; ++i ) data[i] = ds.read(); // we need 9 bytes
  if( OneWire::crc8(data, 8) != data[8]) { Serial.println(F("CRC in data is not valid!")); return SKIP; }
  int16_t raw = (data[1] << 8) | data[0];
  byte cfg = (data[4] & 0x60);
  if(cfg == 0x00) raw = raw & ~7; else if(cfg == 0x20) raw = raw & ~3; else if(cfg == 0x40) raw = raw & ~1;
  return raw;
}

void pm( char* out, float x ) // [-]0.DDD -> ±DDD
{
  if( x<0.0 ) { out[0] = '-'; x = -x; } else out[0] = '+';
  x += 0.0005;
  int mi = (int)(1000.0*(x - (float)(int)x));
  out[1] = '0' + mi/100; out[2] = '0' + (mi/10)%10; out[3] = '0' + mi%10; out[4] = '\0';
}

void p3( char* out, float x )
{
  if( x<0.0 ) { out[0] = '-'; p3( out+1, -x ); return; }
  x += 0.0005;
  int xi = (int)x; int xj = (int)((x - (float)xi) * 1000.0);
  int n = sprintf( out, "%d", xi );
  out[n] = '.'; out[n+1] = '0' + xj/100; out[n+2] = '0' + (xj/10)%10; out[n+3] = '0' + xj%10; out[n+4] = '\0';
}

#define DC()       display.clearDisplay()
#define DTC(x)     display.setTextColor(x)
#define DTS(x)     display.setTextSize(x)
#define DXY(x,y)   display.setCursor(x,y)
#define DP(s)      display.print(s)
#define DPL(s)     display.println(s)
#define DPX(x,y,c) display.drawPixel(x,y,c)

#define NH 61
byte history_1[NH]; // 60*4s = 4m
byte history_2[NH]; // 60*4m = 4h
byte history_3[NH];
int nh1, nh2, h2cnt, h2sum, nh3;


void inithistory()
{
  for( int i=0; i<NH; ++i ) history_1[i] = history_2[i] = 0;
  nh1 = nh2 = h2cnt = 0; h2sum = 0;
}

float t_zero = 15.0; // can be -25.0° -15.0° -5.0° 5.0° 15.0° 25.0° -- range 25.5°
byte t2b( float t ) { return (int)((t-t_zero)*10.0+0.5); } // in 0.1° from t_zero

#define B 18
#define A 9

int mean( byte* history, int n )
{
  int s = 0;
  for( int i=NH-n; i<NH; ++i ) s += history[i];
  return (s+(n/2))/n;
}

int calc_offset( byte* history, int nh, int mv )
{
  int offs = A - mv;
  int L = history[NH-1] + offs; // last
  if( L<0 ) offs += L; else if( L>B ) offs -= (L-B);
  return offs;
}

void showhistory( byte* history, int nh, int x0 )
{
  int mean_value = mean( history, nh );
  int oled_offset = calc_offset( history, nh, mean_value );
  int min_value_o = mean_value / 10 * 10 + oled_offset;
  int max_value_o = min_value_o + 10;
  for( int i=0; i<NH; ++i )
  {
    DPX( x0+i, OLED_H - (history[i]+oled_offset) - 1, WHITE );
    //if(i%10==0||i==NH-1) { DPX( x0+i, OLED_H - 1, WHITE ); DPX( x0+i, OLED_H - B - 1, WHITE ); }
    if(i%5==0||i==NH-1) { DPX( x0+i, OLED_H - min_value_o - 1, WHITE ); DPX( x0+i, OLED_H - max_value_o - 1, WHITE ); }
  }
}

void addtohistory( float t )
{
  byte ti = t2b( t );
  for(int i=0; i<NH-1; ++i) history_1[i] = history_1[i+1];
  history_1[NH-1] = ti;
  if( nh1<NH ) ++nh1;
  h2sum += ti; ++h2cnt;
  if( h2cnt == NH )
  {
    for(int i=0; i<NH-1; ++i) history_2[i] = history_2[i+1];
    history_2[NH-1] = (h2sum + NH/2) / NH;
    if( nh2<NH ) ++nh2;
    h2sum = 0; h2cnt = 0;
  }
}

unsigned long mt = 0;
int k = 0;

void setup(void)
{
  Serial.begin(9600);
  pinMode(13, OUTPUT); digitalWrite(13, LOW);
  enumdevs();
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C); DC(); DTC(WHITE); DXY(0,0); DP(ndevs); /* devs */ display.display();
  inithistory();
}

void loop()
{
  if( mt == 0 ) mt = millis();
  delay(5999);
  float s = 0.0;
  float ss = 0.0;
  int16_t rawmin, rawmax;
  char out[20];
  DC(); DTS(1);
  float td[10];
  for( int d = 0; d < ndevs; ++d )
  {
    int16_t raw = processdev( d );
    if( raw == SKIP ) continue;
    if( d==0 || raw<rawmin ) rawmin = raw;
    if( d==0 || raw>rawmax ) rawmax = raw;
    float t = (float)raw / 16.0;
    s += t;
    ss += t*t;
    td[d] = t;
  }
  float n = (float)ndevs;
  float avg = s/n;
  float std_dev = sqrt( ss/n - avg*avg );
  for( int d = 0; d < ndevs; ++d )
  {
    DXY(0,9*d); pm( out, td[d]-avg ); DP( out );
  }
  DXY(26,0); pm( out, td[0]-avg ); DP( out );
  DXY(56,0); DTS(2); p3( out, avg ); DP( out ); // average
  DXY(92,20); DTS(1); DP( '\xF0' ); p3( out, std_dev ); DP( out ); // std.dev
  unsigned long now = millis();
  DXY(58,36); sprintf( out, "%d %d  ", k, (int)(now-mt)-10000 ); DP( out );
  DXY(92,36); DP( '|' ); p3( out, (float)(rawmax - rawmin) / 16.0 ); DP( out ); // range
  mt = now;
  ++k;
  addtohistory( avg );
  showhistory( history_1, nh1, 67 ); if( nh2>0 ) showhistory( history_2, nh2, 0 );
  display.display();
}

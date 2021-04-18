#include <stdlib.h>

float x = 0.0;
float y = 0.0;

char *dtostrf(double value, int width, unsigned int precision, char *result)
{
  int decpt, sign, reqd, pad;
  const char *s, *e;
  char *p;
  s = fcvt(value, precision, &decpt, &sign);
  if (precision == 0 && decpt == 0) {
  s = (*s < '5') ? "0" : "1";
    reqd = 1;
  } else {
    reqd = strlen(s);
    if (reqd > decpt) reqd++;
    if (decpt == 0) reqd++;
  }
  if (sign) reqd++;
  p = result;
  e = p + reqd;
  pad = width - reqd;
  if (pad > 0) {
    e += pad;
    while (pad-- > 0) *p++ = ' ';
  }
  if (sign) *p++ = '-';
  if (decpt <= 0 && precision > 0) {
    *p++ = '0';
    *p++ = '.';
    e++;
    while ( decpt < 0 ) {
      decpt++;
      *p++ = '0';
    }
  }    
  while (p < e) {
    *p++ = *s++;
    if (p == e) break;
    if (--decpt == 0) *p++ = '.';
  }
  if (width < 0) {
    pad = (reqd + width) * -1;
    while (pad-- > 0) *p++ = ' ';
  }
  *p = 0;
  return result;
}

void setup() {
  while( !SerialUSB ) ;
  Serial.begin(9600);
}

void loop() {
  char str1[20];
  char str2[20];
  x += 0.001;
  y -= 0.001;
  dtostrf(x, 6, 3, str1);
  dtostrf(y, 6, 3, str2);
  SerialUSB.print( "X Val:" );
  SerialUSB.print( str1 );
  SerialUSB.print( " Y Val:" );
  SerialUSB.println( str2 );
}


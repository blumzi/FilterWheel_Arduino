#ifndef DEBUG_H
#define DEBUG_H

//#define DEBUG

#ifdef DEBUG
#define debug(x)    Serial.print(x)
#define debugln(x)  Serial.println(x)
#else
#define debug(x)
#define debugln(x)
#endif

#endif
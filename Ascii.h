#ifndef Ascii_h
#define Ascii_h

#include <inttypes.h>


class Ascii {
public:
    static const uint8_t STX =  2;
    static const uint8_t ETX =  3;
    static const uint8_t NL  = 10;
    static const uint8_t CR  = 13;

    Ascii();
    ~Ascii();
    uint8_t bcd(uint8_t *);
};
#endif

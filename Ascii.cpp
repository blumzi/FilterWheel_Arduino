#include <inttypes.h>
#include "Ascii.h"

Ascii::Ascii() {
}

Ascii::~Ascii() {}

uint8_t a2b(uint8_t* p) {
  if (*p >= '0' && *p <= '9')
    return *p - '0';
  else if (*p >= 'A' && *p <= 'F')
    return *p - 'A' + 10;
  return 0; // should not happen
}

//
// Binary coded decimal
//
uint8_t Ascii::bcd(uint8_t *p) {
  return ((a2b(p) & 0xf) << 4 ) | a2b(p + 1);
}

#include "nanosat_message.h"
/*
//-----------------------------------------------------------
// Encode a Fletcher checksum in the last 2 bytes of buffer
// @see http://en.wikipedia.org/wiki/Fletcher's_checksum
//-----------------------------------------------------------
inline uint16_t fletcher_encode(uint8_t buffer[], long count )
{
    int i;
    unsigned char c0 = 0;
    unsigned char c1 = 0;
     *( buffer + count - 1 ) = 0;
     *( buffer + count - 2 ) = 0;
    for( i = 0; i < count; i++) {
        c0 = c0 + *( buffer + i );
        c1 = c1 + c0;
    }
    uint8_t hob = c0-c1;
    uint8_t lob = c1 - 2*c0;
    return (uint16_t)(hob<<8) | lob;
 }

//-------------------------------------------------------------
// Decode Fletcher Checksum. Returns zero if buffer error-free
// @see http://en.wikipedia.org/wiki/Fletcher's_checksum
//-------------------------------------------------------------
inline long fletcher_decode(uint8_t buffer[], long count )
{
    //long result = 0;
    int i;
    unsigned char c0 = 0;
    unsigned char c1 = 0;
    for( i = 0; i < count; i++) {
        c0 = c0 + *( buffer + i );
        c1 = c1 + c0;
    }
    return((long)(c0 + c1)); // returns zero if buffer is error-free
}
*/

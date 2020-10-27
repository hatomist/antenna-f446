#ifndef ANTENNA_F446_NMEA0183_DECODERS_H
#define ANTENNA_F446_NMEA0183_DECODERS_H


#include "nmea0183_decode.h"

char *strtok_noskip(char * string, char const * delimiter, char **save_ptr);
void NMEA_decoders_GGA(const char *sentence, NMEAObject *object);

#endif //ANTENNA_F446_NMEA0183_DECODERS_H

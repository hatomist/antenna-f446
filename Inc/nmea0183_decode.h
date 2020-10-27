#ifndef ANTENNA_F446_NMEA0183_DECODE_H
#define ANTENNA_F446_NMEA0183_DECODE_H

#include <stm32f4xx_hal.h>
#include <string.h>
#include <errno.h>
#include <malloc.h>
#include <stdlib.h>


#define NMEA_MAX_SENTENCE_LENGTH 80
#define NMEA_GGA_HASH 59988

enum NMEA_sentence_id {
    NMEA_INVALID = -1,
    NMEA_UNKNOWN = 0,
    NMEA_GGA = 1
};

typedef struct {
    // UTC time
    uint8_t hh;
    uint8_t mm;
    uint8_t ss;
    uint8_t ms;
} Time;

typedef struct {
    Time time;
    float lat;
    char north_south; // N for north, S for south
    float lon;
    char west_east; // W for west, E for east
    uint8_t gps_quality; // 0 - fix not av, 1 - GPS fix, 2 - differential GPS fix
    uint8_t satellites_in_view; // 00 - 12
    float HDOP; // horizontal dilution of precision
    float alt; // altitude above/below mean-sea-level (geoid)
    float geoidal_separation; // Geoidal separation, the difference between the WGS-84 earth
//    ellipsoid and mean-sea-level (geoid), "-" means mean-sea-level below ellipsoid
    float data_age;
    uint16_t ref_station_id; // 0000 - 1023
} NMEA_GGA_t;

typedef struct {
    uint8_t type;
    union
    {
        void *any;
        NMEA_GGA_t* gga;
    } data;
} NMEAObject;

uint8_t NMEA_checksum(const char **sentence_ptr);
uint8_t NMEA_hex_to_int(char c);
uint8_t NMEA_hex_word_to_int(const char **sentence_ptr);
uint8_t NMEA_check(const char *sentence, uint8_t strict_length);
void NMEA_decode(const char *sentence, NMEAObject *object, uint8_t strict_checksum);
uint16_t NMEA_hash_string(const char *string);
void NMEA_free_obj(NMEAObject *obj);

#endif //ANTENNA_F446_NMEA0183_DECODE_H

#include "nmea0183_decoders.h"

char *strtok_noskip(char * string, char const * delimiter, char **save_ptr){
    char *p, *ret = 0;
    if(string != NULL)         *save_ptr = string;
    ret = *save_ptr;
    if((p = strpbrk (*save_ptr, delimiter)) != NULL) {
        *p  = 0;
        ret = *save_ptr;
        *save_ptr = ++p;
    }
    return ret;
}

void NMEA_decoders_GGA(const char *sentence, NMEAObject *object)
{
    object->type = NMEA_GGA;
    object->data.gga = (NMEA_GGA_t *) malloc(sizeof(NMEA_GGA_t));

    char mod_sentence[strlen(sentence)];
    memcpy(mod_sentence, sentence, strlen(sentence));
    char *save_ptr;
    char *token = strtok_r(mod_sentence, ",", &save_ptr); // initialize token, discard first

    // time
    token = strtok_noskip(NULL, ",", &save_ptr);
    char time_part[3];
    time_part[2] = 0;
    memcpy(time_part, &token[0], 2);
    object->data.gga->time.hh = strtol(time_part, NULL, 10);
    memcpy(time_part, &token[2], 2);
    object->data.gga->time.mm = strtol(time_part, NULL, 10);
    memcpy(time_part, &token[4], 2);
    object->data.gga->time.ss = strtol(time_part, NULL, 10);
    memcpy(time_part, &token[7], 2);
    object->data.gga->time.ms = strtol(time_part, NULL, 10);

    // lat
    token = strtok_noskip(NULL, ",", &save_ptr);
    object->data.gga->lat = strtof(token, NULL);

    // N/S
    token = strtok_noskip(NULL, ",", &save_ptr);
    object->data.gga->north_south = token[0];

    // lon
    token = strtok_noskip(NULL, ",", &save_ptr);
    object->data.gga->lon = strtof(token, NULL);

    // W/E
    token = strtok_noskip(NULL, ",", &save_ptr);
    object->data.gga->west_east = token[0];

    // GPS quality
    token = strtok_noskip(NULL, ",", &save_ptr);
    object->data.gga->gps_quality = strtol(token, NULL, 10);

    // № of satellites in view
    token = strtok_noskip(NULL, ",", &save_ptr);
    object->data.gga->satellites_in_view = strtol(token, NULL, 10);

    // Horizontal Dilution of precision
    token = strtok_noskip(NULL, ",", &save_ptr);
    object->data.gga->HDOP = strtof(token, NULL);

    // Altitude
    token = strtok_noskip(NULL, ",", &save_ptr);
    object->data.gga->alt = strtof(token, NULL);
    token = strtok_noskip(NULL, ",", &save_ptr);


    // Geoidal separation
    token = strtok_noskip(NULL, ",", &save_ptr);
    object->data.gga->geoidal_separation = strtof(token, NULL);
    token = strtok_noskip(NULL, ",", &save_ptr);

    // GPS data age
    token = strtok_noskip(NULL, ",", &save_ptr);
    object->data.gga->data_age = strtof(token, NULL);

    // Station ID
    token = strtok_noskip(NULL, "*", &save_ptr);
    object->data.gga->ref_station_id = strtol(token, NULL, 10);

}

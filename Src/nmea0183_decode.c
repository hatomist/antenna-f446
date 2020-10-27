#include "nmea0183_decode.h"
#include "nmea0183_decoders.h"

uint8_t NMEA_checksum(const char **sentence_ptr)
{
    uint8_t checksum = 0;
    if (*(*sentence_ptr) == '$')
        sentence_ptr++;

    while (*sentence_ptr && *(*sentence_ptr) != '*')
        checksum ^= *(*sentence_ptr)++;

    return checksum;
}

uint8_t NMEA_hex_to_int(char c)
{
    if (c >= '0' && c <= '9')
        return c - '0';
    if (c >= 'A' && c <= 'F')
        return c - 'A' + 10;
    if (c >= 'a' && c <= 'f')
        return c - 'a' + 10;
    return 255;
}

uint8_t NMEA_hex_word_to_int(const char **sentence_ptr)
{
    uint8_t upper = NMEA_hex_to_int(*(*sentence_ptr)++);
    uint8_t lower = NMEA_hex_to_int(*(*sentence_ptr)++);
    if (upper == 255 || lower == 255)
        errno = -1;
    return upper << 4 | lower;
}

uint8_t NMEA_check(const char *sentence, uint8_t strict_length)
{
    // max sentence length exceeded
    if ((strlen(sentence) > NMEA_MAX_SENTENCE_LENGTH + 3) && strict_length)
        return 0;

    // valid sentence must start with '$'
    if (*sentence++ != '$')
        return 0;

    // compute sentence checksum
    uint8_t checksum = NMEA_checksum(&sentence);
    if (*sentence++ != '*')
        return 0;

    // compare computed checksum to
    errno = 0;
    if (NMEA_hex_word_to_int(&sentence) != checksum || errno)
        return 0;

    return 1;
}

uint16_t NMEA_hash_string(const char *string)
{
    uint16_t hash = 5381;
    int c;

    while ((c = *string++))
        hash = ((hash << 5) + hash) + c; /* hash * 33 + c */

    return hash;
}

void NMEA_decode(const char *sentence, NMEAObject *object, uint8_t strict_checksum)
{
    if (!NMEA_check(sentence, 0) && strict_checksum)
    {
        object->type = NMEA_INVALID;
        return;
    }


    // get 3-char sentence identifier e.g. NMEA_GGA_t
    char type[4];
    type[3] = 0;
    memcpy(type, &sentence[3], 3);
    uint16_t type_hash = NMEA_hash_string(type);

    switch (type_hash)
    {
        case NMEA_GGA_HASH:
            NMEA_decoders_GGA(sentence, object);
            break;
        default:
            object->type = NMEA_UNKNOWN;
            break;
    }

}



void NMEA_free_obj(NMEAObject *obj)
{
    free(obj->data.any);
    free(obj);
}

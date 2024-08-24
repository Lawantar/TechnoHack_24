#ifndef PARSE___NMEA
#define PARSE___NMEA

uint32_t converter_time(char* TimeStamp, char* date);
uint8_t parser(char *inpString, uint16_t len, GPS* gps);

#endif

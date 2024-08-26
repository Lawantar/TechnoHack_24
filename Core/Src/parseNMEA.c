#include "main.h"
#include <stdlib.h>
#include <string.h>
#include <time.h>


uint32_t converter_time(char* TimeStamp, char* date) {
    struct tm tm_time = {0};  // Инициализируем структуру tm нулями
    char buff[3] = {0};  // Временный буфер для преобразования

    // Разбор даты (формат DDMMYY)
    buff[0] = date[0];
    buff[1] = date[1];
    tm_time.tm_mday = atoi(buff);  // День

    buff[0] = date[2];
    buff[1] = date[3];
    tm_time.tm_mon = atoi(buff) - 1;  // Месяц (0-11, поэтому нужно вычесть 1)

    buff[0] = date[4];
    buff[1] = date[5];
    tm_time.tm_year = atoi(buff) + 100;  // Год с учетом 1900, поэтому добавляем 100

    // Разбор времени (формат HHMMSS)
    buff[0] = TimeStamp[0];
    buff[1] = TimeStamp[1];
    tm_time.tm_hour = atoi(buff);  // Часы

    buff[0] = TimeStamp[2];
    buff[1] = TimeStamp[3];
    tm_time.tm_min = atoi(buff);  // Минуты

    buff[0] = TimeStamp[4];
    buff[1] = TimeStamp[5];
    tm_time.tm_sec = atoi(buff);  // Секунды

    // Преобразуем tm в количество секунд с 1 января 1970 года
    uint32_t seconds = mktime(&tm_time);

    return seconds;
}

uint8_t parser(char *inpString, uint16_t len, GPS* gps) {
	
	char buff_for_init_data[6] = {0};  // Буфер для поиска строки "GNRMC"
    int start_index = 0;
    char c;
	
	// Поиск строки "GNRMC"
    for (int k = 0; k < len; k++) {
        for (int i = 0; i < 5 && k + i < len; i++) {
            buff_for_init_data[i] = inpString[k + i];
        }
        buff_for_init_data[5] = '\0';
        if (strcmp(buff_for_init_data, "GNRMC") == 0) {
            start_index = k + 5;  // Начинаем после "GNRMC,"
            break;
        }
    }
	
	if (start_index == 0) {
        return 2;
    }
	
	if (start_index > 1022) {
	        return 3;
	}

	// Выделение строки после "GNRMC,"
    char for_tokens[128];
	
	int i = 0;
    while (inpString[start_index] != '\n' && inpString[start_index] != '\r') {
        for_tokens[i++] = inpString[start_index++];
    }
    for_tokens[i] = '\0';

    // Парсинг строки для извлечения данных
    char timeStamp[11] = {0}, status[2] = {0}, latitude[12] = {0}, ns[2] = {0};
    char longitude[13] = {0}, ew[2] = {0}, date[7] = {0};
	unsigned char ts = 0, lt = 0, lg = 0, dt = 0;
	
	
	i = 0;
	int count = 0;
    
    while(((c = for_tokens[i]) != '\n') && (c != '\r') && (i <= 126)) {

		if (c == ','){
			count++;
			i++;
			continue;
		}
		if (count > 9){
			break;
		}
		if (count == 0){
			break;
		}
		if (count == 1){
			while(c != ',' && c != '\n' && c != '\r'){
				timeStamp[ts++] = c;
				c = for_tokens[++i];
			}
			timeStamp[ts] = '\0';
		}
		else if (count == 2){
			status[0] = c;
			status[1] = '\0';
			i++;
		}
		else if (count == 3){
			while(c != ',' && c != '\n' && c != '\r'){
				latitude[lt++] = c;
				c = for_tokens[++i];
			}
			latitude[lt] = '\0';
		}
		else if (count == 4){
			ns[0] = c;
			ns[1] = '\0';
			i++;
		}
		else if (count == 5){
			while(c != ',' && c != '\n' && c != '\r'){
				longitude[lg++] = c;
				c = for_tokens[++i];
			}
			longitude[lg] = '\0';
		}
		else if (count == 6){
			ew[0] = c;
			ew[1] = '\0';
			i++;
		}
		else if (count == 9){
			while(c != ',' && c != '\n' && c != '\r'){
				date[dt++] = c;
				c = for_tokens[++i];
			}
			date[dt] = '\0';
		}
		else if ( count > 6 && count < 9){
			while(c != ',' && c != '\n' && c != '\r'){
				c = for_tokens[++i];
			}
			continue;
		} else {
			continue;
		}
	}
	
	gps->lat = strtof(latitude, NULL); // Готовые данные
	gps->lon = strtof(longitude, NULL); // Готовые данные
	gps->secs = converter_time(timeStamp, date); // Время

	return (uint8_t)status[0];

	
}

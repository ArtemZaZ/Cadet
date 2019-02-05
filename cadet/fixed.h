#pragma once 
#include "stdint.h"
/* Файл фиксированных параметров */

// для работы с EEPROM (адреса положений серв, записываемых в EEPROM)
#define EEPROM_ADDR_SERV_GUN_MIN      0   // идем с нуля, дальше прибавляем адреса
#define EEPROM_ADDR_SERV_GUN_MAX      EEPROM_ADDR_SERV_GUN_MIN + sizeof(uint16_t)

#define EEPROM_ADDR_SERV_MANIP1_MIN   EEPROM_ADDR_SERV_GUN_MAX + sizeof(uint16_t)
#define EEPROM_ADDR_SERV_MANIP1_MAX   EEPROM_ADDR_SERV_MANIP1_MIN + sizeof(uint16_t)

#define EEPROM_ADDR_SERV_MANIP2_MIN   EEPROM_ADDR_SERV_MANIP1_MAX + sizeof(uint16_t)
#define EEPROM_ADDR_SERV_MANIP2_MAX   EEPROM_ADDR_SERV_MANIP2_MIN + sizeof(uint16_t)

#define EEPROM_ADDR_SERV_MANIP3_MIN   EEPROM_ADDR_SERV_MANIP2_MAX + sizeof(uint16_t)
#define EEPROM_ADDR_SERV_MANIP3_MAX   EEPROM_ADDR_SERV_MANIP3_MIN + sizeof(uint16_t)


#define SERVO_CENTRAL_POSITION  350  // центральное положение серв (1500 мкс)
#define SERVO_FREQ  60  // частота ШИМ (~57Гц)
#define SERVO_CALIBRATE_DELAY 0   // зедержка сервы при калибровке - влияет на скорость при калибровке
#define SERVO_DELAY 3 // задержка сервы при движении - влияет на скорость в рабочем режиме

#define MOTOR_ZERO_PULSE  SERVO_CENTRAL_POSITION
#define MOTOR_MIN_PULSE   250
#define MOTOR_MAX_PULSE   500

// выводы джойстика
#define JOY_DAT_CH  5
#define JOY_CMD_CH  6
#define JOY_SEL_CH  7
#define JOY_CLK_CH  8

// вывод дисплея
#define DISPLAY_RESET_CH  4

// работа со звуком
#define BUZZER_CH 11  // вывод, подключённый к динамику

// ноты
#define NOTE_C 261
#define NOTE_D 294
#define NOTE_E 329
#define NOTE_F 349
#define NOTE_G 391
#define NOTE_A 440
#define NOTE_B 466


// работа с АЦП
#define ADC_VOLTAGE_CH  A1  // канал считывания напряжения
#define ADC_CURRENT_CH  A0  // канал считывания тока
#define ADC_UAREF   5.0     // опорное напряжение
#define ADC_MAX     1024    // максимальная разрядность
#define ADC_VOLT_DIV_CONST  1 // константа делителя напряжения
#define ADC_MAX_COUNT    15  // задержка преобразования АЦП

#define MAX_MCU_CURRENT 5 //максимальный ток, при превышении которого срабатывает защита (5А)
#define MIN_MCU_VOLTAGE 3.3
#define ADC_CURR_CONST 0.47

// Итерируемы объекты - каналы серв и их имена для режима калибровки - нуль-терминальные строки
const unsigned char SERVO_ITERATED[5] = {SERVO_GUN_CH, SERVO_MANIP1_CH, SERVO_MANIP2_CH, SERVO_MANIP3_CH, '\0'};
const char * SERVO_NAMES_ITERATED[5] = {"Gun", "Manip 1", "Manip 2", "Manip 3", '\0'};
const unsigned int EEPROM_ADDR_SERV_MIN[5] = {EEPROM_ADDR_SERV_GUN_MIN, EEPROM_ADDR_SERV_MANIP1_MIN,    // адреса максимальных позиций серв в епроме
                                              EEPROM_ADDR_SERV_MANIP2_MIN, EEPROM_ADDR_SERV_MANIP3_MIN, '\0'};
const unsigned int EEPROM_ADDR_SERV_MAX[5] = {EEPROM_ADDR_SERV_GUN_MAX, EEPROM_ADDR_SERV_MANIP1_MAX,    // адреса максимальных позиций серв в епроме
                                              EEPROM_ADDR_SERV_MANIP2_MAX, EEPROM_ADDR_SERV_MANIP3_MAX, '\0'};


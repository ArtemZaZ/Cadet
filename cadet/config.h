#pragma once
/* Файл переменной конфигурации 
 * файл должен быть подключен в начале скетча 
 */

// определение всех нужных параметров
// определение версии, если что-то координально меняется, т.е. не может быть изменено при попощи
// данного файла - добавляйте новую версию, и в коде несовпадающие эл-ты помещайте в препроцессорные дериктивы ветвления версий
#define VERSION 11 // версия 1.0 - 10 или 1.1 - 11

// регулирование скорости
#define SPEED_MIN   0  // наименьшее допустимое значение 90
#define SPEED_MAX   100 // наибольшее допустимое значение 255
#define SPEED_STEP  30  // размер приращения шага

// режимы работы джойстика
#define JOY_PRESSURES false  // аналоговое считывание кнопок  
#define JOY_RUMBLE    false  // вибромотор

// подключение серв (выводы/каналы pca)
#define SERVO_MANIP1_CH 6 // канал сервы первого звена манипулятора
#define SERVO_MANIP2_CH 5 // канал сервы второго звена манипулятора
#define SERVO_MANIP3_CH 7 // канал сервы третьего звена манипулятора
#define SERVO_GUN_CH    4 // канал сервы стрелялки

#define SERVO_MOTOR_RIGHT_CH  1  // канал правого борта моторов
#define SERVO_MOTOR_LEFT_CH   2  // канал левого борта моторов

#define SERVO_STEP  3   // шаг изменения положения сервы при движении в рабочем режиме, влияет на скорость движения сервы

#define GUN_ACTIVE_DELAY  500 // сколько времени стрелялка будет в активном положении

#define POWER_DISPLAY_MAX_COUNT 80 // время обновления дисплэя в рабочем режиме в попугаях

/* проверка параметров - при настройке не трогаем */
#ifndef VERSION // если не продефайнина версия
# error "VERSION is not defined"
#endif

#if (VERSION != 10) && (VERSION != 11)  // проверка на допустимые версии
# error "The selected version does not exist"
#endif

#if !(defined(SPEED_MIN) && defined(SPEED_MAX) && defined(SPEED_STEP))  // если не продефайнены значения скоростей
# error "Speed parameters are not defined" 
#endif

#if !(defined(JOY_PRESSURES) && defined(JOY_RUMBLE))  // если не продефайнены параметры джойстика
# error "Joystick modes not defined" 
#endif

#if !(defined(SERVO_MANIP1_CH) && defined(SERVO_MANIP2_CH) && defined(SERVO_MANIP3_CH) && defined(SERVO_GUN_CH)) // если не продефайнены каналы серв
# error "Servo channels not defined" 
#endif

// подключаем фиксированные параметры, проверку не делаем, т.к. не меняем их
#include "fixed.h"



#include "config.h"
#include "pictures.h"
#include "stdint.h"
#include <SPI.h>
#include <Wire.h>
#include <EEPROM.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_PWMServoDriver.h>
#include <PS2X_lib.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(); //инициализация i2c для pca с адресом 0x40
Adafruit_SSD1306 display(DISPLAY_RESET_CH); // инициализация дисплея
PS2X ps2x;  // cоздание экземпляра класса для джойстика


uint8_t motorSpeed = SPEED_MIN;   // скорость мотора, текущая
// пока глобальные мб потом сделаю локальные
uint16_t servoGunMin = SERVO_CENTRAL_POSITION;   // минимальное положение диспенсора
uint16_t servoGunMax = SERVO_CENTRAL_POSITION;   // максимальное
uint16_t servoManip1Min = SERVO_CENTRAL_POSITION;    // минимальное полпжение плуга
uint16_t servoManip1Max = SERVO_CENTRAL_POSITION;    // максимальное
uint16_t servoManip2Min = SERVO_CENTRAL_POSITION;  // минимальное положение ковша
uint16_t servoManip2Max = SERVO_CENTRAL_POSITION;  // максимальное
uint16_t servoManip3Min = SERVO_CENTRAL_POSITION;  // минимальное положение схвата ковша
uint16_t servoManip3Max = SERVO_CENTRAL_POSITION;  // максимальное

/// Ф-ии пока особо несмотрел, буду пересматривать после перестройки логики

void motorSetup()   // инициализация моторов
{
  
}


void buzzerSetup()  // инициализация пищалки
{
  pinMode(BUZZER_CH, OUTPUT);
  noTone(BUZZER_CH);
}


void displaySetup() // инициализация дисплея
{
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // Инициализация I2C для дисплея с адресом 0x3D
  display.display();
  delay(2000); //задержка для инициализации дисплея
  display.clearDisplay(); // очистка дисплея
}


void servoSetup() // инициализация серв
{
  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);  // Установка частоты ШИМ
}


void servoCentering() // центрирование серв
{
  pwm.setPWM(SERVO_MANIP1_CH, 0, SERVO_CENTRAL_POSITION);
  pwm.setPWM(SERVO_MANIP2_CH, 0, SERVO_CENTRAL_POSITION);
  pwm.setPWM(SERVO_MANIP3_CH, 0, SERVO_CENTRAL_POSITION);
  pwm.setPWM(SERVO_GUN_CH, 0, SERVO_CENTRAL_POSITION);
}


void readServoRange() // читает значения крайних положений серв из епрома и записывает их в глобальные переменные
{
  //Значения границ серв считываются из энергонезависимой памяти
  EEPROM.get(EEPROM_ADDR_SERV_GUN_MAX, servoGunMax);
  EEPROM.get(EEPROM_ADDR_SERV_GUN_MIN, servoGunMin);

  EEPROM.get(EEPROM_ADDR_SERV_MANIP1_MAX, servoManip1Max);
  EEPROM.get(EEPROM_ADDR_SERV_MANIP1_MIN, servoManip1Min);

  EEPROM.get(EEPROM_ADDR_SERV_MANIP2_MAX, servoManip2Max);
  EEPROM.get(EEPROM_ADDR_SERV_MANIP2_MIN, servoManip2Min);

  EEPROM.get(EEPROM_ADDR_SERV_MANIP3_MAX, servoManip3Max);
  EEPROM.get(EEPROM_ADDR_SERV_MANIP3_MIN, servoManip3Min);
}


void displayImage(const unsigned char* image)
{
  display.clearDisplay();
  display.drawBitmap(0, 0,  image, IMAGE_WIDTH, IMAGE_HEIGHT, 1);
  display.display();
}


void printText(char* str, uint8_t textsize)  //Вывод строки на дисплей
{
  display.clearDisplay();
  display.setTextSize(textsize);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.println(str);
  display.display();
}


void servoCalibrateDisplay(char* servoName, uint16_t servoPosition)   // вывод на дисплей название сервы и ее текущую позицию
{
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.println(servoName);
  display.setCursor(0, 16);
  display.println(servoPosition);
  display.display();  
}


void servoInfoDisplay(char* servoName, uint16_t servoPositionMin, uint16_t servoPositionMax)    // вывод на дисплей имя сервы и ее максимальное и минимальную позицию
{
  display.clearDisplay();
  display.setTextSize(1.5);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.println(servoName);
  display.setCursor(0, 10);
  display.println("MIN: ");
  display.setCursor(30, 8);
  display.println(servoPositionMin);
  display.setCursor(0, 20);
  display.println("MAX: ");
  display.setCursor(30, 20);
  display.println(servoPositionMax);
  display.display();    
}


uint16_t rerangeSpeed(int16_t mspeed)  // проверка и корректировка скорости
{
  if (mspeed > SPEED_MAX) return SPEED_MAX;
  if (mspeed < SPEED_MIN) return SPEED_MIN;
  return mspeed;
}


//Запуск двигателей 
void setSpeedRight(int16_t mspeed)  // первый двигатель - А, принимает значения от -100 до 100
{
  static uint16_t pulseLen = MOTOR_ZERO_PULSE;
  if(mspeed >= 0)
  {
    pulseLen = (uint16_t)((float)(abs(mspeed) / 100.f) * (MOTOR_MAX_PULSE - MOTOR_ZERO_PULSE)) + MOTOR_ZERO_PULSE;
  }
  else
  {
    pulseLen = (uint16_t)((float)(abs(mspeed) / 100.f) * (MOTOR_MIN_PULSE - MOTOR_ZERO_PULSE)) + MOTOR_ZERO_PULSE;
  }
  pwm.setPWM(SERVO_MOTOR_RIGHT_CH, 0, pulseLen);
}


void setSpeedLeft(int16_t mspeed) // второй двигатель - B
{
  static uint16_t pulseLen = MOTOR_ZERO_PULSE;
  mspeed = -mspeed;
  if(mspeed >= 0)
  {
    pulseLen = (uint16_t)((float)(abs(mspeed) / 100.f) * (MOTOR_MAX_PULSE - MOTOR_ZERO_PULSE)) + MOTOR_ZERO_PULSE;
  }
  else
  {
    pulseLen = (uint16_t)((float)(abs(mspeed) / 100.f) * (MOTOR_MIN_PULSE - MOTOR_ZERO_PULSE)) + MOTOR_ZERO_PULSE;
  }
  pwm.setPWM(SERVO_MOTOR_LEFT_CH, 0, pulseLen);
}


void stopMotors()   // остановка двигателей
{
  setSpeedRight(0);
  setSpeedLeft(0);
}


#if VERSION == 11
void beep(uint16_t ton, uint16_t tim) // для проигрывания тона
{
  tone(BUZZER_CH, ton, tim);
  delay(tim + 20);
}
#endif


#if VERSION == 10
void beep(uint8_t num, uint16_t tim)
{
  for (uint16_t i = 0; i < num; i++)
  {
    digitalWrite(BUZZER_CH, HIGH);
    delay(tim);
    digitalWrite(BUZZER_CH, LOW);
    delay(50);
  }
}
#endif


void beepAlarm() // мелодия предупреждения
{
#if VERSION == 11
  //мелодия
  beep(NOTE_G, 50);
  beep(NOTE_E, 50);
  beep(NOTE_C, 50);
  noTone(BUZZER_CH);
#endif
#if VERSION == 10
  beep(3, 100);
#endif
}


void gunActivate(uint32_t del)  // выстрел из стрелялки
{
  static uint32_t gunPulseLen = SERVO_CENTRAL_POSITION;  
  for (gunPulseLen = servoGunMax; gunPulseLen > servoGunMin; gunPulseLen--)
  {
    pwm.setPWM(SERVO_GUN_CH, 0, gunPulseLen);
    delay(del);
  }  
  delay(GUN_ACTIVE_DELAY);   
  for (gunPulseLen = servoGunMin; gunPulseLen < servoGunMax; gunPulseLen++)
  {
    pwm.setPWM(SERVO_GUN_CH, 0, gunPulseLen);
    delay(del);
  }
}


void adcDataCounter(float* voltage, float* current)   // вычисление значения напряжения питания и тока, запись в параметры
{
  static uint8_t adcCount = ADC_MAX_COUNT;  // ограничение по частоте считывания данных с АЦП
  static float m_voltage = 0;   // доп переменные для того, чтобы не ждать 14 вызовов ф-ии, если пропустили первый 
  static float m_current = 0;
  if (adcCount >= ADC_MAX_COUNT)    // каждые ADC_MAX_COUNT = 15 раз меняет значения,приходящие в параметрах
  {
    adcCount = 0;
    m_voltage = ADC_VOLT_DIV_CONST * analogRead(ADC_VOLTAGE_CH) * ADC_UAREF / ADC_MAX; //вычисление напряжения на выходе буффера
    m_current = analogRead(ADC_CURRENT_CH) * ADC_UAREF / ADC_MAX / ADC_CURR_CONST;
  }
  *voltage = m_voltage;
  *current = m_current;
  adcCount++;
}


bool calibrationFSM()   // режим калибровки, нетривиальный конечный автомат, где состояния управляются от одного лидера, состояния переключаются по нажатию кнопок джойстика 
{
  static int8_t servoCounter = 0;  // каретка, переключающаяся от сервы к серве (нужен знаковый) - текущая выбранная серва из массива
  static uint16_t servoCalibPos = SERVO_CENTRAL_POSITION;   // текущая позиция выбранной сервы
  static uint16_t tempInfoPositionMin = SERVO_CENTRAL_POSITION;   // доп переменные, для считывания информации из епрома о выбранной серве
  static uint16_t tempInfoPositionMax = SERVO_CENTRAL_POSITION;
  static enum   
  {
    LEAD,   // главный режим, отсюда идет переход ко всем остальным состояниям
    EEPROM_CLEAR,  // тут происходит очистка EEPROM
    SERVO_NEXT, // переход к следующей серве
    SERVO_PREV, // переход к предыдущей серве
    SERVO_CENTERING,  // центровка выбранной сервы
    SERVO_MOVE_UP,    // увеличение скважности ШИМа на серве
    SERVO_MOVE_DOWN,  // уменьшение скважности ШИМа на серве
    SERVO_FIND_MAX,   // находим максимальную скважность сервы, в одном из крайних положений
    SERVO_FIND_MIN,   // находим минимальную скважность, в одном из крайних положений
    EXIT   // выход из конечного автомата, переход к другому режиму
  } state;

  switch(state)
  {
    case LEAD:  // состояние которое переходит в другие состояния, если были нажаты какие-то кнопки
      if (ps2x.Button(PSB_L3) && ps2x.Button(PSB_R3) && ps2x.Button(PSB_R1) && ps2x.Button(PSB_L1)){ state = EEPROM_CLEAR; }  
      if (ps2x.ButtonPressed(PSB_R1)) { state = SERVO_NEXT; }
      if (ps2x.ButtonPressed(PSB_L1)) { state = SERVO_PREV; }
      if (ps2x.Button(PSB_L3) || ps2x.Button(PSB_R3)) { state = SERVO_CENTERING; }
      if (ps2x.Button(PSB_PAD_UP)) { state = SERVO_MOVE_UP; }
      if (ps2x.Button(PSB_PAD_DOWN)) { state = SERVO_MOVE_DOWN; }
      if (ps2x.ButtonPressed(PSB_TRIANGLE)) { state = SERVO_FIND_MAX; }
      if (ps2x.ButtonPressed(PSB_CROSS)) { state = SERVO_FIND_MIN; }
      if (ps2x.Button(PSB_L2) && ps2x.Button(PSB_R2)) { state = EXIT; }
      return false;

    case EEPROM_CLEAR:
      // тут будет очистка епрома
      state = LEAD;
      return false;

    case SERVO_NEXT:  // делаем кольцевой массив, можем ходить от серве к серве по замкнутому кругу
      servoCounter++;  
      if (servoCounter >= (strlen((const char*)SERVO_ITERATED))) servoCounter = 0;
      servoCalibPos = SERVO_CENTRAL_POSITION;   // центровка сервы при переключении
      pwm.setPWM(SERVO_ITERATED[servoCounter], 0, servoCalibPos);   // задаем ей центральное положение
      
      EEPROM.get(EEPROM_ADDR_SERV_MIN[servoCounter], tempInfoPositionMin);  // после выбора сервы считываем о ней информацию из епрома и выводим на дисплей
      EEPROM.get(EEPROM_ADDR_SERV_MAX[servoCounter], tempInfoPositionMax);     
      servoInfoDisplay((char*)SERVO_NAMES_ITERATED[servoCounter], tempInfoPositionMin, tempInfoPositionMax);
      state = LEAD;      
      return false;

    case SERVO_PREV:  // делаем кольцевой массив, можем ходить от серве к серве по замкнутому кругу
      servoCounter--;
      if (servoCounter < 0) servoCounter = strlen((const char*)SERVO_ITERATED) - 1; 
      servoCalibPos = SERVO_CENTRAL_POSITION;   // центровка сервы при переключении
      pwm.setPWM(SERVO_ITERATED[servoCounter], 0, servoCalibPos); // задаем ей центральное положение

      EEPROM.get(EEPROM_ADDR_SERV_MIN[servoCounter], tempInfoPositionMin);  // после выбора сервы считываем о ней информацию из епрома и выводим на дисплей
      EEPROM.get(EEPROM_ADDR_SERV_MAX[servoCounter], tempInfoPositionMax);     
      servoInfoDisplay((char*)SERVO_NAMES_ITERATED[servoCounter], tempInfoPositionMin, tempInfoPositionMax);
      state = LEAD;
      return false;

    case SERVO_CENTERING:   // центрирование выбранной сервы
      servoCalibPos = SERVO_CENTRAL_POSITION; // установка центрального значения для текущей сервы
      pwm.setPWM(SERVO_ITERATED[servoCounter], 0, servoCalibPos);
      
      EEPROM.get(EEPROM_ADDR_SERV_MIN[servoCounter], tempInfoPositionMin);    // после центрирования сервы считываем о ней информацию из епрома и выводим на дисплей
      EEPROM.get(EEPROM_ADDR_SERV_MAX[servoCounter], tempInfoPositionMax);     
      servoInfoDisplay((char*)SERVO_NAMES_ITERATED[servoCounter], tempInfoPositionMin, tempInfoPositionMax);
      state = LEAD;
      return false;

    case SERVO_MOVE_UP:  
      servoCalibPos++;    // увеличиваем положение сервы
      pwm.setPWM(SERVO_ITERATED[servoCounter], 0, servoCalibPos);   // задаем его
      delay(SERVO_CALIBRATE_DELAY); // делаем задержку
      servoCalibrateDisplay((char*)SERVO_NAMES_ITERATED[servoCounter], servoCalibPos);   // выводим имя сервы и текущее положение
      state = LEAD;
      return false;

    case SERVO_MOVE_DOWN:
      servoCalibPos--;    // уменьшаем положение сервы
      pwm.setPWM(SERVO_ITERATED[servoCounter], 0, servoCalibPos); // задаем его
      delay(SERVO_CALIBRATE_DELAY); // делаем задержку
      servoCalibrateDisplay((char*)SERVO_NAMES_ITERATED[servoCounter], servoCalibPos); // выводим имя сервы и текущее положение
      state = LEAD;
      return false;

    case SERVO_FIND_MAX:  // если найдено максимальное положение сервы
      EEPROM.put(EEPROM_ADDR_SERV_MAX[servoCounter], servoCalibPos);  // записываем его в епром по адресу из массива и сигналим
#if VERSION == 11
      beep(NOTE_C, 200);  
      noTone(BUZZER_CH);
#endif
#if VERSION == 10
      beep(1, 200);
#endif

      EEPROM.get(EEPROM_ADDR_SERV_MIN[servoCounter], tempInfoPositionMin);  // после записи, считываем информацию из епрома и выводим на дисплей
      EEPROM.get(EEPROM_ADDR_SERV_MAX[servoCounter], tempInfoPositionMax);     
      servoInfoDisplay((char*)SERVO_NAMES_ITERATED[servoCounter], tempInfoPositionMin, tempInfoPositionMax);
      state = LEAD;
      return false;

    case SERVO_FIND_MIN:  // если найдено максимальное положение сервы
      EEPROM.put(EEPROM_ADDR_SERV_MIN[servoCounter], servoCalibPos);  // записываем его в епром по адресу из массива и сигналим
#if VERSION == 11
      beep(NOTE_C, 200);
      noTone(BUZZER_CH);
#endif
#if VERSION == 10
      beep(1, 200);
#endif

      EEPROM.get(EEPROM_ADDR_SERV_MIN[servoCounter], tempInfoPositionMin);  // после записи, считываем информацию из епрома и выводим на дисплей
      EEPROM.get(EEPROM_ADDR_SERV_MAX[servoCounter], tempInfoPositionMax);     
      servoInfoDisplay((char*)SERVO_NAMES_ITERATED[servoCounter], tempInfoPositionMin, tempInfoPositionMax);
      state = LEAD;      
      return false;

    case EXIT:    // выход из режима калибровки
      readServoRange(); // чтение границ серв из епрома и запись в глобальные переменные
#if VERSION == 11
      beep(NOTE_C, 300);
      beep(NOTE_G, 300);
      beep(NOTE_B, 300);
      noTone(BUZZER_CH);
#endif
#if VERSION == 10
      beep(1, 500);
#endif
      servoCentering();   // центровка серв
      servoCounter = 0;   // сбрасываем каретку
      servoCalibPos = SERVO_CENTRAL_POSITION;   // сбрасываем положение
      display.clearDisplay();   // чистим дисплей
      display.display();  
      state = LEAD;  
      return true;
  }
}


bool workFSM()    // рабочий режим
{
  static uint16_t manip1PulseLen = SERVO_CENTRAL_POSITION;  // текущие положения сервы первого звена манипулятора
  static uint16_t manip2PulseLen = SERVO_CENTRAL_POSITION;  // текущие положения сервы второго звена манипулятора
  static uint16_t manip3PulseLen = SERVO_CENTRAL_POSITION;  // текущие положения сервы второго звена манипулятора
  static enum
  {
    LEAD,  // управляющий режим
    FORWARD,  // вперед
    BACKWARD, // назад 
    LEFT,   // влево
    RIGHT,  // вправо
    NOTHING, // остановка + ничего не делать
    SPEED_UP, // уменьшение скорости
    SPEED_DOWN,  // увеличение скорости
    GUN_ACTIVATION, // активация стрелялки
    MANIP1_UP,  // повернуть серву первого звена
    MANIP1_DOWN,  // повернуть серву первого звена
    MANIP2_UP,  // повернуть серву второго звена
    MANIP2_DOWN,  // повернуть серву второго звена
    MANIP3_UP,  // повернуть серву третьего звена       // !!! привязать к кнопкам
    MANIP3_DOWN,  // повернуть серву третьего звена
    EXIT  // переход к другому режиму 
  } state;

  switch(state)
  {
    case LEAD:
      if (ps2x.Button(PSB_PAD_UP)) { state = FORWARD; }
      if (ps2x.Button(PSB_PAD_DOWN)) { state = BACKWARD; }
      if (ps2x.Button(PSB_PAD_LEFT)) { state = LEFT; }
      if (ps2x.Button(PSB_PAD_RIGHT)) { state = RIGHT; }
      if (!((ps2x.Button(PSB_PAD_UP) || ps2x.Button(PSB_PAD_DOWN) ||    \
             ps2x.Button(PSB_PAD_LEFT) || ps2x.Button(PSB_PAD_RIGHT)))) { state = NOTHING; }
      if (ps2x.ButtonPressed(PSB_R1)) { state = SPEED_UP; }
      if (ps2x.ButtonPressed(PSB_R2)) { state = SPEED_DOWN; }
      if (ps2x.ButtonPressed(PSB_L1)) { state = GUN_ACTIVATION; }
      if (ps2x.Button(PSB_TRIANGLE)) { state = MANIP1_UP; }
      if (ps2x.Button(PSB_CROSS)) { state = MANIP1_DOWN; }
      if (ps2x.Button(PSB_CIRCLE)) { state = MANIP2_UP; }
      if (ps2x.Button(PSB_SQUARE)) { state = MANIP2_DOWN; }
      if (ps2x.Button(PSB_L3) && ps2x.Button(PSB_R3)) { state = EXIT; }
      return false;

    case FORWARD:
      setSpeedRight(motorSpeed);    // задаем скорости бортам моторов
      setSpeedLeft(motorSpeed);
      displayImage(eyes_up);  
      state = LEAD;
      return false;

    case BACKWARD:
      setSpeedRight(-motorSpeed);
      setSpeedLeft(-motorSpeed);
      displayImage(eyes_down);  
      state = LEAD;
      return false;

    case LEFT:
      setSpeedRight(motorSpeed);
      setSpeedLeft(-motorSpeed);
      displayImage(eyes_left);  
      state = LEAD;
      return false;

    case RIGHT:
      setSpeedRight(-motorSpeed);
      setSpeedLeft(motorSpeed);
      displayImage(eyes_right);  
      state = LEAD;
      return false;

    case NOTHING:
      stopMotors();
      state = LEAD;      
      return false;

    case SPEED_UP:
      motorSpeed = rerangeSpeed(motorSpeed + SPEED_STEP);   // переоценка скоростей, если они выходят из диапазона 
      state = LEAD;
      return false;

    case SPEED_DOWN:
      motorSpeed = rerangeSpeed(motorSpeed - SPEED_STEP);   // переоценка скоростей, если они выходят из диапазона 
      state = LEAD;
      return false;

     case GUN_ACTIVATION:
      gunActivate(0);
      displayImage(eyes_difficult);  
      state = LEAD;
      return false;

    case MANIP1_UP:
      manip1PulseLen = manip1PulseLen - SERVO_STEP;
      if (manip1PulseLen > servoManip1Max) manip1PulseLen = servoManip1Max;
      if (manip1PulseLen < servoManip1Min) manip1PulseLen = servoManip1Min;
      pwm.setPWM(SERVO_MANIP1_CH, 0, manip1PulseLen);
      displayImage(eyes_difficult);  
      state = LEAD;
      return false;

    case MANIP1_DOWN:
      manip1PulseLen = manip1PulseLen + SERVO_STEP;
      if (manip1PulseLen > servoManip1Max) manip1PulseLen = servoManip1Max;
      if (manip1PulseLen < servoManip1Min) manip1PulseLen = servoManip1Min;
      pwm.setPWM(SERVO_MANIP1_CH, 0, manip1PulseLen);
      displayImage(eyes_difficult);  
      state = LEAD;
      return false;

    case MANIP2_UP:
      manip2PulseLen = manip2PulseLen - SERVO_STEP;
      if (manip2PulseLen > servoManip2Max) manip2PulseLen = servoManip2Max;
      if (manip2PulseLen < servoManip2Min) manip2PulseLen = servoManip2Min;
      pwm.setPWM(SERVO_MANIP2_CH, 0, manip2PulseLen);
      displayImage(eyes_difficult);  
      state = LEAD;
      return false;

    case MANIP2_DOWN:
      manip2PulseLen = manip2PulseLen + SERVO_STEP;
      if (manip2PulseLen > servoManip2Max) manip2PulseLen = servoManip2Max;
      if (manip2PulseLen < servoManip2Min) manip2PulseLen = servoManip2Min;
      pwm.setPWM(SERVO_MANIP2_CH, 0, manip2PulseLen);
      displayImage(eyes_difficult);  
      state = LEAD;
      return false;

      case MANIP3_UP:
      manip3PulseLen = manip3PulseLen - SERVO_STEP;
      if (manip3PulseLen > servoManip3Max) manip3PulseLen = servoManip3Max;
      if (manip3PulseLen < servoManip3Min) manip3PulseLen = servoManip3Min;
      pwm.setPWM(SERVO_MANIP3_CH, 0, manip3PulseLen);
      displayImage(eyes_difficult);  
      state = LEAD;
      return false;

    case MANIP3_DOWN:
      manip3PulseLen = manip3PulseLen + SERVO_STEP;
      if (manip3PulseLen > servoManip3Max) manip3PulseLen = servoManip3Max;
      if (manip3PulseLen < servoManip3Min) manip3PulseLen = servoManip3Min;
      pwm.setPWM(SERVO_MANIP3_CH, 0, manip3PulseLen);
      displayImage(eyes_difficult);  
      state = LEAD;
      return false;

    case EXIT:
      manip1PulseLen = SERVO_CENTRAL_POSITION;
      manip2PulseLen = SERVO_CENTRAL_POSITION;
      manip3PulseLen = SERVO_CENTRAL_POSITION;
      state = LEAD;
      return true;
  }
}



void setup() 
{
  displaySetup(); // инициализация дисплея
  motorSetup();   // инициализация моторов
  servoSetup();   // инициализация серв
  servoCentering();   // центрирование серв
  
  pinMode(A4, INPUT_PULLUP);    // подтяжка линий I2C к питанию, мб и не надо 
  pinMode(A5, INPUT_PULLUP);

  //установка выводов и настроек: GamePad(clock, command, attention, data, Pressures?, Rumble?) проверка ошибок
  ps2x.config_gamepad(JOY_CLK_CH, JOY_CMD_CH, JOY_SEL_CH, JOY_DAT_CH, JOY_PRESSURES, JOY_RUMBLE); 

  readServoRange();   // чтение границ серв из епрома и запись в глобальные переменные

  //мелодия включения
#if VERSION == 11
  beep(NOTE_C, 400);
  beep(NOTE_E, 350);
  beep(NOTE_G, 150);
  beep(NOTE_B, 400);
  noTone(BUZZER_CH);
#endif
#if VERSION == 10
  beep(1, 500);
#endif

  analogReference(EXTERNAL);  // настройка опорного напряжения для АЦП: внешний источник на выводе AREF
  
  Serial.begin(9600);   // для отладки
}



void loop()
{
  static bool m_exit = false; // доп переменная для хранения данных о выходе из некоторых конечных автоматов
  static float voltage = 0;
  static float current = 0;
  static enum 
  {
    WORK,   // рабочий режим - ездит, кривляется
    CALIBRATION,  // режим калибровки
  } state;

  ps2x.read_gamepad(false, 0); // считывание данных с джойстика и установка скорости вибрации !!! (пока так)
  adcDataCounter(&voltage, &current); // обновляем донные с АЦП
  switch(state)
  {
    case WORK:
      m_exit = workFSM();   // крутимся в рабочем режиме, пока не придет флаг о выходе из него - вернется true
      if(m_exit)  state = CALIBRATION;
      m_exit = true;
      break;

    case CALIBRATION:
      m_exit = calibrationFSM();  // крутимся в режиме калибровки, пока не придет флаг о выходе из него - вернется true
      if(m_exit)  state = WORK;
      m_exit = false;
      break;    
  }
  delay(5);
}


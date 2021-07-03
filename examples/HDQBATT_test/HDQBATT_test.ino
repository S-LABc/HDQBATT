/*
 * HDQBATT_test
 *
 ** Пример использования функций бибилотеки HDQBATT.h для общения
 ** по интерфейсу HDQ с bq27546, который находится в АКБ от iPhone.
 * 
 * ########## !!! В А Ж Н О !!! ##########
 ** Если выводы не толерантны к 5В, НЕОБХОДИМО использовать 
 ** согласователь уровней, например TXS0108E(HW-221) или подобный.
 *
 ** Для корректной работы необходимо подключить резистор номиналом
 ** от 4.7кОм до 10кОм между контактом HDQ и +VBATT АКБ.
 * 
 ** Протокол HDQ реализован программно и использует стандартные функции Ардуино
 ** pinMode()
 ** digitalRead()
 ** digitalWrite()
 ** delayMicroseconds()
 *
 ** Если нужно получить данные из регистров, для которых нет готовых функций,
 ** то для этого есть доплнительные функции.
 *** readByte(reg) - прочитать один байт из одиного регистра 
 *** readWord(low_reg, high_reg) - прочитать пару байт из парного регистра
 *** readControlAddresses() - прочитать пару байт из регистра Control()
 *** writeByte(reg, payload) - записать один байт в один регистр
 *** writeWord(low_reg, high_reg, low_payload, high_payload) - записать пару байт в парный регистр
 *** writeControlAddresses(low_payload, high_payload) - записать пару байт в регистр Control()
 * 
 * Проект на GitHub - https://github.com/S-LABc/HDQBATT
 * 
 * Контакты:
 ** YouTube - https://www.youtube.com/channel/UCbkE52YKRphgkvQtdwzQbZQ
 ** Telegram - https://www.t.me/slabyt
 ** GitHub - https://github.com/S-LABc
 ** Gmail - romansklyar15@gmail.com
 * 
 * Copyright (C) 2021. v1.0 / Скляр Роман S-LAB
 */
 
// Подключаем библиотеку
#include <HDQBATT.h>

/* Создаем объект HDQ с указанием вывода микроконтроллера
 * STM32_HDQ_DEFAULT_PIN = PB12
 * ESP8266_HDQ_DEFAULT_PIN = D4 - СОГЛАСОВАТЕЛЬ
 * ESP32_HDQ_DEFAULT_PIN = 5 - СОГЛАСОВАТЕЛЬ
 * ARDUINO_BOARDS_HDQ_DEFAULT_PIN = 3
 * или какой-нибудь еще
 */
HDQBATT HDQ(STM32_HDQ_DEFAULT_PIN);

void setup() {
  /* 
   * Настройка таймингов протокола HDQ
   * Подробнее - 7.13 HDQ Communication Timing Characteristics
   */
  //HDQ.setTimeCYCH(HDQ_DELAY_CYCH_MIN); // 230
  //HDQ.setTimeCYCD(203); // 205
  //HDQ.setTimeHW1(35); // 40
  //HDQ.setTimeDW1(HDQ_DELAY_DW1_MIN); // 40
  //HDQ.setTimeHW0(99); // 120
  //HDQ.setTimeDW0(HDQ_DELAY_DW0_MAX); // 100
  //HDQ.setTimeRSPS(624); // 300
  //HDQ.setTimeB(115); // 200
  //HDQ.setTimeBR(76); // 90
  //HDQ.setTimeFailTries(HDQ_DELAY_FAIL_TRIES_MIN); // Не из даташита. 100 - 65535
  
  Serial.begin(115200);
  
  while (!HDQ.isConnected()) { // Пока не подключен АКБ
    // Выводим сообщение об отсутствии АКБ
    Serial.println("Battery not detected"); 
    delay(1000);
  }
}

void loop() {
  if (HDQ.isConnected()) { // Если АКБ подключен
    // Читаем данные и выводим в "Монитор порта"
    readBatteryData();
  }
  else {
    // Выводим сообщение об отсутствии АКБ
    Serial.println("Battery not detected");
  }
  
  delay(3000); // Проверяем раз в 3 секунды
}

void readBatteryData() {
  // Флаги регистра CONTROL_STATUS
  Serial.print("Flags Status: ");
  Serial.print("0x");
  Serial.print(HDQ.getControlStatus(), HEX); // hex значение регистра CONTROL_STATUS
  Serial.print(" SE="); // Вывод SE активен
  Serial.print(HDQ.getFlagSEPinIsActive());
  Serial.print("|FAS="); // Контроллер находится в состоянии FULL ACCESS SEALED 
  Serial.print(HDQ.getFlagIsFullAccessSealedMode());
  Serial.print("|SS="); // Контроллер находится в состоянии SEALED
  Serial.print(HDQ.getFlagIsSealedMode());
  Serial.print("|CALMODE="); // Функция калибровки активна
  Serial.print(HDQ.getFlagCalibrationFunctionIsActive());
  Serial.print("|CCA="); // Процедура CCA активна
  Serial.print(HDQ.getFlagCoulombCounterCalibrationRoutineIsActive());
  Serial.print("|BCA="); // Процедура калибровки платы активна
  Serial.print(HDQ.getFlagBoardCalibrationRoutineIsActive());
  Serial.print("|QMAXUPDATE="); // Изменилась QMAX
  Serial.print(HDQ.getFlagQMAXUpdate());
  Serial.print("|HOSTIE="); // Функция прерывания HDQ активна
  Serial.print(HDQ.getFlagHDQInterruptFunctionIsActive());
  Serial.print("|SHUTDWN="); // Команда SET_SHUTDOWN была отправлена
  Serial.print(HDQ.getFlagShutdownCommandIsSent());
  Serial.print("|HIBERNATE="); // Выдан запрос на переход в режим HIBERNATE
  Serial.print(HDQ.getFlagRequestHibernateFromSleepMode());
  Serial.print("|FULLSLEEP="); // Контроллер находится в режиме FULLSLEEP
  Serial.print(HDQ.getFlagIsFullSleepMode());
  Serial.print("|SLEEP="); // Контроллер находится в режиме SLEEP
  Serial.print(HDQ.getFlagIsSleepMode());
  Serial.print("|LDMD="); // Используется режим CONSTANT-POWER
  Serial.print(HDQ.getFlagImpedanceTrackAlgorithm());
  Serial.print("|RUP_DIS="); // Обновления таблицы Ra отключены
  Serial.print(HDQ.getFlagRaTableUpdatesDisabled());
  Serial.print("|VOK="); // Напряжения банки в порядке
  Serial.print(HDQ.getFlagCellVoltagesOK());
  Serial.print("|QEN="); // Обновления QMAX включены
  Serial.println(HDQ.getFlagQmaxUpdatesEnabled());
  
  // Флаги состояния Flags()
  Serial.print("Flags Definitions: ");
  Serial.print("0x");
  Serial.print(HDQ.getFlags(), HEX); // hex значение регистра Flags()
  Serial.print(" RSVD(5)="); // Так можно достать отдельныве биты
  Serial.print((HDQ.getFlags() >> BIT_DEFINITIONS_RSVD_5) & 1);
  Serial.print("|RSVD(4)="); // Так можно достать отдельныве биты
  Serial.print((HDQ.getFlags() >> BIT_DEFINITIONS_RSVD_4) & 1);
  Serial.print("|BATHI="); // Высокий уровень зяряда
  Serial.print(HDQ.getFlagBatteryHighIndicating());
  Serial.print("|BATLOW="); // Низкий уровень зяряда
  Serial.print(HDQ.getFlagBatteryLowIndicating());
  Serial.print("|CHG_INH="); // Запрет зарядки при высокой температуре
  Serial.print(HDQ.getFlagChargeInhibitindicates());
  Serial.print("|RSVD(3)="); // Так можно достать отдельныве биты
  Serial.print((HDQ.getFlags() >> BIT_DEFINITIONS_RSVD_3) & 1);
  Serial.print("|FC="); // Полностью зажен
  Serial.print(HDQ.getFlagFullChargedIsDetected());
  Serial.print("RSVD(2)="); // Так можно достать отдельныве биты
  Serial.print((HDQ.getFlags() >> BIT_DEFINITIONS_RSVD_2) & 1);
  Serial.print("|CHG_SUS="); // Приостановка зарядки
  Serial.print(HDQ.getFlagChargeSuspend());
  Serial.print("|RSVD(1)="); // Так можно достать отдельныве биты
  Serial.print((HDQ.getFlags() >> BIT_DEFINITIONS_RSVD_1) & 1);
  Serial.print("|RSVD(0)="); // Так можно достать отдельныве биты
  Serial.print((HDQ.getFlags() >> BIT_DEFINITIONS_RSVD_0) & 1);
  Serial.print("|IMAX="); // Готовность вычислений Imax()
  Serial.print(HDQ.getFlagIndicatesComputedImax());
  Serial.print("|CHG="); // Разрешена быстрая зарядка
  Serial.print(HDQ.getFlagChargingAllowed());
  Serial.print("|SOC1="); // Достугнут порг заряда SOC1
  Serial.print(HDQ.getFlagStateOfChargeThreshold1());
  Serial.print("|SOCF="); // Достугнут порг заряда SOCF
  Serial.print(HDQ.getFlagStateOfChargeThresholdFinal());
  Serial.print("|DSG="); // Обнаружена разрядка АКБ
  Serial.println(HDQ.getFlagDischargingDetected());

  // Модель контроллера
  Serial.print("Device Type: ");
  Serial.println(HDQ.getDeviceType(), HEX);
  
  // Версия прошивки
  Serial.print("Firmware Version: ");
  Serial.println(HDQ.getFirmwareVersion());

  // Версия оборудования
  Serial.print("Hardware Version: ");
  Serial.println(HDQ.getHardwareVersion());
  
  // Температура 
  Serial.print("Temperature: ");
  Serial.print(HDQ.getTemperatureKelvin()); // Кельвин
  Serial.print(" K | ");
  Serial.print(HDQ.getTemperatureCelsius()); // Цельсий
  Serial.print(" C | ");
  Serial.print(HDQ.getTemperatureFahrenheit()); //Фаренгейт
  Serial.println(" F");
  
  // Напряжение
  Serial.print("Voltage: ");
  Serial.print(HDQ.getVoltageMilli()); // Милливольты
  Serial.print(" mV | ");
  Serial.print(HDQ.getVoltage()); // Вольты
  Serial.println(" V");

  // Оставшийся заряд в АКБ
  Serial.print("Remaining Capacity: ");
  Serial.print(HDQ.getRemainingCapacity());
  Serial.println(" mAh");

  // Оставшаяся емкость АКБ
  Serial.print("Full Charge Capacity: ");
  Serial.print(HDQ.getFullChargeCapacity());
  Serial.println(" mAh");
  
  // Ток при зарядке/разрядке
  Serial.print("Average Current: ");
  Serial.print(HDQ.getAverageCurrentMilli()); // Миллиамперы
  Serial.print(" mA | ");
  Serial.print(HDQ.getAverageCurrent()); // Амперы
  Serial.println(" A");
  
  // Время до полной разрядки
  Serial.print("Time To Empty: ");
  Serial.print(HDQ.getTimeToEmpty());
  Serial.println(" min");
  
  // Мощность при зарядке/разрядке
  Serial.print("Average Power: ");
  Serial.print(HDQ.getAveragePowerMilli()); // Милливатты
  Serial.print(" mW | ");
  Serial.print(HDQ.getAveragePower()); // Ватты
  Serial.println(" W");
  
  // Количество циклов зарядок/разрядок
  Serial.print("Cycles: ");
  Serial.print(HDQ.getCycleCount());
  Serial.println(" count");
  
  // Уровень заряда
  Serial.print("State Of Charge: ");
  Serial.print(HDQ.getStateOfCharge());
  Serial.println(" %");
  
  // Заводская емкость
  Serial.print("Design Capacity: ");
  Serial.print(HDQ.getDesignCapacity());
  Serial.println(" mAh");

  // Блоки информации о производителе 
  Serial.print("Manufacturer Info Block A: ");
  Serial.println(HDQ.getManufacturerInfoBlockA());
  Serial.print("Manufacturer Info Block B: ");
  Serial.println(HDQ.getManufacturerInfoBlockB());
  Serial.print("Manufacturer Info Block C: ");
  Serial.println(HDQ.getManufacturerInfoBlockC());

  // Контрольная сумма блока данных
  Serial.print("Block Data Checksum: ");
  Serial.println(HDQ.getBlockDataChecksum(), HEX);

  Serial.println();
}

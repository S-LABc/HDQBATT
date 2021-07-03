#include "HDQBATT.h"

// ########## CONSTRUCTOR ##########
HDQBATT::HDQBATT(uint8_t pin) { 
  _pin = pin;
}
// ########## PRIVATE ##########
/*
 * doBreak: сигнал сброса на линии данных
 */
void HDQBATT::doBreak(void) {
  // Настроить вывод на выход
  pinMode(_pin, OUTPUT);

  // Сигнал сброса
  digitalWrite(_pin, LOW);
  delayMicroseconds(hdq_delay.t_b);

  // Отпустить вывод
  pinMode(_pin, INPUT);
  delayMicroseconds(hdq_delay.t_br);
}
/*
 * sendByteBitByBit: передача одного байта в контроллер АКБ побитово
 * @param payload: полезные данные
 */
void HDQBATT::sendByteBitByBit(uint8_t payload) {
  // Настроить вывод на выход
  pinMode(_pin, OUTPUT);

  for (uint8_t i = 0; i < 8; i++) {
    // Стартовый бит
    digitalWrite(_pin, LOW);
    delayMicroseconds(hdq_delay.t_hw1);

    // LSB
    if (payload>>i & 0x01) {
        digitalWrite(_pin, HIGH);
    }
    else {
        digitalWrite(_pin, LOW);
    }
    delayMicroseconds(hdq_delay.t_hw0 - hdq_delay.t_hw1);

    // Стоповый бит
    digitalWrite(_pin, HIGH);
    delayMicroseconds(hdq_delay.t_cych - hdq_delay.t_hw0);
  }

  // Отпустить вывод
  pinMode(_pin, INPUT);
  delayMicroseconds(hdq_delay.t_cych - hdq_delay.t_hw0);
}
/*
 * pullBlockAuthenticate: получить данные из регистров 0x40...0x5F
 * @param str[]: массив для хранения считанных данных
 */
void HDQBATT::pullBlockData(uint8_t str[]) {
  uint8_t count = 0;
  for(uint8_t i = EXTD_CMD_BLOCK_DATA_L; i <= EXTD_CMD_BLOCK_DATA_H; i ++) {
    str[count] = HDQBATT::readByte(i);
    count ++;
  }
}
// ########## PUBLIC ##########
/*
 * setTimeCYCH: установить новое время задержки t(CYCH)
 * @param new_delay: время задержки 
 * @return: логическое значение bool
 */
bool HDQBATT::setTimeCYCH(uint8_t new_delay) {
  if (new_delay < HDQ_DELAY_CYCH_MIN || new_delay > HDQ_DELAY_CYCH_MAX) {
	return false;
  }
	
  hdq_delay.t_cych = new_delay;
	
  return true;
}
/*
 * setTimeCYCD: установить новое время задержки t(CYCD)
 * @param new_delay: время задержки 
 * @return: логическое значение bool
 */
bool HDQBATT::setTimeCYCD(uint8_t new_delay) {
  if (new_delay < HDQ_DELAY_CYCD_MIN || new_delay > HDQ_DELAY_CYCD_MAX) {
	return false;
  }
	
  hdq_delay.t_cycd = new_delay;
	
  return true;
}
/*
 * setTimeHW1: установить новое время задержки t(HW1)
 * @param new_delay: время задержки 
 * @return: логическое значение bool
 */
bool HDQBATT::setTimeHW1(uint8_t new_delay) {
  if (new_delay < HDQ_DELAY_HW1_MIN || new_delay > HDQ_DELAY_HW1_MAX) {
	return false;
  }
	
  hdq_delay.t_hw1 = new_delay;
	
  return true;
}
/*
 * setTimeDW1: установить новое время задержки t(DW1)
 * @param new_delay: время задержки 
 * @return: логическое значение bool
 */
bool HDQBATT::setTimeDW1(uint8_t new_delay) {
  if (new_delay < HDQ_DELAY_DW1_MIN || new_delay > HDQ_DELAY_DW1_MAX) {
	return false;
  }
	
  hdq_delay.t_dw1 = new_delay;
	
  return true;
}
/*
 * setTimeHW0: установить новое время задержки t(HW0)
 * @param new_delay: время задержки 
 * @return: логическое значение bool
 */
bool HDQBATT::setTimeHW0(uint8_t new_delay) {
  if (new_delay < HDQ_DELAY_HW0_MIN || new_delay > HDQ_DELAY_HW0_MAX) {
	return false;
  }
	
  hdq_delay.t_hw0 = new_delay;
	
  return true;
}
/*
 * setTimeDW0: установить новое время задержки t(DW0)
 * @param new_delay: время задержки 
 * @return: логическое значение bool
 */
bool HDQBATT::setTimeDW0(uint8_t new_delay) {
  if (new_delay < HDQ_DELAY_DW0_MIN || new_delay > HDQ_DELAY_DW0_MAX) {
	return false;
  }
	
  hdq_delay.t_dw0 = new_delay;
	
  return true;
}
/*
 * setTimeRSPS: установить новое время задержки t(RSPS)
 * @param new_delay: время задержки 
 * @return: логическое значение bool
 */
bool HDQBATT::setTimeRSPS(uint16_t new_delay) {
  if (new_delay < HDQ_DELAY_RSPS_MIN || new_delay > HDQ_DELAY_RSPS_MAX) {
	return false;
  }
	
  hdq_delay.t_rsps = new_delay;
	
  return true;
}
/*
 * setTimeB: установить новое время задержки t(B)
 * @param new_delay: время задержки 
 * @return: логическое значение bool
 */
bool HDQBATT::setTimeB(uint16_t new_delay) {
  if (new_delay < HDQ_DELAY_B_MIN) {
	return false;
  }
	
  hdq_delay.t_b = new_delay;
	
  return true;
}
/*
 * setTimeBR: установить новое время задержки t(BR)
 * @param new_delay: время задержки 
 * @return: логическое значение bool
 */
bool HDQBATT::setTimeBR(uint16_t new_delay) {
  if (new_delay < HDQ_DELAY_BR_MIN) {
    return false;
  }
	
  hdq_delay.t_br = new_delay;
	
  return true;
}
/*
 * setTimeFailTries: установить новое время задержки FailTries (она не из даташита)
 * @param new_delay: время задержки 
 * @return: логическое значение bool
 */
bool HDQBATT::setTimeFailTries(uint16_t new_delay) {
  if (new_delay < HDQ_DELAY_RSPS_MIN) {
    return false;
  }
	
  hdq_delay.t_fail_tries = new_delay;
	
  return true;
}
/*
 * readByte: прочитать один байт из регистра
 * @param reg: адрес регистра для чтения 
 * @return: целое число размером один байт uint8_t
 */
uint8_t HDQBATT::readByte(uint8_t reg) {
  uint8_t result = 0; 
  uint8_t maxTries = hdq_delay.t_fail_tries;
  
  // Сброс
  HDQBATT::doBreak();
  
  // Пишем в регистр чтобы прочитать данные
  HDQBATT::sendByteBitByBit((reg |= HDQ_ADDR_MASK_READ));
  
  for (uint8_t i = 0; i < 8; i++) {
    maxTries = hdq_delay.t_fail_tries;
    
    // Ожидаем пока контроллер АКБ соизволит ответить
    while (digitalRead(_pin) != 0 && maxTries-- > 0) {
      if (maxTries == 1) {
        return 0xFF;
      }
    }
    
    delayMicroseconds(((hdq_delay.t_dw0 - hdq_delay.t_dw1) / 2) + hdq_delay.t_dw1);
    
    // Читаем бит
    result |= digitalRead(_pin) << i;
    delayMicroseconds(hdq_delay.t_cycd - hdq_delay.t_dw0);
  }

  delayMicroseconds(hdq_delay.t_b);

  return result;
}
/*
 * readWord: прочитать пару регистров из контроллера АКБ
 * @param low_byte: младший байт регистра
 * @param high_byte: старший байт регистра
 * @return: целое число размером два байта uint16_t
 */
uint16_t HDQBATT::readWord(uint8_t low_reg, uint8_t high_reg) {
  return (HDQBATT::readByte(high_reg) << 8) | HDQBATT::readByte(low_reg);
}
/*
 * readControlAddresses: прочитать регистр Control()
 * @return: целое число размером два байта uint16_t
 */
uint16_t HDQBATT::readControlAddresses(void) {
  return HDQBATT::readWord(CMD_CONTROL_L, CMD_CONTROL_H);
}
/*
 * writeByte: отправить один байт полезных данных в регистр контроллера АКБ
 * @param reg: адрес регистра для записи
 * @param payload: полезные данные
 */
void HDQBATT::writeByte(uint8_t reg, uint8_t payload) { 
  // Сброс
  HDQBATT::doBreak();
  
  // Передача адреса
  HDQBATT::sendByteBitByBit((reg |= HDQ_ADDR_MASK_WRITE));
  delayMicroseconds(hdq_delay.t_rsps);
  
  // Передача данных
  HDQBATT::sendByteBitByBit(payload);
  delayMicroseconds(hdq_delay.t_rsps);

  // Отпустить вывод
  pinMode(_pin, INPUT);
}
/*
 * writeWord: отправить 2 байта данных в регистр контроллера АКБ
 * @param low_reg: младший байт регистра
 * @param high_reg: старший байт регистра
 * @param low_payload: младший байт полезных данных
 * @param high_payload: старший байт полезных данных
 */
void HDQBATT::writeWord(uint8_t low_reg, uint8_t high_reg, uint8_t low_payload, uint8_t high_payload) {
  HDQBATT::writeByte(low_reg, high_payload);
  HDQBATT::writeByte(high_reg, low_payload);
}
/*
 * writeControlAddresses: записать 2 байта в регистр Control()
 * @param low_payload: младший байт полезных данных
 * @param high_payload: старший байт полезных данных
 */
void HDQBATT::writeControlAddresses(uint8_t low_payload, uint8_t high_payload) {
  HDQBATT::writeWord(CMD_CONTROL_L, CMD_CONTROL_H, low_payload, high_payload);
}
/*
 * isConnected: узнать подкючен ли АКБ. Используется проверка модели контроллера, ибо она точно должна быть
 * @return: логическое значение bool
 */
bool HDQBATT::isConnected(void) {
  return (HDQBATT::getDeviceType() != 0xFFFF) ? true : false;
}
/* 
 * getControlStatus: получить флаги из регистра CONTROL_STATUS АКБ
 * @return: целое число размером два байта uint16_t
 */
uint16_t HDQBATT::getControlStatus(void) {
  HDQBATT::writeControlAddresses(CNTL_CONTROL_STATUS_L, CNTL_CONTROL_STATUS_H);
  return HDQBATT::readControlAddresses();
}
/* 
 * getFlagSEPinIsActive: получить Бит состояния, указывающий, что вывод SE активен
 * @return: логическое значение bool
 */
bool HDQBATT::getFlagSEPinIsActive(void) {
  return (HDQBATT::getControlStatus() >> CONTROL_STATUS_SE) & 1;
}
/* 
 * getFlagIsFullAccessSealedMode: получить Бит состояния, указывающий, что контроллер находится в состоянии FULL ACCESS SEALED
 * @return: логическое значение bool
 */
bool HDQBATT::getFlagIsFullAccessSealedMode(void) {
  return (HDQBATT::getControlStatus() >> CONTROL_STATUS_FAS) & 1;
}
/* 
 * getFlagIsSealedMode: получить Бит состояния, указывающий, что контроллер находится в состоянии SEALED
 * @return: логическое значение bool
 */
bool HDQBATT::getFlagIsSealedMode(void) {
  return (HDQBATT::getControlStatus() >> CONTROL_STATUS_SS) & 1;
}
/* 
 * getFlagCalibrationFunctionIsActive: получить Бит состояния, указывающий, что функция калибровки активна
 * @return: логическое значение bool
 */
bool HDQBATT::getFlagCalibrationFunctionIsActive(void) {
  return (HDQBATT::getControlStatus() >> CONTROL_STATUS_CALMODE) & 1;
}
/* 
 * getFlagCoulombCounterCalibrationRoutineIsActive: получить Бит состояния, указывающий, что активна процедура CCA
 * @return: логическое значение bool
 */
bool HDQBATT::getFlagCoulombCounterCalibrationRoutineIsActive(void) {
  return (HDQBATT::getControlStatus() >> CONTROL_STATUS_CCA) & 1;
}
/* 
 * getFlagBoardCalibrationRoutineIsActive: получить Бит состояния, указывающий, что процедура калибровки платы активна
 * @return: логическое значение bool
 */
bool HDQBATT::getFlagBoardCalibrationRoutineIsActive(void) {
  return (HDQBATT::getControlStatus() >> CONTROL_STATUS_BCA) & 1;
}
/* 
 * getFlagQMAXUpdate: получить Бит состояния, переключаемый при измении состояния QMAX
 * @return: логическое значение bool
 */
bool HDQBATT::getFlagQMAXUpdate(void) {
  return (HDQBATT::getControlStatus() >> CONTROL_STATUS_QMAXUPDATE) & 1;
}
/* 
 * getFlagHDQInterruptFunctionIsActive: получить Бит состояния, указывающий, что функция прерывания HDQ активна
 * @return: логическое значение bool
 */
bool HDQBATT::getFlagHDQInterruptFunctionIsActive(void) {
  return (HDQBATT::getControlStatus() >> CONTROL_STATUS_HOSTIE) & 1;
}
/* 
 * getFlagShutdownCommandIsSent: получить управляющий Бит, указывающий, что команда SET_SHUTDOWN была отправлена
 * @return: логическое значение bool
 */
bool HDQBATT::getFlagShutdownCommandIsSent(void) {
  return (HDQBATT::getControlStatus() >> CONTROL_STATUS_SHUTDWN) & 1;
}
/* 
 * getFlagRequestHibernateFromSleepMode: получить Бит состояния, указывающий, что запрос на переход в режим HIBERNATE из режима SLEEP был выдан
 * @return: логическое значение bool
 */
bool HDQBATT::getFlagRequestHibernateFromSleepMode(void) {
  return (HDQBATT::getControlStatus() >> CONTROL_STATUS_HIBERNATE) & 1;
}
/* 
 * getFlagIsFullSleepMode: получить Бит состояния, указывающий, что контроллер находится в режиме FULLSLEEP
 * @return: логическое значение bool
 */
bool HDQBATT::getFlagIsFullSleepMode(void) {
  return (HDQBATT::getControlStatus() >> CONTROL_STATUS_FULLSLEEP) & 1;
}
/* 
 * getFlagIsSleepMode: получить Бит состояния, указывающий, что контроллер находится в режиме SLEEP
 * @return: логическое значение bool
 */
bool HDQBATT::getFlagIsSleepMode(void) {
  return (HDQBATT::getControlStatus() >> CONTROL_STATUS_SLEEP) & 1;
}
/* 
 * getFlagImpedanceTrackAlgorithm: получить Бит состояния, указывающий, что алгоритм отслеживания импеданса использует режим CONSTANT-POWER
 * @return: логическое значение bool
 */
bool HDQBATT::getFlagImpedanceTrackAlgorithm(void) {
  return (HDQBATT::getControlStatus() >> CONTROL_STATUS_LDMD) & 1;
}
/* 
 * getFlagRaTableUpdatesDisabled: получить Бит состояния, указывающий, что обновления таблицы Ra отключены
 * @return: логическое значение bool
 */
bool HDQBATT::getFlagRaTableUpdatesDisabled(void) {
  return (HDQBATT::getControlStatus() >> CONTROL_STATUS_RUP_DIS) & 1;
}
/* 
 * getFlagCellVoltagesOK: получить Бит состояния, указывающий, что напряжения банки в порядке
 * @return: логическое значение bool
 */
bool HDQBATT::getFlagCellVoltagesOK(void) {
  return (HDQBATT::getControlStatus() >> CONTROL_STATUS_VOK) & 1;
}
/* 
 * getFlagQmaxUpdatesEnabled: получить Бит состояния, указывающий, что обновления Qmax включены
 * @return: логическое значение bool
 */
bool HDQBATT::getFlagQmaxUpdatesEnabled(void) {
  return (HDQBATT::getControlStatus() >> CONTROL_STATUS_QEN) & 1;
}
/* 
 * getDeviceType: получить модель контроллера АКБ в шестнадцатиричном виде (модель: bq27546, результат: 0x0546)
 * @return: целое число размером два байта uint16_t
 */
uint16_t HDQBATT::getDeviceType(void) {
  HDQBATT::writeControlAddresses(CNTL_DEVICE_TYPE_L, CNTL_DEVICE_TYPE_H);
  return HDQBATT::readControlAddresses();
}
/* 
 * getFirmwareVersion: получить версию прошивки контроллера АКБ
 * @return: число с плавающей точкой float
 */
float HDQBATT::getFirmwareVersion(void) {
  HDQBATT::writeControlAddresses(CNTL_FW_VERSION_L, CNTL_FW_VERSION_H);
  float f = readByte(CMD_CONTROL_L) * 0.01;
  return readByte(CMD_CONTROL_H) + f;
}
/* 
 * getHardwareVersion: получить версию оборудования контроллера АКБ
 * @return: число с плавающей точкой float
 */
float HDQBATT::getHardwareVersion(void) {
  HDQBATT::writeControlAddresses(CNTL_HW_VERSION_L, CNTL_HW_VERSION_H);
  float f = readByte(CMD_CONTROL_L) * 0.01;
  return readByte(CMD_CONTROL_H) + f;
}
/* 
 * getTemperatureKelvin: получить температуру АКБ в градусах Кельвина
 * @return: число с плавающей точкой float
 */
float HDQBATT::getTemperatureKelvin(void) {
  return HDQBATT::readWord(CMD_TEMPERATURE_L, CMD_TEMPERATURE_H) * 0.1;
}
/* 
 * getTemperatureCelsius: получить температуру АКБ в градусах Цельсия
 * @return: число с плавающей точкой float
 */
float HDQBATT::getTemperatureCelsius(void) {  
  return HDQBATT::getTemperatureKelvin() - 273.1;
}
/* 
 * getTemperatureFahrenheit: получить температуру АКБ в градусах Фаренгейта
 * @return: число с плавающей точкой float
 */
float HDQBATT::getTemperatureFahrenheit(void) {
  return HDQBATT::getTemperatureCelsius() * 1.8 + 32;
}
/* 
 * getVoltageMilli: получить напряжение на клемах АКБ в милливольтах
 * @return: целое число размером два байта unsigned short
 */
unsigned short HDQBATT::getVoltageMilli(void) {  
  return HDQBATT::readWord(CMD_VOLTAGE_L, CMD_VOLTAGE_H);
}
/* 
 * getVoltage: получить напряжение на клемах АКБ в вольтах
 * @return: число с плавающей точкой float
 */
float HDQBATT::getVoltage(void) {
  return HDQBATT::getVoltageMilli() * 0.001;
}
/* 
 * getFlags: получить флаги из регистра FLAGS АКБ
 * @return: целое число размером два байта uint16_t
 */
uint16_t HDQBATT::getFlags(void) {
  return HDQBATT::readWord(CMD_FLAGS_L, CMD_FLAGS_H);
}
/* 
 * getFlagBatteryHighIndicating: получить Бит высокого уровня заряда
 * @return: логическое значение bool
 */
bool HDQBATT::getFlagBatteryHighIndicating(void) {  
  return (HDQBATT::getFlags() >> BIT_DEFINITIONS_BATHI) & 1;
}
/* 
 * getFlagBatteryLowIndicating: получить Бит низкого уровня заряда
 * @return: логическое значение bool
 */
bool HDQBATT::getFlagBatteryLowIndicating(void) {  
  return (HDQBATT::getFlags() >> BIT_DEFINITIONS_BATLOW) & 1;
}
/* 
 * getFlagChargeInhibitindicates: получить Бит запрета зарядки при высокой температуре
 * @return: логическое значение bool
 */
bool HDQBATT::getFlagChargeInhibitindicates(void) {  
  return (HDQBATT::getFlags() >> BIT_DEFINITIONS_CHG_INH) & 1;
}
/* 
 * getFlagFullChargedIsDetected: получить Бит полностью заряжен
 * @return: логическое значение bool
 */
bool HDQBATT::getFlagFullChargedIsDetected(void) {  
  return (HDQBATT::getFlags() >> BIT_DEFINITIONS_FC) & 1;
}
/* 
 * getFlagChargeSuspend: получить Бит приостановки зарядки
 * @return: логическое значение bool
 */
bool HDQBATT::getFlagChargeSuspend(void) {  
  return (HDQBATT::getFlags() >> BIT_DEFINITIONS_CHG_SUS) & 1;
}
/* 
 * getFlagIndicatesComputedImax: получить Бит готовности вычисления Imax()
 * @return: логическое значение bool
 */
bool HDQBATT::getFlagIndicatesComputedImax(void) {  
  return (HDQBATT::getFlags() >> BIT_DEFINITIONS_IMAX) & 1;
}
/* 
 * getFlagChargingAllowed: получить Бит разрешенной быстрой зарядки
 * @return: логическое значение bool
 */
bool HDQBATT::getFlagChargingAllowed(void) {  
  return (HDQBATT::getFlags() >> BIT_DEFINITIONS_CHG) & 1;
}
/* 
 * getFlagStateOfChargeThreshold1: получить Бит достижения порога заряда SOC1
 * @return: логическое значение bool
 */
bool HDQBATT::getFlagStateOfChargeThreshold1(void) {  
  return (HDQBATT::getFlags() >> BIT_DEFINITIONS_SOC1) & 1;
}
/* 
 * getFlagStateOfChargeThresholdFinal: получить Бит достижения порога заряда SOCF
 * @return: логическое значение bool
 */
bool HDQBATT::getFlagStateOfChargeThresholdFinal(void) {  
  return (HDQBATT::getFlags() >> BIT_DEFINITIONS_SOCF) & 1;
}
/* 
 * getFlagDischargingDetected: получить Бит обнаружения разрядки АКБ
 * @return: логическое значение bool
 */
bool HDQBATT::getFlagDischargingDetected(void) {  
  return (HDQBATT::getFlags() >> BIT_DEFINITIONS_DSG) & 1;
}
/* 
 * getRemainingCapacity: получить скомпенсированный оставшийся заряд в АКБ в миллиампер-часах
 * @return: целое число размером два байта unsigned short
 */
unsigned short HDQBATT::getRemainingCapacity(void) {
  return HDQBATT::readWord(CMD_REMAINING_CAPACITY_L, CMD_REMAINING_CAPACITY_H);
}
/* 
 * getFullChargeCapacity: получить скомпенсированнаую оставшуюся емкость АКБ в миллиампер-часах
 * @return: целое число размером два байта unsigned short
 */
unsigned short HDQBATT::getFullChargeCapacity(void) {
  return HDQBATT::readWord(CMD_FULL_CHARGE_CAPACITY_L, CMD_FULL_CHARGE_CAPACITY_H);
}
/* 
 * getAverageCurrentMilli: получить ток зарядки/разрядки АКБ в миллиамперах
 * @return: число со знаком размером два байта short
 */
short HDQBATT::getAverageCurrentMilli(void) {
  return HDQBATT::readWord(CMD_AVERAGE_CURRENT_L, CMD_AVERAGE_CURRENT_H);
}
/* 
 * getAverageCurrent: получить ток зарядки/разрядки АКБ в амперх
 * @return: число с плавающей точкой float
 */
float HDQBATT::getAverageCurrent(void) {
  return HDQBATT::getAverageCurrentMilli() * 0.001;
}
/* 
 * getTimeToEmpty: получить время до полного разряда в минутах
 * @return: целое число размером два байта unsigned short
 */
unsigned short HDQBATT::getTimeToEmpty(void) {
  return HDQBATT::readWord(CMD_TIME_TO_EMPTY_L, CMD_TIME_TO_EMPTY_H);
}
/* 
 * getAveragePowerMilli: получить потребляемую мощность при зарядке/разрядке в милливаттах
 * @return: целое число размером два байта unsigned short
 */
short HDQBATT::getAveragePowerMilli(void) {
  return HDQBATT::readWord(CMD_AVERAGE_POWER_L, CMD_AVERAGE_POWER_H);
}
/* 
 * getAveragePower: получить потребляемую мощность при зарядке/разрядке в ваттах
 * @return: число с плавающей точкой float
 */
float HDQBATT::getAveragePower(void) {
  return HDQBATT::getAveragePowerMilli() * 0.001;
}
/* 
 * getCycleCount: получить количество циклов перезарядки АКБ в штуках
 * @return: целое число размером два байта unsigned short
 */
unsigned short HDQBATT::getCycleCount(void) {
  return HDQBATT::readWord(CMD_CYCLE_COUNT_L, CMD_CYCLE_COUNT_H);
}
/* 
 * getStateOfCharge: получить оставшийся заряд АКБ в процентах
 * @return: целое число размером один байт byte
 */
byte HDQBATT::getStateOfCharge(void) {
  return (HDQBATT::readWord(CMD_STATE_OF_CHARGE_L, CMD_STATE_OF_CHARGE_H));
}
/* 
 * getDesignCapacity: получить емкость АКБ установленную на заводе в миллиампер-часах
 * @return: целое число размером два байта unsigned short
 */
unsigned short HDQBATT::getDesignCapacity(void) {
  return HDQBATT::readWord(EXTD_CMD_DESIGN_CAPACITY_L, EXTD_CMD_DESIGN_CAPACITY_H);
}
/*
 * getManufacturerInfoBlockA: получить информацию о производителе строкой. Блок A
 * @return: массив символов (строка)
 */
char* HDQBATT::getManufacturerInfoBlockA(void) {
  HDQBATT::writeByte(EXTD_CMD_DATA_FLASH_BLOCK, EXTD_CMD_MANUFACTURE_BLOCK_A);
  HDQBATT::pullBlockData(_block_data);
  return (char*)_block_data;
}
/*
 * getManufacturerInfoBlockB: получить информацию о производителе строкой. Блок B
 * @return: массив символов (строка)
 */
char* HDQBATT::getManufacturerInfoBlockB(void) {
  HDQBATT::writeByte(EXTD_CMD_DATA_FLASH_BLOCK, EXTD_CMD_MANUFACTURE_BLOCK_B);
  HDQBATT::pullBlockData(_block_data);
  return (char*)_block_data;
}
/*
 * getManufacturerInfoBlockC: получить информацию о производителе строкой. Блок C
 * @return: массив символов (строка)
 */
char* HDQBATT::getManufacturerInfoBlockC(void) {
  HDQBATT::writeByte(EXTD_CMD_DATA_FLASH_BLOCK, EXTD_CMD_MANUFACTURE_BLOCK_C);
  HDQBATT::pullBlockData(_block_data);
  return (char*)_block_data;
}
/*
 * getBlockDataChecksum: получить контрольную сумму блока данных в диапазоне 0x40...0x5f в шестнадцатиричном виде
 * @return: целое число размером один байт uint8_t
 */
uint8_t HDQBATT::getBlockDataChecksum(void) {
  return (HDQBATT::readByte(EXTD_CMD_BLOCK_DATA_CHEKSUM));
}

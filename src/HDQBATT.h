/*
 * Библиотека позволяет получить доступ к некоторым регистрам
 ** контроллера bq27546 от компании Texas Instruments по интерфейсу HDQ.
 ** Можно подключится и к другим контроллерам семейства bq2754x
 * 
 ** Протокол HDQ реализован программно и использует стандартные функции Ардуино
 ** Функции:
 ** pinMode()
 ** digitalRead()
 ** digitalWrite()
 ** delayMicroseconds()
 * 
 * Исходный проект:
 ** автор - mozzwald https://github.com/mozzwald
 ** библиотека - https://github.com/mozzwald/HDQLib
 ** пример - https://github.com/mozzwald/hdq-batt-status
 * 
 * Сопроводительная документация:
 ** https://www.ti.com/lit/an/slua408a/slua408a.pdf
 ** https://www.ti.com/lit/an/slva101/slva101.pdf
 ** https://www.ti.com/lit/an/slua917/slua917.pdf
 ** https://www.ti.com/lit/an/slua503/slua503.pdf
 ** https://www.ti.com/lit/ug/sluub74/sluub74.pdf
 ** https://www.ti.com/lit/ds/symlink/bq27541.pdf
 ** https://www.ti.com/lit/ds/symlink/bq27545-g1.pdf
 ** https://www.ti.com/lit/ds/symlink/bq27425-g2a.pdf
 ** https://www.ti.com/lit/ds/symlink/bq27546-g1.pdf
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

#pragma once
#include <Arduino.h>

// Выводы по умолчанию для разных плат (зависит от ядра)
#define STM32_HDQ_DEFAULT_PIN PB12
#define ESP8266_HDQ_DEFAULT_PIN D4 // Только с использованием согласователя уровней!
#define ESP32_HDQ_DEFAULT_PIN 4 // Только с использованием согласователя уровней!
#define ARDUINO_BOARDS_HDQ_DEFAULT_PIN 3

#define HDQ_ADDR_MASK_READ 0x00
#define HDQ_ADDR_MASK_WRITE 0x80 // b10000000

#define HDQ_DELAY_CYCH_MIN 190
#define HDQ_DELAY_CYCH_MAX 250
#define HDQ_DELAY_CYCH_NOM 230

#define HDQ_DELAY_CYCD_MIN 190
#define HDQ_DELAY_CYCD_MAX 250
#define HDQ_DELAY_CYCD_NOM 205

#define HDQ_DELAY_HW1_MIN 1
#define HDQ_DELAY_HW1_MAX 50
#define HDQ_DELAY_HW1_NOM 40

#define HDQ_DELAY_DW1_MIN 32
#define HDQ_DELAY_DW1_MAX 50
#define HDQ_DELAY_DW1_NOM 40

#define HDQ_DELAY_HW0_MIN 86
#define HDQ_DELAY_HW0_MAX 145
#define HDQ_DELAY_HW0_NOM 120

#define HDQ_DELAY_DW0_MIN 80
#define HDQ_DELAY_DW0_MAX 145
#define HDQ_DELAY_DW0_NOM 100

#define HDQ_DELAY_RSPS_MIN 190
#define HDQ_DELAY_RSPS_MAX 950
#define HDQ_DELAY_RSPS_NOM 300

#define HDQ_DELAY_B_MIN 190
#define HDQ_DELAY_B_NOM 200

#define HDQ_DELAY_BR_MIN 40
#define HDQ_DELAY_BR_NOM 90

#define HDQ_DELAY_FAIL_TRIES_MIN 100
#define HDQ_DELAY_FAIL_TRIES_NOM 255 // Ожидание ответа контроллера АКБ

#pragma pack(push,1)
typedef struct {
	byte t_cych = HDQ_DELAY_CYCH_NOM;
	byte t_cycd = HDQ_DELAY_CYCD_NOM;
	byte t_hw1 = HDQ_DELAY_HW1_NOM;
	byte t_dw1 = HDQ_DELAY_DW1_NOM;
	byte t_hw0 = HDQ_DELAY_HW0_NOM;
	byte t_dw0 = HDQ_DELAY_DW0_NOM;
	short t_rsps = HDQ_DELAY_RSPS_NOM;
	short t_b = HDQ_DELAY_B_NOM;
	short t_br = HDQ_DELAY_BR_NOM;
	short t_fail_tries = HDQ_DELAY_FAIL_TRIES_NOM;
} HDQDelay;
#pragma pack(pop)

#define CMD_CONTROL_L 0x00
#define CMD_CONTROL_H 0x01

/* Table 13. CONTROL_STATUS Flags */
#define CNTL_CONTROL_STATUS_L 0x00
#define CNTL_CONTROL_STATUS_H 0x00
enum ControlStatusFlags {
  CONTROL_STATUS_QEN,
  CONTROL_STATUS_VOK,
  CONTROL_STATUS_RUP_DIS,
  CONTROL_STATUS_LDMD,
  CONTROL_STATUS_SLEEP,
  CONTROL_STATUS_FULLSLEEP,
  CONTROL_STATUS_HIBERNATE,
  CONTROL_STATUS_SHUTDWN,
  CONTROL_STATUS_HOSTIE,
  CONTROL_STATUS_QMAXUPDATE,
  CONTROL_STATUS_BCA,
  CONTROL_STATUS_CCA,
  CONTROL_STATUS_CALMODE,
  CONTROL_STATUS_SS,
  CONTROL_STATUS_FAS,
  CONTROL_STATUS_SE,
};

#define CNTL_DEVICE_TYPE_L 0x00
#define CNTL_DEVICE_TYPE_H 0x01

#define CNTL_FW_VERSION_L 0x00
#define CNTL_FW_VERSION_H 0x02

#define CNTL_HW_VERSION_L 0x00
#define CNTL_HW_VERSION_H 0x03

#define CMD_TEMPERATURE_L 0x06
#define CMD_TEMPERATURE_H 0x07

#define CMD_VOLTAGE_L 0x08
#define CMD_VOLTAGE_H 0x09

/* Table 14. Flags Bit Definitions */
#define CMD_FLAGS_L 0x0A
#define CMD_FLAGS_H 0x0B
enum FlagsBitDefinitions {
  BIT_DEFINITIONS_DSG,
  BIT_DEFINITIONS_SOCF,
  BIT_DEFINITIONS_SOC1,
  BIT_DEFINITIONS_CHG,
  BIT_DEFINITIONS_IMAX,
  BIT_DEFINITIONS_RSVD_0, // не задействован
  BIT_DEFINITIONS_RSVD_1, // не задействован
  BIT_DEFINITIONS_CHG_SUS,
  BIT_DEFINITIONS_RSVD_2, // не задействован
  BIT_DEFINITIONS_FC,
  BIT_DEFINITIONS_RSVD_3, // не задействован
  BIT_DEFINITIONS_CHG_INH,
  BIT_DEFINITIONS_BATLOW,
  BIT_DEFINITIONS_BATHI,
  BIT_DEFINITIONS_RSVD_4, // не задействован
  BIT_DEFINITIONS_RSVD_5, // не задействован
};

#define CMD_REMAINING_CAPACITY_L 0x10
#define CMD_REMAINING_CAPACITY_H 0x11

#define CMD_FULL_CHARGE_CAPACITY_L 0x12
#define CMD_FULL_CHARGE_CAPACITY_H 0x13

#define CMD_AVERAGE_CURRENT_L 0x14
#define CMD_AVERAGE_CURRENT_H 0x15

#define CMD_TIME_TO_EMPTY_L 0x16
#define CMD_TIME_TO_EMPTY_H 0x17

#define CMD_AVERAGE_POWER_L 0x24
#define CMD_AVERAGE_POWER_H 0x25

#define CMD_CYCLE_COUNT_L 0x2A
#define CMD_CYCLE_COUNT_H 0x2B

#define CMD_STATE_OF_CHARGE_L 0x2C
#define CMD_STATE_OF_CHARGE_H 0x2D

#define EXTD_CMD_PACK_CONFIG_L 0x3A
#define EXTD_CMD_PACK_CONFIG_H 0x3B

#define EXTD_CMD_DESIGN_CAPACITY_L 0x3C
#define EXTD_CMD_DESIGN_CAPACITY_H 0x3D

#define EXTD_CMD_DATA_FLASH_BLOCK 0x3F

#define EXTD_CMD_MANUFACTURE_BLOCK_A 0x01
#define EXTD_CMD_MANUFACTURE_BLOCK_B 0x02
#define EXTD_CMD_MANUFACTURE_BLOCK_C 0x03

#define EXTD_CMD_BLOCK_DATA_L 0x40
#define EXTD_CMD_BLOCK_DATA_H 0x5f

#define EXTD_CMD_BLOCK_DATA_CHEKSUM 0x60

class HDQBATT {
  private:
	HDQDelay hdq_delay;
    uint8_t _pin;
    uint8_t _block_data[EXTD_CMD_BLOCK_DATA_H - EXTD_CMD_BLOCK_DATA_L];
    
    void doBreak(void); // Сигнал сброса на линии
    void sendByteBitByBit(uint8_t payload); // Передача байта побитово

    void pullBlockData(uint8_t str[]);
    
  public:
    HDQBATT(uint8_t pin);
    // Для изменения значений задержек протокола HDQ
    // 7.13 HDQ Communication Timing Characteristics
    bool setTimeCYCH(uint8_t new_delay);
    bool setTimeCYCD(uint8_t new_delay);
    bool setTimeHW1(uint8_t new_delay);
    bool setTimeDW1(uint8_t new_delay);
    bool setTimeHW0(uint8_t new_delay);
    bool setTimeDW0(uint8_t new_delay);
    bool setTimeRSPS(uint16_t new_delay);
    bool setTimeB(uint16_t new_delay);
    bool setTimeBR(uint16_t new_delay);
    bool setTimeFailTries(uint16_t new_delay); // Не из даташита
    // Для работы с произвольными регистрами (смотреть datasheet)
    uint8_t readByte(uint8_t reg);
    uint16_t readWord(uint8_t low_reg, uint8_t high_reg);
    uint16_t readControlAddresses(void);
    void writeByte(uint8_t reg, uint8_t payload);
    void writeWord(uint8_t low_reg, uint8_t high_reg, uint8_t low_payload, uint8_t high_payload);
    void writeControlAddresses(uint8_t low_payload, uint8_t high_payload);
    // Регистры 00 01 (00 01) getDeviceType
    bool isConnected(void);
    // Регистры Status 00 01
    uint16_t getControlStatus(void); // HEX
    // Биты регистров Status
    bool getFlagSEPinIsActive(void);
    bool getFlagIsFullAccessSealedMode(void);
    bool getFlagIsSealedMode(void);
    bool getFlagCalibrationFunctionIsActive(void);
    bool getFlagCoulombCounterCalibrationRoutineIsActive(void);
    bool getFlagBoardCalibrationRoutineIsActive(void);
    bool getFlagQMAXUpdate(void);
    bool getFlagHDQInterruptFunctionIsActive(void);
    bool getFlagShutdownCommandIsSent(void);
    bool getFlagRequestHibernateFromSleepMode(void);
    bool getFlagIsFullSleepMode(void);
    bool getFlagIsSleepMode(void);
    bool getFlagImpedanceTrackAlgorithm(void);
    bool getFlagRaTableUpdatesDisabled(void);
    bool getFlagCellVoltagesOK(void);
    bool getFlagQmaxUpdatesEnabled(void);
    // Регистры 00 01 (00 01, 00 02, 00 03)
    uint16_t getDeviceType(void); // HEX
    float getFirmwareVersion(void);
    float getHardwareVersion(void);
    // Регистры Temperature 06 07
    float getTemperatureKelvin(void);
    float getTemperatureCelsius(void);
    float getTemperatureFahrenheit(void);
    // Регистры Voltage 08 09
    unsigned short getVoltageMilli(void);
    float getVoltage(void);
    // Регистры Flags 0A 0B
    uint16_t getFlags(void); // HEX
    // Биты регистров Flags
    bool getFlagBatteryHighIndicating(void);
    bool getFlagBatteryLowIndicating(void);
    bool getFlagChargeInhibitindicates(void);
    bool getFlagFullChargedIsDetected(void);
    bool getFlagChargeSuspend(void);
    bool getFlagIndicatesComputedImax(void);
    bool getFlagChargingAllowed(void);
    bool getFlagStateOfChargeThreshold1(void);
    bool getFlagStateOfChargeThresholdFinal(void);
    bool getFlagDischargingDetected(void);
    // Регистры Remaining Capacity 10 11
    unsigned short getRemainingCapacity(void);
    // Регистры FullCharge Capacity 12 13
    unsigned short getFullChargeCapacity(void);
    // Регистры Average Current 14 15
    short getAverageCurrentMilli(void);
    float getAverageCurrent(void);
    // Регистры Time to Empty 16 17
    unsigned short getTimeToEmpty(void);
    // Регистры Average Power 24 25
    short getAveragePowerMilli(void);
    float getAveragePower(void);
    // Регистры Cycle Count 2A 2B
    unsigned short getCycleCount(void);
    // Регистры State of Charge 2C 2D
    byte getStateOfCharge(void);
    // Регистры Design Capacity 3C 3D
    unsigned short getDesignCapacity(void);
    // Зипись в регистр Data Flash Block 3F (01, 02, 03); чтение из регистров Block Data 40 - 5F
    char* getManufacturerInfoBlockA(void);
    char* getManufacturerInfoBlockB(void);
    char* getManufacturerInfoBlockC(void); // Не описан в документации, но данные там есть
    // Регистр Block Data Checksum 60
    uint8_t getBlockDataChecksum(void);
};

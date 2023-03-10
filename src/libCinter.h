// Do not remove the include below
#ifndef LIBCINTER_H_
#define LIBCINTER_H_

/*
#include "loraE5_station.h"

#include <Wire.h>
#include "lora/lora.h"
#include "lora/logUtils.h"
#include "cinterUtils.h"
#include "malloc.h"

#include <bmx280Reader.h>
#include <ADCfilteredReader.h>
#include <ArduinoModbus.h>
#include <DS3232RTC.h>
#include <ArduinoJson.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>


*/

#include "LinkedList.h"
#include "tasksketuler.h"
#include "FileLogger.h"

enum tasksEnum{
    TASK_PORT_A,
    TASK_PORT_B,
    TASK_PORT_C,
    TASK_SENSOR_READ,
    TASK_TRANSMIT,
};

typedef  struct
{
    union __attribute__ ((__packed__)) {
        uint8_t data[51];
         struct __attribute__ ((__packed__)){
            uint8_t stationID[2];
            uint8_t cmdNum;
            uint32_t time;
            union __attribute__ ((__packed__)) {
                uint8_t payload[44];
                int16_t payload_int16_t[22];
                int32_t payload_int32_t[11];
            };
        };
    };
    int len;
    int payloadLen;
}dataPack;

typedef union
{
    struct {
        uint8_t id;
        uint8_t type;
        uint8_t address;
        uint8_t nb;
    };
    struct{
        uint8_t data[4];
    };
}ModbusCommand;

typedef struct{
    uint8_t cmd;
    int (*function) (dataPack &cmd);
} commandFuction;

typedef struct __attribute__ ((__packed__)){
    int16_t transmit_period;
    int16_t transmit_moment;
    int16_t measurement_period;
    int16_t port_A_onTime;
    int16_t port_B_onTime;
    int16_t port_C_onTime;
} timings_t;

enum cmdNumbers{CMD_testCMD=3,
   CMD_setReadoutCMD=4,
   CMD_printText=5,
   CMD_commandACK=6,
   CMD_setTimings=7,
   CMD_setDatetime=8,
};

bool mountSD();
void unmountSD();
uint32_t getutctimestamp();
int printText(uint8_t * args,int argc);
int setReadoutCMD(uint8_t * args,int argc);
int sensorReadout(ModbusCommand * commands,int numCommands);
void runCommand(uint8_t * args,int argc,bool saveCMDinFile=false);
void updatePortPower();
void sensorReadTask();
void loraSendDataBuffer();
void actualizeDisplay();
int setTimings(uint8_t * args,int argc);
void loadCMDfromConfigFile();
void saveCMDinConfigfile(uint8_t * args,int argc);
void runCommand(uint8_t * args,int argc,bool saveCMDinFile);
void loraSendDataBuffer();
void updatePortPower();
void sensorReadTask();
void loraSetRecive();
void loraSetSleep();
void actualizeDisplay();
void libCinter_init();

#endif

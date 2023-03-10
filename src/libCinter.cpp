// Do not remove the include below

#include "libCinter.h"

#include <Wire.h>
#include "lora.h"
//#include "lora/logUtils.h"
#include "cinterUtils.h"
#include "malloc.h"
#include "LinkedList.h"
#include <bmx280Reader.h>
#include <ADCfilteredReader.h>
#include <ArduinoModbus.h>
#include <DS3232RTC.h>
#include <ArduinoJson.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "tasksketuler.h"
#include "base64.hpp"
#include "FileLogger.h"
#include "cinter_config.h"


extern SdFat sd;
extern RS485Class rs485;
extern Adafruit_SSD1306 display;
extern String displayText="";
extern char stationID[];

uint8_t rtcRead(tmElements_t & tm);
uint8_t rtcWrite(tmElements_t & tm);

FileLogger dataLogger(STATION_ID"/dataLog");
LinkedList<dataPack> sendData = LinkedList<dataPack>();
StaticJsonDocument<1024> config;

#define internalRegisters_LEN 7
unsigned short internalRegisters[internalRegisters_LEN]={0x7FFF,0x7FFF,0x7FFF,0x7FFF,0x7FFF,0x7FFF,0x7FFF};

#define readoutCommands_LEN 16
ModbusCommand readoutCommands[readoutCommands_LEN]={};
#define commandList_LEN (int) (sizeof(commandList) / sizeof(commandFuction))
 
bool loraSending=false;



uint32_t getutctimestamp(){
    tmElements_t now;
    rtcRead(now);
    return makeTime(now);
}

int printText(dataPack & cmd){
    Serial.print("printText(");
    for(int i =0;i< cmd.payloadLen;i++){
        Serial.print((char)cmd.payload[i]);
    }
    cmd.payload[cmd.payloadLen]=0;
    displayText=(char *)cmd.payload;
    Serial.println(")");
    return 0;
}

int setReadoutCMD(dataPack & cmd){
    Serial.println("setReadoutCMD");
    for(int i =0;i< cmd.payloadLen;i++){
        readoutCommands[i/4].data[i%4]=cmd.payload[i];
    }
    for(int i =cmd.payloadLen;i< (int)sizeof(readoutCommands);i++){
        readoutCommands[i/4].data[i%4]=0;
    }
    return 0;
}

int sensorReadout(ModbusCommand * commands,int numCommands){
    mountSD();

    unsigned short mbdata[16];
    int mbdataIndex=0;
    dataPack outD={.len=0};
    outD.stationID[0]=stationID[0];
    outD.stationID[1]=stationID[1];
    int outDoffset=0;
    int outDLen=0;
    outD.time=getutctimestamp();
    for (int i=0;i<numCommands;i++){
        ModbusCommand cmd=commands[i];
        if (cmd.nb>0){
            if(cmd.id>0){
                if (!ModbusRTUClient.requestFrom(cmd.id, cmd.type, cmd.address, cmd.nb)) {
                    Serial.print("failed to read registers! ");
                    Serial.println(ModbusRTUClient.lastError());
                    for (int i = 0; i < cmd.nb; i++) {
                        mbdata[mbdataIndex] = 0x7FFF;
                        mbdataIndex++;
                    }
                } else {
                    for (int i = 0; i < cmd.nb; i++) {
                        mbdata[mbdataIndex] = (unsigned short) ModbusRTUClient.read();
                        mbdataIndex++;
                    }
                }
            }
            else if(cmd.id==0){
                for (int i = 0; i < cmd.nb; i++) {
                    int internalRegisterAddress=i+cmd.address;
                    if(internalRegisterAddress<internalRegisters_LEN){
                        mbdata[mbdataIndex] = internalRegisters[internalRegisterAddress];
                    }
                    else{
                        mbdata[mbdataIndex] = 0x7FFF;
                    }
                    mbdataIndex++;
                }
            }

            if(outDLen+mbdataIndex>22){
                outD.cmdNum=0x80|outDoffset;
                outD.len=outDLen*2+7;
                sendData.add(outD);
                outDoffset+=outDLen;
                outDLen=0;
            }
            //Data
            for (int i = 0; i < mbdataIndex; i++) {
                dataLogger.print(mbdata[i]);
                dataLogger.print(",");
                outD.payload_int16_t[outDLen+i]=mbdata[i];
            }
            outDLen+=mbdataIndex;
            mbdataIndex=0;
        }
    }
    if(outDLen>0){
        outD.cmdNum=0x80|outDoffset;
        outD.len=outDLen*2+7;
        sendData.add(outD);
    }
    dataLogger.println();
    return 0;
}


task_t tasks[]={
    {20,10,-10,0,updatePortPower,NULL},
    {20,5,-5,0,updatePortPower,NULL},
    {20,2,-2,0,updatePortPower,NULL},
    {20,1,0,0,sensorReadTask,NULL},
    {20,5,-10,0,loraSendDataBuffer,NULL},
    //{5,0,0,0,loraSend,NULL},
};
const int taskList_LEN = (sizeof(tasks) / sizeof(task_t));

int setTimings(dataPack & cmd){
    if(cmd.payloadLen==sizeof(timings_t)){

        timings_t * t= (timings_t *) cmd.payload;

        Serial.print("measurement_period: ");
        Serial.println(t->measurement_period);
        Serial.print("port_A_onTime: ");
        Serial.println(t->port_A_onTime);
        Serial.print("port_B_onTime: ");
        Serial.println(t->port_B_onTime);
        Serial.print("port_C_onTime: ");
        Serial.println(t->port_C_onTime);
        Serial.print("transmit_period: ");
        Serial.println(t->transmit_period);
        Serial.print("transmit_moment: ");
        Serial.println(t->transmit_moment);

        tasks[TASK_PORT_A].period=t->measurement_period;
        tasks[TASK_PORT_B].period=t->measurement_period;
        tasks[TASK_PORT_C].period=t->measurement_period;
        tasks[TASK_SENSOR_READ].period=t->measurement_period;
        Serial.print("measurement_period: ");
        Serial.println(t->measurement_period);


        tasks[TASK_PORT_A].onTime=t->port_A_onTime;
        tasks[TASK_PORT_B].onTime=t->port_B_onTime;
        tasks[TASK_PORT_C].onTime=t->port_C_onTime;

        tasks[TASK_PORT_A].offset=-t->port_A_onTime;
        tasks[TASK_PORT_B].offset=-t->port_B_onTime;
        tasks[TASK_PORT_C].offset=-t->port_C_onTime;

        tasks[TASK_TRANSMIT].period=t->transmit_period;
        tasks[TASK_TRANSMIT].offset=t->transmit_moment;

        return 0;
    }
    else{
        return -1;
    }
}

int setDateTime(dataPack & cmd){
    if(cmd.payloadLen==0){
        tmElements_t  tm;
        breakTime(cmd.time, tm);
        rtcWrite(tm);
        return 0;
    }
    return -1;
}

commandFuction commandList[]={
    {4,&setReadoutCMD},
    {5,&printText},
    //{6,&commandACK},
    {7,&setTimings},
    {8,&setDateTime},
};

void loadCMDfromConfigFile(){
    if(!mountSD()){
        Serial.println("loadCMDfromConfigFile: error mounting");
        return;
    }
    Serial.println("loadCMDfromConfigFile");
    File file = sd.open(CONFIG_FILE);
    if(file<=0){
        Serial.println( "loadCMDfromConfigFile: open config.json failed ");
        return;
    }
    DeserializationError error = deserializeJson(config, file);
    if (error) {
      Serial.print( "deserializeJson() failed: ");
      Serial.println( error.c_str());
      return;
    }
    for( const auto& kv : config["startupCMDs"].as<JsonObject>() ) {
        const char * cmd=kv.value().as<const char*>();
        uint8_t args[52];
        int argc;
        argc=decode_base64((unsigned char*)cmd+2,strlen(cmd)-2,(unsigned char*)args+2)+2;//do not decode stationID
        args[0]=cmd[0];
        args[1]=cmd[1];
        runCommand(args,argc);
    }
}

void saveCMDinConfigfile(uint8_t * args,int argc){
    if(!mountSD()){
        Serial.println("saveCMDinConfigfile: error mounting");
        return;
    }
    Serial.println("saveCMDinConfigfile");
    File file = sd.open(CONFIG_FILE, O_WRITE | O_CREAT);
    if(file<=0){
        Serial.println( "saveCMDinConfigfile: open config.json failed ");
        return;
    }

    JsonVariant variant=config["startupCMDs"];
    JsonObject startupCMDs;
    if(variant.is<JsonObject>()){
        startupCMDs=config["startupCMDs"].as<JsonObject>();
    }
    else{
        startupCMDs=config["startupCMDs"].to<JsonObject>();
    }

    unsigned char encodedCommand[100];
    encodedCommand[0]=args[0];
    encodedCommand[1]=args[1];
    int encodedCommandLen=encode_base64((unsigned char*)args+2,argc-2,encodedCommand+2);
    encodedCommand[encodedCommandLen+2]=0;
    char cmdNumStr[5];
    itoa((int)args[2],cmdNumStr,10);
    startupCMDs[cmdNumStr] = encodedCommand;
    serializeJson(config, file);
    file.close();
}

void runCommand(uint8_t * args,int argc,bool saveCMDinFile){
    //cout << args[0]<< args[1] << " cmd:" << (int)args[2] << " " << argc << endl;
    dataPack * cmd=(dataPack*) args;
    cmd->payloadLen=argc-7;
    cmd->len=argc;
    if(cmd->stationID[0]==stationID[0] && cmd->stationID[1]==stationID[1]){
        for(int j=0;j<commandList_LEN;j++){
            if(commandList[j].cmd==cmd->cmdNum){
                int32_t retVal=commandList[j].function(*cmd);

                if(saveCMDinFile&&cmd->cmdNum!=CMD_setDatetime){
                    saveCMDinConfigfile(args,argc);
                }
                uint32_t time=getutctimestamp();
                dataPack outD;

                //Station ID
                outD.stationID[0]=stationID[0];
                outD.stationID[1]=stationID[1];
                outD.cmdNum=CMD_commandACK;
                outD.time=time;
                outD.payload_int32_t[0]=cmd->time; // commandID
                Serial.print("commandID: ");
                Serial.println(cmd->time);
                outD.payload_int32_t[1]=retVal;
                outD.len=15;
                if(args[2]!=CMD_commandACK){
                    Serial.println("Push ACK");
                    sendData.add(outD);
                }
            }
        }
    }
    else{
        //RadioSend(args,argc);
    }
}

void loraSendDataBuffer(){
    while(sendData.size()>0){
        loraSending=true;
        dataPack inD=sendData.shift();
        loraSend(inD.data,inD.len,1);
        actualizeDisplay();
    }
    loraSending=false;
}

extern int loraRxCount;
void updatePortPower();

void sensorReadTask(){
    sensorReadout(readoutCommands,readoutCommands_LEN);
    updatePortPower();
    unmountSD();
}
uint8_t loraInRX=1;

uint8_t powerState=0;
void updatePortPower(){
    powerState=0;
    if(tasks[0].active){
        powerState|=1<<3;
    }
    else{
        volatile int k=0;
    }
    if(tasks[1].active){
        powerState|=1<<7;
    }
    else{
        volatile int k=0;
    }
    if(tasks[2].active){
        powerState|=1<<5;
    }
    else{
        volatile int k=0;
    }
    setSensorPower(powerState);
}

void actualizeDisplay(){
    char timeStr[32];
    tmElements_t now;
    rtcRead(now);
    int nowint=makeTime(now);
    formatDateTime(timeStr, 32, now);

    display.fillRect(0, 0, 128, 64, 0);
    display.setCursor(0, 0);
    display.println(timeStr);
    display.print("rxCnt: ");
    display.println(loraRxCount);
    display.println(displayText);

    struct mallinfo mi = mallinfo();
    display.print("Mem ");
    display.println(mi.uordblks);

    for(int i=0;i<8;i++){
        char pwrText[]="   ";
        if(powerState&1<<3){
            pwrText[0]='A';
        }
        if(powerState&1<<7){
            pwrText[1]='B';
        }
        if(powerState&1<<5){
            pwrText[2]='C';
        }
        display.setCursor(10, 52);
        display.print(pwrText);

    }
    if (loraSending){
        display.setCursor(120, 52);
        display.print("T");
    }
    else if (loraInRX){
        display.setCursor(120, 52);
        display.print("R");
    }
    display.display();
}

void libCinter_init(){
    dataLogger.begin(&rtcRead);
}

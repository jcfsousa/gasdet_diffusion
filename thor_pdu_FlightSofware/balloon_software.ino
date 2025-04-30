/* UNO:
    CLK - 13
    MOSI - 11
    MISO - 12

    =======================================================================
    ===================== SPI Test for THOR_PDU_DM.1 =====================
    =======================================================================

    => SPI MAX SPEED: 2 MHz

    => READ: bit to 1
    => WRITE: bit to 0

    => SPI MODE: 0 (CPOL=0 && CPHA=0) OR 3 (CPOL=1 && CPHA=1)
       - ADC MCP3208 supports mode 0 and 3;
       - GPIO_CNTRL MCP23S17 supports mode 0 and 3;


    SDI -> R/W | A6 | A5 | A4 | A3 | A2 | A1 | A0 || D7 | D6 | D5 | D4 | D3 | D2 | D1 | D0
    SDO ->  0  | 0  | 1  | 0  | 0  | 0  | 0  | 0  || R7 | R6 | R5 | R4 | R3 | R2 | R1 | R0 -> or 8 zeros, if WRITE operation


    ========== COM with GPIO_CNTRL ===========
      MOSI -» [Control Byte + R/W] + [Register Address] + [Register Address]
           -» 0 | 1 | 0 | 0 | 0 | 0 | 0 | R/W || A7 | A6 | A5 | A4 | A3 | A2 | A1 | A0 || D7 | D6 | D5 | D4 | D3 | D2 | D1 | D0 -» Dx when W
      
      MISO -» 0 | 1 | 0 | 0 | 0 | 0 | 0 | R/W || NA | NA | NA | NA | NA | NA | NA | NA || R7 | R6 | R5 | R4 | R3 | R2 | R1 | R0 

*/

#include <SPI.h>
//#include "src/PDU_REG_ADD.h"
#include <Ethernet.h>
#include <TaskScheduler.h>
#include <SD.h>
#include <vector>



#ifdef ESP32-WROOM-DA


#define LED 2 //
#define CS_BAR 13 //
#define CS_SD 14 //
#define RESET_ETH 16 //
#define CS_GPIO 17 //dummy
#define CS_ETH 22 //
#define INTA_DUMMY 25 //
#define INTB_DUMMY 26 //
#define CS_ADC1 27 //
#define CS_ADC2 32 //
#define CS_GPIO_PDU 33 //
#define INTA_PDU 34 //
#define INTB_PDU 35 //

#define MOSI 23
#define MISO 19
#define CLK 18

//default ESP32 VSPI
// SPI_MOSI 23
// SPI_MISO 19
// SPI_CLK 18

#endif

// SPI Operation
#define _WRITE 0x40
#define _READ  0x41


#define DEBUG 2

#if DEBUG == 0
#define debugPrint(...)
#define debugPrintln(...)
#define pyPrint(...) Serial.print(__VA_ARGS__)
#define pyPrintln(...) Serial.println(__VA_ARGS__)
#define ballonPrint(...)
#define ballonPrintln(...)
#elif DEBUG == 1
#define debugPrint(...) Serial.print(__VA_ARGS__)
#define debugPrintln(...) Serial.println(__VA_ARGS__)
#define pyPrint(...)
#define pyPrintln(...)
#define ballonPrint(...)
#define ballonPrintln(...)
#elif DEBUG == 2
#define debugPrint(...)
#define debugPrintln(...)
#define pyPrint(...) 
#define pyPrintln(...) 
#define ballonPrint(...) Serial.print(__VA_ARGS__)
#define ballonPrintln(...) Serial.println(__VA_ARGS__)
#else 
#define debugPrint(...)
#define debugPrintln(...)
#define pyPrint(...) 
#define pyPrintln(...) 
#define ballonPrint(...)
#define ballonPrintln(...) 
#endif


#define SPI_STANDARD_SETTINGS SPISettings(100000,MSBFIRST,SPI_MODE0)



// IC code: GPIO_CNTRL = 0 ; ADC1 = 1 ; ADC2 = 2; ADC3 = 3; DAC = 4
String GPIO_PDU = "GPIO_PDU";
String ADC1 = "ADC1";
String ADC2 = "ADC2";
String GPIO_DUMMY = "GPIO_DUMMY";
String ETH = "ETH";
String SD_CARD = "SD_CARD";
String BAR = "BAR";

int INTA_PDU_Status = 0;
int INTB_PDU_Status = 0;
int INTA_DUMMY_Status = 0;
int INTB_DUMMY_Status = 0;



// ESP32 CONSTANTS
const float VHK_OBC_A_calib_const = 1.01292;  // y=ax+b
const float VHK_OBC_B_calib_const = -0.00044;   // 
const float VHK_OBC_offset = 0;

const float VHK_OBC_out_A_calib_const = 1.02334;
const float VHK_OBC_out_B_calib_const = 0.00036;  //
const float VHK_OBC_out_offset = 0;

const float VHK_GAM_A_calib_const = 1.00747;
const float VHK_GAM_B_calib_const = -0.00046; // 
const float VHK_GAM_offset = 0;

const float VHK_GAM_out_A_calib_const = 1.00414;
const float VHK_GAM_out_B_calib_const = -0.00081;
const float VHK_GAM_out_offset = 0;


const float CHK_OBC_A_calib_const = 1.004675;
const float CHK_OBC_B_calib_const = 0.673118; // added 0.5 after calib
const float CHK_OBC_noLoad_offset = 4;  //LSB 4

const float CHK_GAM_A_calib_const = 1.008430;  // y=ax+b
const float CHK_GAM_B_calib_const = 1.115179;
const float CHK_GAM_noLoad_offset = 22; //LSB 12

const float CHK_SR_A_calib_const = 1;
const float CHK_SR_B_calib_const = 0;
const float CHK_SR_noLoad_offset = 0;   //LSB 0

// const float CHKconstantFactor = 0.000305 * 1000; // pre-compute the constant to reduce time calculating CHK

// DAC calibration constants
const float DAC_a = 1.012452;  
const float DAC_b = 4.484228; //added 1.69 after calib

float mean_CHK_SR = 0;




unsigned long startTime;
unsigned long endTime;
float elapsedTime;

const int op_mode_waitTime = 600000;  // change this to 10min for flight!!!!!!!!!!

bool LCL_bool = false;
bool test_1 = false;
bool test_2 = false;

bool esp32_initialsetup_check = false;
bool DUMMY_initialsetup_check = false;
bool PDU_initialsetup_check = false;
bool SD_initialsetup_check = false;
bool ETH_initialsetup_check = false;

bool low_pressure = false; // it triggers when the pressure is <40kpa
bool low_pressure_firstTime = true; // just to run once the 'tenKmTestTrigger()' function
bool start_opMode_tests = false; // triggers to check that we are ready for the operations to start, just need to wait to reach 10km.
bool endOfTests = false; // to trigger when I am on the last Operation mode to test - 'Full Load'
bool endOfTests_changeMode = false; // At the end of operations, just chage mode 1 time
bool needs_header = false; //If i need to send HK header to server

bool log_lowpressure = false;
bool check_setDummyLoad = false;

int tenK_wait_time;
int timeNow_pressure7km;
int time_activate_operations;


String selectedIC = "GPIO_PDU"; //starts with the GPIO_CNTRL
int selectedIC_CS = CS_GPIO_PDU;


// Ethernet settings
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
IPAddress ip(192, 168, 0, 101);
IPAddress server(192,168,0,100);
//IPAddress server(192,168,0,100);
IPAddress myDns(192, 168, 0, 1);
EthernetClient client;

// SD settings
File myFile;


std::vector<String> listFiles= {
        "/SDboot.txt",
        "/HKmeasurement.txt",
        "/LCLDataFile.txt",
        "/LCLTrigger.txt",
        "/log.txt"
};


std::vector<String> listOPmodes_toTest = {
        "gam",
        "obc redundancy",
        "full load"
};

std::vector<String> listCHK = {
        "CHK_GAM",
        "CHK_OBC"  
};

std::vector<String> listVHK = {
        "VHK_GAM",
        "VHK_GAM_out",
        "VHK_OBC",
        "VHK_OBC_out"
};

std::vector<String> listBAR = {
        "Temperature",
        "Pressure",
};

std::vector<String> listALTITUDE = {
        "Altitude",
        "Velocity"
};

float velocity_oldTime;
float altitude_oldValue;


std::vector<String> listSensors;
void readSensorData();
void sendDataToServer();


// TaskScheduler object
Scheduler runner;

// Task definitions
Task taskReadSensor(1000, TASK_FOREVER, &readSensorData);
//Task taskSendData(5000, TASK_FOREVER, &sendDataToServer);

// Mutex dor SPI, shedule multicore acess to SPI interface
SemaphoreHandle_t spiMutex;


//###################              FUNCTIONS               #####################
void set_selected_IC(String selected_ic) {
    char buffer[100];  // Adjust buffer size as needed
    
    if(selected_ic.equals("GPIO_PDU")) {
        selectedIC = GPIO_PDU;
        selectedIC_CS = CS_GPIO_PDU;
        debugPrintln("Changed communication to GPIO PDU");
    }
    else if(selected_ic.equals("ADC1")) {
        selectedIC = ADC1;
        selectedIC_CS = CS_ADC1;
        debugPrintln("Changed communication to ADC 1");
    }
    else if(selected_ic.equals("ADC2")) {
        selectedIC = ADC2;
        selectedIC_CS = CS_ADC2;
        debugPrintln("Changed communication to ADC 2");
    }
    else if(selected_ic.equals("GPIO_DUMMY")){
        selectedIC = GPIO_DUMMY;
        selectedIC_CS = CS_GPIO;
        debugPrintln("Changed communication to GPIO DUMMY");
    }
    else if(selected_ic.equals("ETH")){
        selectedIC = ETH;
        selectedIC_CS = CS_ETH;
        debugPrintln("Changed communication to ETH");
    }
    else if(selected_ic.equals("CS")){
        selectedIC = SD_CARD;
        selectedIC_CS = CS_SD;
        debugPrintln("Changed communication to CS");
    }
    else if(selected_ic.equals("BAR")){
        selectedIC = BAR;
        selectedIC_CS = CS_BAR;
        debugPrintln("Changed communication to BAR");
    }
    else {
        debugPrint("Sorry, IC \"");
        debugPrint(selected_ic);
        debugPrintln("\" is invalid.");
        return;
    }
    
    // Format and print the CS pin and IC info
    snprintf(buffer, sizeof(buffer), "CS pin selected: %d IC: %s", selectedIC_CS, selectedIC);
    debugPrintln(buffer);
}




//###################          [BOOT] initial setup          ###################
void PDU_initialsetup(){
  SPI.beginTransaction(SPI_STANDARD_SETTINGS);
  set_selected_IC("GPIO_PDU");

  SPI_write(selectedIC_CS, 0x05, 0xfe); 
  SPI_write(selectedIC_CS, 0x15, 0xfe);   
    
  SPI_write(selectedIC_CS, 0x00, 0xc9);
        
  SPI_write(selectedIC_CS, 0x01, 0x00);
  SPI_write(selectedIC_CS, 0x11, 0x00);
    
  SPI_write(selectedIC_CS, 0x02, 0x00);
  SPI_write(selectedIC_CS, 0x12, 0x00);
    
  SPI_write(selectedIC_CS, 0x03, 0x00);
  SPI_write(selectedIC_CS, 0x13, 0x00);
    
  SPI_write(selectedIC_CS, 0x04, 0x00);
  SPI_write(selectedIC_CS, 0x14, 0x00);
  
  SPI_write(selectedIC_CS, 0x06, 0x00);
  SPI_write(selectedIC_CS, 0x16, 0x00);
  
  SPI_write(selectedIC_CS, 0x0a, 0xff);
  SPI_write(selectedIC_CS, 0x0a, 0x30);
  SPI_write(selectedIC_CS, 0x09, 0x30);

  SPI_write(selectedIC_CS, 0x05, 0xba);
  SPI_write(selectedIC_CS, 0x15, 0xba);
    
  SPI_write(selectedIC_CS, 0x10, 0xff);

  SPI.endTransaction();

  ballonPrintln("   PDU initial setup: done...");

  PDU_initialsetup_check = true;
  
}

void DUMMY_initialsetup(){
  SPI.beginTransaction(SPI_STANDARD_SETTINGS);
  set_selected_IC("GPIO_DUMMY");

  SPI_write(selectedIC_CS, 0x05, 0xfe); 
  SPI_write(selectedIC_CS, 0x15, 0xfe);   

  SPI_write(selectedIC_CS, 0x00, 0x00);

  SPI_write(selectedIC_CS, 0x01, 0x00);
  SPI_write(selectedIC_CS, 0x11, 0x00);
    
  SPI_write(selectedIC_CS, 0x02, 0x00);
  SPI_write(selectedIC_CS, 0x12, 0x00);
    
  SPI_write(selectedIC_CS, 0x03, 0x00);
  SPI_write(selectedIC_CS, 0x13, 0x00);
    
  SPI_write(selectedIC_CS, 0x04, 0x00);
  SPI_write(selectedIC_CS, 0x14, 0x00);
  
  SPI_write(selectedIC_CS, 0x06, 0x00);
  SPI_write(selectedIC_CS, 0x16, 0x00);
  
  SPI_write(selectedIC_CS, 0x0a, 0xff);
  SPI_write(selectedIC_CS, 0x0a, 0xff);
  SPI_write(selectedIC_CS, 0x09, 0xff);

  SPI_write(selectedIC_CS, 0x0a, 0x00);
  SPI_write(selectedIC_CS, 0x0a, 0x00);
  SPI_write(selectedIC_CS, 0x1a, 0x00);

  SPI_write(selectedIC_CS, 0x05, 0xba);
  SPI_write(selectedIC_CS, 0x15, 0xba);
    
  SPI_write(selectedIC_CS, 0x10, 0x80);

  // need to write DAC CS to high again!!!!!

  SPI_write(selectedIC_CS, 0x09, 0xff); // set CS High

  SPI.endTransaction();

  DUMMY_initialsetup_check = true;

  ballonPrintln("   DUMMY initial setup: done...");

}

void esp32_initialsetup(){
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);

  pinMode(CS_BAR, OUTPUT);
  digitalWrite(CS_BAR, HIGH);

  pinMode(CS_SD, OUTPUT);
  digitalWrite(CS_SD, HIGH);

  //pinMode(RESET_ETH, OUTPUT);
  //digitalWrite(RESET_ETH, HIGH);

  pinMode(CS_GPIO, OUTPUT);
  digitalWrite(CS_GPIO, HIGH);

  pinMode(CS_ETH, OUTPUT);
  digitalWrite(CS_ETH, HIGH);

  pinMode(CS_ADC1, OUTPUT);
  digitalWrite(CS_ADC1, HIGH);

  pinMode(CS_ADC2, OUTPUT);
  digitalWrite(CS_ADC2, HIGH);

  pinMode(CS_GPIO_PDU, OUTPUT);
  digitalWrite(CS_GPIO_PDU, HIGH);


  pinMode(INTA_DUMMY, INPUT);
  pinMode(INTB_DUMMY, INPUT);
  pinMode(INTB_PDU, INPUT);
  pinMode(INTA_PDU, INPUT);

  esp32_initialsetup_check = true;

  ballonPrintln("   ESP32 initial setup: done...");
}

void SD_initialsetup() {

  if (!SD.begin(CS_SD, SPI, 100000)) {
      ballonPrintln("   SD card initialization failed!!!");
      return;
  }

  for (int i = 0; i < listFiles.size(); i++) {
      if (SD.exists(listFiles[i])) {
          SD.remove(listFiles[i]);
      }

      File myFile = SD.open(listFiles[i], FILE_WRITE);

      if (!myFile) {
          ballonPrintln("Error opening file: " + listFiles[i]);
          continue; 
      }

      if (listFiles[i].equals("/SDboot.txt")) {
          myFile.println("SD boot successful!");
          myFile.print("Startup millis: ");
          myFile.println(millis());
      }
      myFile.close();
  }

  SD_initialsetup_check = true;

  ballonPrintln("   SD card: done...");
  
}


void ETH_initialsetup() {

  Ethernet.init(CS_ETH);

  if (Ethernet.begin(mac) == 0) {
    ballonPrintln("   Failed to configure Ethernet using DHCP.");

    if (Ethernet.hardwareStatus() == EthernetNoHardware) {
      ballonPrintln("   Ethernet shield was not found. Cannot run without hardware.");
      return;
    }

    if (Ethernet.linkStatus() == LinkOFF) {
      ballonPrintln("   Ethernet cable is not connected.");
      return;
    }

    else {
      Ethernet.begin(mac, ip, myDns);
      ballonPrint("   Static IP assigned: ");
      ballonPrintln(Ethernet.localIP());
    }
  } 
  
  else {
    ballonPrint("   DHCP assigned IP: ");
    ballonPrintln(Ethernet.localIP());
  }

  delay(1000);
  ballonPrint("   Connecting to server: ");
  ballonPrintln(server);

  ETH_initialsetup_check = true;

  ballonPrintln("   ETH: done...");

}
//##############################################################################




//###################         [SPI] general functions        ###################
void SPI_write(int CS_pin, byte register_address, byte _data) {

  digitalWrite(CS_pin, LOW);

  byte byte1 = SPI.transfer(_WRITE);
  byte byte2 = SPI.transfer(register_address);
  byte byte3 = SPI.transfer(_data);

  digitalWrite(CS_pin, HIGH);

}

byte SPI_read(int CS_pin, byte register_address) {

  digitalWrite(CS_pin, LOW);

  byte byte1 = SPI.transfer(_READ);
  byte byte2 = SPI.transfer(register_address);
  byte byte3 = SPI.transfer(0x00);
  
  digitalWrite(CS_pin, HIGH);

  return byte3;
}
//##############################################################################




//####################      [SPI] temperature sensors      #####################
float SPI_read_TMP(int CS_pin, byte byte_1, byte byte_2, int n) {
  digitalWrite(CS_pin, LOW);
  //delay(0.1);

  byte byte1 = SPI.transfer(byte_1);
  //delay(0.1);
  byte byte2 = SPI.transfer(byte_2);
  //delay(0.1);
  byte byte3 = SPI.transfer(0x00);
  //delay(0.1);
  digitalWrite(CS_pin, HIGH);

  float TMP_Value_Calibrated = 0;

  byte byte2_trimmed = byte2 & 0x0f;

  uint16_t combinedValue = (uint16_t(byte2_trimmed) << 8) | byte3;

  int intValue = (int)combinedValue;

  float TMP_Value = 136 - 0.147*intValue + 8.06*pow(10,-5) * pow(intValue,2) - 2.06 * pow(10,-8)*pow(intValue,3) + 1.75* pow(10,-12)*pow(intValue,4);

  //calculations with INT is way faster than float, maybe improve in the future

  if (n == 1){
    float TMP_Value_Calibrated = 1.52*pow(10,-4)* pow(TMP_Value,3) - 3.05*pow(10,-2) * pow(TMP_Value,2) + 2.81 * TMP_Value - 30.6;
    
  
    debugPrint("Temperature [");
    debugPrint("TMP_GAM_FET.2");
    debugPrint("]: ");
    debugPrintln(TMP_Value_Calibrated);
    pyPrint("Temperature [");
    pyPrint("TMP_GAM_FET.2");
    pyPrint("]: ");
    pyPrintln(TMP_Value_Calibrated);


    return TMP_Value_Calibrated;

  }
  else if (n == 2){
    float TMP_Value_Calibrated = 1.54*pow(10,-4)* pow(TMP_Value,3) - 3.08*pow(10,-2) * pow(TMP_Value,2) + 2.85 * TMP_Value - 31.5;
    

    debugPrint("Temperature [");
    debugPrint("TMP_DUMMY_FET");
    debugPrint("]: ");
    debugPrintln(TMP_Value_Calibrated);
    pyPrint("Temperature [");
    pyPrint("TMP_DUMMY_FET");
    pyPrint("]: ");
    pyPrintln(TMP_Value_Calibrated);

    
    return TMP_Value_Calibrated;
  }
  else if (n == 3){
    float TMP_Value_Calibrated = 1.52*pow(10,-4)* pow(TMP_Value,3) - 3.05*pow(10,-2) * pow(TMP_Value,2) + 2.80 * TMP_Value - 30.5;
    

    debugPrint("Temperature [");
    debugPrint("TMP_GAM_Diode");
    debugPrint("]: ");
    debugPrintln(TMP_Value_Calibrated);
    pyPrint("Temperature [");
    pyPrint("TMP_GAM_Diode");
    pyPrint("]: ");
    pyPrintln(TMP_Value_Calibrated);


    return TMP_Value_Calibrated;
  }
  else if (n == 4){
    float TMP_Value_Calibrated = 1.53*pow(10,-4)* pow(TMP_Value,3) - 3.06*pow(10,-2) * pow(TMP_Value,2) + 2.83 * TMP_Value - 31.0;
    

    debugPrint("Temperature [");
    debugPrint("TMP_GAM_FET.1");
    debugPrint("]: ");
    debugPrintln(TMP_Value_Calibrated);
    pyPrint("Temperature [");
    pyPrint("TMP_GAM_FET.1");
    pyPrint("]: ");
    pyPrintln(TMP_Value_Calibrated);

    
    return TMP_Value_Calibrated;
  }
  else if (n == 5){
    float TMP_Value_Calibrated = 1.54*pow(10,-4)* pow(TMP_Value,3) - 3.09*pow(10,-2) * pow(TMP_Value,2) + 2.85 * TMP_Value - 31.5;
    
    debugPrint("Temperature [");
    debugPrint("TMP_OBC_RES-out");
    debugPrint("]: ");
    debugPrintln(TMP_Value_Calibrated);
    pyPrint("Temperature [");
    pyPrint("TMP_OBC_RES-out");
    pyPrint("]: ");
    pyPrintln(TMP_Value_Calibrated);


    return TMP_Value_Calibrated;
  }
  else if (n == 6){
    float TMP_Value_Calibrated = 1.54*pow(10,-4)* pow(TMP_Value,3) - 3.09*pow(10,-2) * pow(TMP_Value,2) + 2.85 * TMP_Value - 31.6;
    
    debugPrint("Temperature [");
    debugPrint("TMP_OBC_Diode");
    debugPrint("]: ");
    debugPrintln(TMP_Value_Calibrated);
    pyPrint("Temperature [");
    pyPrint("TMP_OBC_Diode");
    pyPrint("]: ");
    pyPrintln(TMP_Value_Calibrated);

    return TMP_Value_Calibrated;
  }
  else if (n == 7){
    float TMP_Value_Calibrated = 1.53*pow(10,-4)* pow(TMP_Value,3) - 3.06*pow(10,-2) * pow(TMP_Value,2) + 2.83 * TMP_Value - 31.2;
    
    debugPrint("Temperature [");
    debugPrint("TMP_OBC_FET.2");
    debugPrint("]: ");
    debugPrintln(TMP_Value_Calibrated);
    pyPrint("Temperature [");
    pyPrint("TMP_OBC_FET.2");
    pyPrint("]: ");
    pyPrintln(TMP_Value_Calibrated);
    
    return TMP_Value_Calibrated;
  }
  else if (n == 8){
    float TMP_Value_Calibrated = 1.53*pow(10,-4)* pow(TMP_Value,3) - 3.07*pow(10,-2) * pow(TMP_Value,2) + 2.83 * TMP_Value - 31.0;
    
    debugPrint("Temperature [");
    debugPrint("TMP_OBC_RLCL");
    debugPrint("]: ");
    debugPrintln(TMP_Value_Calibrated);
    pyPrint("Temperature [");
    pyPrint("TMP_OBC_RLCL");
    pyPrint("]: ");
    pyPrintln(TMP_Value_Calibrated);

    return TMP_Value_Calibrated;
  }
  
  //debugPrint(byte_1, HEX, ' ',  byte_2 , HEX, '\n')
  

  

}

float getTMP (String command){
  //command.toLowerCase();
  SPI.beginTransaction(SPI_STANDARD_SETTINGS);
  set_selected_IC("ADC2");
  if(command.equals("1")){
    float temperature = SPI_read_TMP(selectedIC_CS, 0x06, 0x00, 1);
    SPI.endTransaction();
    return temperature;
  }
  if(command.equals("2")){
    float temperature = SPI_read_TMP(selectedIC_CS, 0x06, 0x80, 2);
    SPI.endTransaction();
    return temperature;
  }
  if(command.equals("3")){
    float temperature = SPI_read_TMP(selectedIC_CS, 0x06, 0x40, 3); 
    SPI.endTransaction();
    return temperature;
  }
  if(command.equals("4")){
    float temperature = SPI_read_TMP(selectedIC_CS, 0x07, 0x00, 4); 
    SPI.endTransaction();
    return temperature;
  }
  if(command.equals("5")){
    float temperature = SPI_read_TMP(selectedIC_CS, 0x06, 0xc0, 5); 
    SPI.endTransaction();
    return temperature;
  }
  if(command.equals("6")){
    float temperature = SPI_read_TMP(selectedIC_CS, 0x07, 0x80, 6); 
    SPI.endTransaction();
    return temperature;
  }
  if(command.equals("7")){
    float temperature = SPI_read_TMP(selectedIC_CS, 0x07, 0x40, 7); 
    SPI.endTransaction();
    return temperature;
  }
  if(command.equals("8")){
    float temperature = SPI_read_TMP(selectedIC_CS, 0x07, 0xc0, 8);
    SPI.endTransaction();
    return temperature;
  }
  if(command.equals("all")){
    float temperature_1 = SPI_read_TMP(selectedIC_CS, 0x06, 0x00, 1);
    float temperature_2 = SPI_read_TMP(selectedIC_CS, 0x06, 0x80, 2);
    float temperature_3 = SPI_read_TMP(selectedIC_CS, 0x06, 0x40, 3);
    float temperature_4 = SPI_read_TMP(selectedIC_CS, 0x07, 0x00, 4);
    float temperature_5 = SPI_read_TMP(selectedIC_CS, 0x06, 0xc0, 5);
    float temperature_6 = SPI_read_TMP(selectedIC_CS, 0x07, 0x80, 6);
    float temperature_7 = SPI_read_TMP(selectedIC_CS, 0x07, 0x40, 7);
    float temperature_8 = SPI_read_TMP(selectedIC_CS, 0x07, 0xc0, 8);
    SPI.endTransaction();
    return 0;
  }
}
//##############################################################################




//######################      [SPI] voltage sensors      #######################
float SPI_read_VHK(int CS_pin, byte byte_1, byte byte_2, String VHK_id, float A_calib_const, float B_calib_const, float offset) {


  digitalWrite(CS_pin, LOW);

  byte byte1 = SPI.transfer(byte_1);
  byte byte2 = SPI.transfer(byte_2);
  byte byte3 = SPI.transfer(0x00);
 
  digitalWrite(CS_pin, HIGH);


  String line_print = "n/a";

  if (VHK_id.equals("VHK_OBC")){
    line_print = "VHK_OBC";
  }
  else if (VHK_id.equals("VHK_OBC_out")){
    line_print = "VHK_OBC_out";
  }
  else if (VHK_id.equals("VHK_GAM_out")){
    line_print = "VHK_GAM_out";
  }
  else if (VHK_id.equals("VHK_GAM")){
    line_print = "VHK_GAM";
  }

  byte byte2_trimmed = byte2 & 0x0f;

  uint16_t combinedValue = (uint16_t(byte2_trimmed) << 8) | byte3;
  //debugPrint("Combined Bytes:");
  //debugPrintln(combinedValue, BIN);

  int intValue = (int)combinedValue;
  //debugPrint("Int ADC number: ");
  //debugPrintln(intValue);
  int intValue_offset = intValue - offset; //ADC zero point off-set number

  float VHK_Value = (0.00427 * intValue_offset) * A_calib_const + B_calib_const;

  //float V_in_ADC = ((intValue_offset * 5)/4096)*calib_const;
  /*
  debugPrint("----------");
  debugPrint(line_print);
  debugPrintln("----------");
  debugPrint("Voltage (V): ");
  debugPrintln(VHK_Value);

  debugPrintln("------------------------------");
  debugPrintln("");
  debugPrintln("------------------------------");
  */
  
  debugPrint("Voltage [");
  debugPrint(line_print);
  debugPrint("]: ");
  debugPrintln(VHK_Value);
  pyPrint("Voltage [");
  pyPrint(line_print);
  pyPrint("]: ");
  pyPrintln(VHK_Value);


  return VHK_Value;
}

float getVHK (String command){
  //command.toLowerCase();
  SPI.beginTransaction(SPI_STANDARD_SETTINGS);

  set_selected_IC("ADC1");
  if(command.equals("VHK_OBC")){
    float voltage = SPI_read_VHK(selectedIC_CS, 0x06, 0x40, command, VHK_OBC_A_calib_const, VHK_OBC_B_calib_const, VHK_OBC_offset);
    SPI.endTransaction();
    return voltage;
  }
  if(command.equals("VHK_OBC_out")){
    float voltage = SPI_read_VHK(selectedIC_CS, 0x06, 0xc0, command, VHK_OBC_out_A_calib_const, VHK_OBC_out_B_calib_const, VHK_OBC_out_offset);
    SPI.endTransaction();
    return voltage;
  }
  if(command.equals("VHK_GAM_out")){
    float voltage = SPI_read_VHK(selectedIC_CS, 0x07, 0x00, command, VHK_GAM_out_A_calib_const, VHK_GAM_out_B_calib_const, VHK_GAM_out_offset);
    SPI.endTransaction();
    return voltage;
  }
  if(command.equals("VHK_GAM")){
    float voltage = SPI_read_VHK(selectedIC_CS, 0x07, 0x40, command, VHK_GAM_A_calib_const, VHK_GAM_B_calib_const, VHK_GAM_offset);
    SPI.endTransaction();
    return voltage;
  }
  if(command.equals("all")){

    float voltage_obc = SPI_read_VHK(selectedIC_CS, 0x06, 0x40, command, VHK_OBC_A_calib_const, VHK_OBC_B_calib_const, VHK_OBC_offset);
    float voltage_obc_out = SPI_read_VHK(selectedIC_CS, 0x06, 0xc0, command, VHK_OBC_out_A_calib_const, VHK_OBC_out_B_calib_const, VHK_OBC_out_offset);
    float voltage_gam = SPI_read_VHK(selectedIC_CS, 0x07, 0x40, command, VHK_GAM_A_calib_const, VHK_GAM_B_calib_const, VHK_GAM_offset);
    float voltage_gam_out = SPI_read_VHK(selectedIC_CS, 0x07, 0x00, command, VHK_GAM_out_A_calib_const, VHK_GAM_out_B_calib_const, VHK_GAM_out_offset);
    SPI.endTransaction();
    return 0;
  }

}
//##############################################################################




//######################      [SPI] current sensors      #######################
float SPI_read_CHK(int CS_pin, byte byte_1, byte byte_2, String CHK_id, float A_calib_const, float B_calib_const, int offset) {
  digitalWrite(CS_pin, LOW);

  byte byte1 = SPI.transfer(byte_1);
  byte byte2 = SPI.transfer(byte_2);
  byte byte3 = SPI.transfer(0x00);

  byte fill = 00000000;
  digitalWrite(CS_pin, HIGH);

  byte byte2_trimmed = byte2 & 0x0f;

  uint16_t combinedValue = (uint16_t(byte2_trimmed) << 8) | byte3;

  //debugPrint("Combined Bytes:");
  //debugPrintln(combinedValue, BIN);

  int intValue = (int)combinedValue;

  //debugPrint("intValue ADC number: ");
  //debugPrintln(intValue);

  int intValue_noLoad = intValue - offset;

  float CHK_Value = 0;

  String line_print = "n/a";

  if (CHK_id.equals("CHK_OBC")){
    line_print = "CHK_OBC";
    float CHK_Value = round(((1.22*intValue_noLoad)/(0.08*25000) * pow(10, 3)) * A_calib_const + B_calib_const);
    debugPrint("Current [");
    debugPrint(line_print);
    debugPrint("]: ");
    debugPrintln(CHK_Value);
    pyPrint("Current [");
    pyPrint(line_print);
    pyPrint("]: ");
    pyPrintln(CHK_Value);
    return CHK_Value;

  }
  else if (CHK_id.equals("CHK_GAM")){
    line_print = "CHK_GAM";
    float CHK_Value = round(((1.22*intValue_noLoad)/(0.08*50000) * pow(10, 3))* A_calib_const + B_calib_const);
  
    debugPrint("Current [");
    debugPrint(line_print);
    debugPrint("]: ");
    debugPrintln(CHK_Value);
    pyPrint("Current [");
    pyPrint(line_print);
    pyPrint("]: ");
    pyPrintln(CHK_Value);
    return CHK_Value;
    
  }
  else if (CHK_id.equals("CHK_SR")){
    line_print = "CHK_SR";
    float CHK_Value = round(((1.22*intValue_noLoad)/(0.08*40000) * pow(10, 3)) * A_calib_const + B_calib_const);
    
    debugPrint("Current [");
    debugPrint(line_print);
    debugPrint("]: ");
    debugPrintln(CHK_Value);
    pyPrint("Current [");
    pyPrint(line_print);
    pyPrint("]: ");
    pyPrintln(CHK_Value);
    return CHK_Value;
    //debugPrint("Mean CHK/10 = ");
    //mean_CHK_SR = (mean_CHK_SR + CHK_Value);
    //float to_show_value = mean_CHK_SR / 10;
    //debugPrintln(to_show_value);
  }

  
}

float getCHK (String command){
  //command.toLowerCase();
  SPI.beginTransaction(SPI_STANDARD_SETTINGS);

  set_selected_IC("ADC1");
  if(command.equals("CHK_OBC")){
    float current = SPI_read_CHK(selectedIC_CS, 0x06, 0x80, command, CHK_OBC_A_calib_const, CHK_OBC_B_calib_const, CHK_OBC_noLoad_offset);
    SPI.endTransaction();
    return current;
  }
  if(command.equals("CHK_GAM")){
    float current = SPI_read_CHK(selectedIC_CS, 0x07, 0x80, command, CHK_GAM_A_calib_const, CHK_GAM_B_calib_const, CHK_GAM_noLoad_offset);
    SPI.endTransaction();
    return current;
  }
  if(command.equals("CHK_SR")){
    float current = SPI_read_CHK(selectedIC_CS, 0x07, 0xc0, command, CHK_SR_A_calib_const, CHK_SR_B_calib_const, CHK_SR_noLoad_offset);
    SPI.endTransaction();
    return current;
  }
  if(command.equals("all")){
    float current_obc = SPI_read_CHK(selectedIC_CS, 0x06, 0x80, command, CHK_OBC_A_calib_const, CHK_OBC_B_calib_const, CHK_OBC_noLoad_offset);
    float current_gam = SPI_read_CHK(selectedIC_CS, 0x07, 0x80, command, CHK_GAM_A_calib_const, CHK_GAM_B_calib_const, CHK_GAM_noLoad_offset);
    float current_sr = SPI_read_CHK(selectedIC_CS, 0x07, 0xc0, command, CHK_SR_A_calib_const, CHK_SR_B_calib_const, CHK_SR_noLoad_offset);
    SPI.endTransaction();
    return 0;
  }
}
//##############################################################################




//######################      [SPI] pressure sensor      #######################
float SPI_read_Pressure_Data(String HK_type) {
  set_selected_IC("BAR");

  SPI.beginTransaction(SPISettings(100000, MSBFIRST, SPI_MODE1));

  if (HK_type.equals("Pressure")){

    // get pressure
    digitalWrite(selectedIC_CS, LOW);
    byte should_be_pressurerandom1 = SPI.transfer(0x20);  // Send high byte of the command
    byte should_be_pressurerandom2 = SPI.transfer(0x00);   // Send low byte of the command
    digitalWrite(selectedIC_CS, HIGH);


    digitalWrite(selectedIC_CS, LOW);
    byte response_byte1 = SPI.transfer(0xe0);  // asks for sensor ID, next read gives the sensor ID
    byte response_byte2 = SPI.transfer(0x00);  // this read should give 0101000100110111 - 0x51 0x37
    digitalWrite(selectedIC_CS, HIGH); // maybe add a global variable to check if pressure sensor is operating nominaly

    uint16_t response_raw = (uint16_t(response_byte1) << 8) | response_byte2;
    uint8_t diagnosis = (response_raw >> 11) & 0x1F;
    uint16_t pressure_raw = (response_raw >> 1) & 0x3FF;
    uint8_t parity = response_raw & 0x01;

    float pressure_value = ((float)pressure_raw+ 545.6) /13.64;    

    SPI.endTransaction();
    
  // Return the pressure value (maybe also return diagnosis or handle errors)
  return pressure_value;
  }

  else if (HK_type.equals("Temperature")){
    // get temeprature
    digitalWrite(selectedIC_CS, LOW);
    byte should_be_temperaturerandom1 = SPI.transfer(0x40);  // Send high byte of the command
    byte should_be_temperaturerandom2 = SPI.transfer(0x00);   // Send low byte of the command
    digitalWrite(selectedIC_CS, HIGH);

    digitalWrite(selectedIC_CS, LOW);
    byte response_temperature_byte1 = SPI.transfer(0xe0);  // asks for sensor ID, next read gives the sensor ID
    byte response_temperature_byte2 = SPI.transfer(0x00);  // this read should give 0101000100110111 - 0x51 0x37
    digitalWrite(selectedIC_CS, HIGH);

    uint16_t response_raw = (uint16_t(response_temperature_byte1) << 8) | response_temperature_byte2;
    uint8_t diagnosis = (response_raw >> 11) & 0x1F;
    uint16_t temperature_raw = (response_raw >> 1) & 0x3FF;
    uint8_t parity = response_raw & 0x01;

    float temperature = ((float)temperature_raw  - 209.6 )/ 5.115;

    SPI.endTransaction();

    return temperature;
  }
}
//##############################################################################




//#####################             [SPI] DAC             ######################
void SPI_DAC_write(byte highByte, byte lowByte) {
  
  byte _config = 0b0011;
  byte write_highByte = (_config << 4) | highByte ;

  byte byte1 = SPI.transfer(write_highByte);
  byte byte2 = SPI.transfer(lowByte);

}

bool SPI_Dummy_DAC_write(String DAC_ID, byte highByte, byte lowByte){
  SPI.beginTransaction(SPI_STANDARD_SETTINGS);
  set_selected_IC("GPIO_DUMMY");
  
  if (DAC_ID.equals("DAC1") || DAC_ID.equals("VS_OBC")){
    ballonPrintln("writing to VS_OBC");
    SPI_write(selectedIC_CS, 0x09, 0xef); // set CS low
    SPI_DAC_write(highByte, lowByte);
    SPI_write(selectedIC_CS, 0x09, 0xff); // set CS High
    SPI.endTransaction();
    return true;
  }
  else if (DAC_ID.equals("DAC2") || DAC_ID.equals("VS_GAM.1")){
    ballonPrintln("writing to VS_GAM.1");
    SPI_write(selectedIC_CS, 0x09, 0xdf); // set CS low
    SPI_DAC_write(highByte, lowByte);
    SPI_write(selectedIC_CS, 0x09, 0xff); // set CS High
    SPI.endTransaction();
    return true;
  }
  else if (DAC_ID.equals("DAC3") || DAC_ID.equals("VS_GAM.2")){
    SPI_write(selectedIC_CS, 0x09, 0xbf); // set CS low
    SPI_DAC_write(highByte, lowByte);
    SPI_write(selectedIC_CS, 0x09, 0xff); // set CS High
    SPI.endTransaction();
    return true;
  }
  else if (DAC_ID.equals("DAC4") || DAC_ID.equals("VS_GAM.3")){
    SPI_write(selectedIC_CS, 0x09, 0x7f); // set CS low
    SPI_DAC_write(highByte, lowByte);
    SPI_write(selectedIC_CS, 0x09, 0xff); // set CS High
    SPI.endTransaction();
    return true;
  }
  else if (DAC_ID.equals("DAC5") || DAC_ID.equals("VS_GAM.4")){
    SPI_write(selectedIC_CS, 0x09, 0xfe); // set CS low
    SPI_DAC_write(highByte, lowByte);
    SPI_write(selectedIC_CS, 0x09, 0xff); // set CS High
    SPI.endTransaction();
    return true;
  }
  else if (DAC_ID.equals("DAC6") || DAC_ID.equals("VS_PAR.1")){
    SPI_write(selectedIC_CS, 0x09, 0xfd); // set CS low
    SPI_DAC_write(highByte, lowByte);
    SPI_write(selectedIC_CS, 0x09, 0xff); // set CS High
    SPI.endTransaction();
    return true;
  }
  else if (DAC_ID.equals("DAC7") || DAC_ID.equals("VS_PAR.2")){
    SPI_write(selectedIC_CS, 0x09, 0xfb); // set CS low
    SPI_DAC_write(highByte, lowByte);
    SPI_write(selectedIC_CS, 0x09, 0xff); // set CS High
    SPI.endTransaction();
    return true;
  }
  else{
    ballonPrintln("Error DAC: select available DAC's (1-7)");
    SPI.endTransaction();
    return false;
  }
}
//##############################################################################




//######################       [PDU] operation mode      #######################
void changeOPmode (String mode){

  SPI.beginTransaction(SPI_STANDARD_SETTINGS);
  set_selected_IC("GPIO_PDU");
  mode.toLowerCase();

  if (mode.equals("power save")){
    SPI_write(selectedIC_CS, 0x09, 0x30);
    byte read_check = SPI_read(selectedIC_CS, 0x09);
  }

  else if (mode.equals("power ready")){
    SPI_write(selectedIC_CS, 0x09, 0x20);
    byte read_check = SPI_read(selectedIC_CS, 0x09);
  }

  else if (mode.equals("gam")){
    SPI_write(selectedIC_CS, 0x09, 0x24);
    byte read_check = SPI_read(selectedIC_CS, 0x09);
  }

  else if (mode.equals("gam redundancy")){
    SPI_write(selectedIC_CS, 0x09, 0x14);
    byte read_check = SPI_read(selectedIC_CS, 0x09);
  }

  else if (mode.equals("obc")){
    SPI_write(selectedIC_CS, 0x09, 0x12);
    byte read_check = SPI_read(selectedIC_CS, 0x09);
  }

  else if (mode.equals("obc redundancy")){
    SPI_write(selectedIC_CS, 0x09, 0x22);
    byte read_check = SPI_read(selectedIC_CS, 0x09);
  }

  else if (mode.equals("full load")){
    SPI_write(selectedIC_CS, 0x09, 0x26);
    byte read_check = SPI_read(selectedIC_CS, 0x09);
  }

  else if (mode.equals("full load redundancy")){
    SPI_write(selectedIC_CS, 0x09, 0x16);
    byte read_check = SPI_read(selectedIC_CS, 0x09);
  }
  
  ballonPrint("Successfull Mode change: ");
  ballonPrint(mode);
  SPI.endTransaction();

}
//##############################################################################




//#####################          [DAC] operation          ######################
bool ON_Dummy_DAC(String DAC_ID){
  SPI.beginTransaction(SPI_STANDARD_SETTINGS);
  set_selected_IC("GPIO_DUMMY");

  if (DAC_ID.equals("DAC1") || DAC_ID.equals("VS_OBC")){
    SPI_write(selectedIC_CS, 0x19, 0x08); //write ONOFF GPIO pin to LOW to turn ON the DAC
    SPI.endTransaction();
    return true;
  }
  else if (DAC_ID.equals("DAC2") || DAC_ID.equals("VS_GAM.1")){
    SPI_write(selectedIC_CS, 0x19, 0x04);
    SPI.endTransaction();
    return true;
  }
  else if (DAC_ID.equals("DAC3") || DAC_ID.equals("VS_GAM.2")){
    SPI_write(selectedIC_CS, 0x19, 0x02);
    SPI.endTransaction();
    return true;
  }
  else if (DAC_ID.equals("DAC4") || DAC_ID.equals("VS_GAM.3")){
    SPI_write(selectedIC_CS, 0x19, 0x01);
    SPI.endTransaction();
    return true;
  }
  else if (DAC_ID.equals("DAC5") || DAC_ID.equals("VS_GAM.4")){
    SPI_write(selectedIC_CS, 0x19, 0x10);
    SPI.endTransaction();
    return true;
  }
  else if (DAC_ID.equals("DAC6") || DAC_ID.equals("VS_PAR.1")){
    SPI_write(selectedIC_CS, 0x19, 0x20);
    SPI.endTransaction();
    return true;
  }
  else if (DAC_ID.equals("DAC7") || DAC_ID.equals("VS_PAR.2")){
    SPI_write(selectedIC_CS, 0x19, 0x40);
    SPI.endTransaction();
    return true;
  }
  else{
    ballonPrintln("Error DAC: select available DAC's (1-7)");
    SPI.endTransaction();
    return false;
  }
}

bool OFF_Dummy_DAC(String DAC_ID){
  SPI.beginTransaction(SPI_STANDARD_SETTINGS);
  set_selected_IC("GPIO_DUMMY");

  if (DAC_ID.equals("DAC1") || DAC_ID.equals("VS_OBC")){
    SPI_write(selectedIC_CS, 0x19, 0x00);  //write ONOFF GPIO pin to LOW to turn OFF the DAC
    SPI.endTransaction();
    return true;
  }
  else if (DAC_ID.equals("DAC2") || DAC_ID.equals("VS_GAM.1")){
    SPI_write(selectedIC_CS, 0x19, 0x00);
    SPI.endTransaction();
    return true;
  }
  else if (DAC_ID.equals("DAC3") || DAC_ID.equals("VS_GAM.2")){
    SPI_write(selectedIC_CS, 0x19, 0x00);
    SPI.endTransaction();
    return true;
  }
  else if (DAC_ID.equals("DAC4") || DAC_ID.equals("VS_GAM.3")){
    SPI_write(selectedIC_CS, 0x19, 0x00);
    SPI.endTransaction();
    return true;
  }
  else if (DAC_ID.equals("DAC5") || DAC_ID.equals("VS_GAM.4")){
    SPI_write(selectedIC_CS, 0x19, 0x00);
    SPI.endTransaction();
    return true;
  }
  else if (DAC_ID.equals("DAC6") || DAC_ID.equals("VS_PAR.1")){
    SPI_write(selectedIC_CS, 0x19, 0x00);
    SPI.endTransaction();
    return true;
  }
  else if (DAC_ID.equals("DAC7") || DAC_ID.equals("VS_PAR.2")){
    SPI_write(selectedIC_CS, 0x19, 0x00);
    SPI.endTransaction();
    return true;
  }
  else{
    ballonPrintln("Error DAC: select available DAC's (1-7)");
    SPI.endTransaction();
    return false;
  }
}

bool setDummyLoad(String Output, int mA, bool Status){
  // Output: select output to test, See ON_Dummy_DAC() function
  // mA: set load in mA
  // Status: write True to turn ON, False to Turn OFF

  bool check = false;

  if (Status == true){
    bool DAC_ON = ON_Dummy_DAC(Output);
    if (DAC_ON == true){
      float _load = DAC_a * mA + DAC_b;
      float R1 = 15.0;
      float R2 = 2.0;

      int _Ndac = (_load/(10000*(R2/(R1+R2))*2.048))*4096;

      ballonPrint("_Ndac: ");
      ballonPrintln(_Ndac);

      if (_Ndac < 0 || _Ndac > 4096){

        File log = SD.open("/log.txt", FILE_APPEND);

        log.printf("%d: setDummyLoad() - Error: I_Load is out of the valid range (0-2400 mA)\n",millis());

        log.close();

        ballonPrintln("Error: I_Load is out of the valid range (0-2400 mA)");
        return check;
      }
      byte highByte = (_Ndac >> 8) & 0xFF; 
      byte lowByte = _Ndac & 0xFF;
      ballonPrintln("writing SPI_Dummy_DAC");
      SPI_Dummy_DAC_write(Output, highByte, lowByte);

      check = true;

    }
  }

  else if (Status == false){
    bool DAC_OFF = OFF_Dummy_DAC(Output);
    if (DAC_OFF == true){
      ballonPrint("Output ");
      ballonPrint(Output);
      ballonPrintln(" turned OFF");

      check = true;

    }
    else{
      ballonPrintln("CRITICAL ERROR: DAC didn't turn OFF. Hard shutdown the System NOW!!!!");

      File log = SD.open("/log.txt", FILE_APPEND);

      log.printf("%d: setDummyLoad() - CRITICAL ERROR: DAC didn't turn OFF. Hard shutdown the System NOW!!!!\n",millis());

      log.close();

      check = false;
    }
  }
  
  return check;

}
//##############################################################################




//#####################           [LCL] testing           ######################
struct testLCLParameters{
  String nameLCL;
  int aqFrequency;
  int runTime;
};

void testLCLreadSensors(void *pvParameters){
  testLCLParameters *params = (testLCLParameters *)pvParameters;
  String nameLCL = params->nameLCL;
  int aqFrequency = params->aqFrequency;
  int runTime = params->runTime;

  float PeriodUs = (1000000.0/aqFrequency); // ms
  int nIterations = (runTime/1000.0)*aqFrequency;
  const int bufferSize = static_cast<int>((4*aqFrequency*(runTime/1000.0))*1.05); // @2kHz, int=4bytes, buffersize=(4bytes*2k*time)*1.1 , (10% margin)
  ballonPrint("bufferSize: ");
  ballonPrint(bufferSize);
  ballonPrintln(" bytes");
  int bufferIndex = 0;         
  bool bufferFull = false;    

  size_t freeMemory = heap_caps_get_free_size(MALLOC_CAP_DEFAULT);
  ballonPrint("Free heap memory before allocation: ");
  ballonPrintln(freeMemory);

  std::vector<float> CHKData(bufferSize);
  std::vector<float> VHKData(bufferSize);
  std::vector<float> VHK_outData(bufferSize);
  std::vector<int> real_time(bufferSize);

  size_t freeMemory_after = heap_caps_get_free_size(MALLOC_CAP_DEFAULT);
  ballonPrint("Free heap memory after allocation: ");
  ballonPrintln(freeMemory_after);

  unsigned long LCLstartTime = millis();
  int count = 0;
  unsigned long lastExecutionTime = esp_timer_get_time(); // Get current time in microseconds


  if (nameLCL.equals("VS_OBC")){
    ballonPrintln("START LCL VS_OBC HK aquisition...");
    int64_t start_time, end_time;
    
    while (millis() - LCLstartTime < runTime){
      start_time = esp_timer_get_time(); // Get start time in microseconds

      if (xSemaphoreTake(spiMutex, pdMS_TO_TICKS(100)) == pdTRUE){
        CHKData[bufferIndex] = getCHK("CHK_OBC");
        VHKData[bufferIndex] = getVHK("VHK_OBC");
        VHK_outData[bufferIndex] = getVHK("VHK_OBC_out");
        real_time[bufferIndex] = millis();
        
        bufferIndex++;
        count++;

        if (bufferIndex >= bufferSize){
          ballonPrint(nameLCL);
          ballonPrintln(" Buffer is full. Check the code.");
          bufferIndex = 0; // reset index to avoid memory issues
          bufferFull = true;
        }
        xSemaphoreGive(spiMutex);
      }

      // Calculate the remaining time and busy-wait to ensure correct frequency
      end_time = esp_timer_get_time();
      int64_t elapsed_time = end_time - start_time;
      int64_t remaining_time = PeriodUs - elapsed_time;
      
      if (remaining_time > 0){
        delayMicroseconds(remaining_time); // precise microsecond delay
      }
    }
    ballonPrintln("END LCL VS_OBC HK aquisition...");
  }

  else if (nameLCL.equals("VS_GAM.1")){
    ballonPrintln("START LCL VS_GAM HK aquisition...");
    int64_t start_time, end_time;

    while (millis() - LCLstartTime < runTime){
      start_time = esp_timer_get_time(); // Get start time in microseconds

      //ADD: measure the temperature of the LCL IC
      if (xSemaphoreTake(spiMutex, pdMS_TO_TICKS(100)) == pdTRUE){
        CHKData[bufferIndex] = getCHK("CHK_GAM");
        VHKData[bufferIndex] = getVHK("VHK_GAM");
        VHK_outData[bufferIndex] = getVHK("VHK_GAM_out");
        real_time[bufferIndex] = millis();
        
        bufferIndex++;
        count++;

        if (bufferIndex >= bufferSize){
          ballonPrint(nameLCL);
          ballonPrintln(" Buffer is full. Check the code.");
          bufferIndex = 0; // reset index to avoid memory issues
          bufferFull = true;
        }
        xSemaphoreGive(spiMutex);
      }

      // Calculate the remaining time and busy-wait to ensure correct frequency
      end_time = esp_timer_get_time();
      int64_t elapsed_time = end_time - start_time;
      int64_t remaining_time = PeriodUs - elapsed_time;
      
      if (remaining_time > 0){
        delayMicroseconds(remaining_time); // precise microsecond delay
      }
    }

  ballonPrintln("END LCL VS_GAM HK aquisition...");
  }


  File LCLDataFile = SD.open("/LCLDataFile.txt", FILE_APPEND);

  int i;

  if (LCLDataFile){
    ballonPrintln("Writing LCLData to SD...");
    LCLDataFile.printf("LCLTest,%s\nFrequency,%d\nRunTime,%d\nindex,millis,CHK,VHK,VHK_out\n", nameLCL.c_str(), aqFrequency, runTime);

    char buffer[128];
    for (i = 0; i < bufferIndex; i++) {
      snprintf(buffer, sizeof(buffer), "%d,%d,%.2f,%.2f,%.2f\n", i, real_time[i], CHKData[i], VHKData[i], VHK_outData[i]);
      LCLDataFile.print(buffer);
    }
    LCLDataFile.close();
    
    ballonPrintln("End of SD LCLData write.");
    ballonPrint("Last i: ");
    ballonPrintln(i);
  }
  else{

    // maybe check if the file exists on the loop before calling the function
    ballonPrintln("Error opening the LCLDataFile.");
    ballonPrintln("Creating file...");
    ballonPrintln("Try again the function");
    File LCLDataFile = SD.open("/LCLDataFile.txt", FILE_WRITE);
    LCLDataFile.close();    
    
    testLCLParameters params = {nameLCL, aqFrequency, runTime};

    xTaskCreatePinnedToCore(
      testLCLreadSensors,  // function
      "LCL Task",          // name of the task
      10000,               // stack size in words (idk)
      &params,             // Task input parameters, from TaskParameters struct testLCLParameters
      1,                   // priority
      NULL,              // Pointer to store task handle (idk if i need)
      0);                  // core ID

    vTaskDelete(NULL);
  }

  CHKData.clear();
  CHKData.shrink_to_fit();

  VHKData.clear();
  VHKData.shrink_to_fit();

  VHK_outData.clear();
  VHK_outData.shrink_to_fit();

  real_time.clear();
  real_time.shrink_to_fit();

  //digitalWrite(LED, LOW);

  vTaskDelete(NULL);

}

void testLCLtrigger(void *pvParameters){
  testLCLParameters *params = (testLCLParameters *)pvParameters;
  String nameLCL = params->nameLCL;
  int aqFrequency = params->aqFrequency;
  int runTime = params->runTime;

  int stable_load;

  if (nameLCL.equals("VS_OBC")){
    stable_load = 1200;
  }
  else{
    stable_load = 450;
  }

  // change the test to let come load at the output and check if it recuperates
  // have a wait time of 1 min or more of time under 2100 load and also measure the 
  // temperature of LCL IC. 
  delay(runTime*0.05);
  int setStableLoad = millis();
  setDummyLoad(nameLCL, stable_load, true);
  delay(runTime*0.1);
  int triggerTimeBefore = millis();
  setDummyLoad(nameLCL, 2100, true); 
  //ballonPrintln("LCLtrigger");
  int triggerTimeAfter = millis();
  delay(runTime*0.75);
  setDummyLoad(nameLCL, 10, false);

  ballonPrintln("Saving on SD millis() mode change.");

  File LCLTriggerFile = SD.open("/LCLTrigger.txt", FILE_APPEND);

  // need to add the : LCLname and OP mode.
  if (LCLTriggerFile){
    ballonPrintln("Writing LCLTrigger to SD...");
    LCLTriggerFile.printf("LCLTrigger\nNameLCL,%s\nRunTime,%d\nSetTableLoad,%d\nTriggerBeforeFunctionCall,%d\nTriggerAfterFunctionCall, %d\n", nameLCL, runTime, setStableLoad, triggerTimeBefore, triggerTimeAfter);
    LCLTriggerFile.close();
    ballonPrintln("End of SD LCLTrigger write.");

    File log = SD.open("/log.txt", FILE_APPEND);

    log.printf("%d: testLCLtrigger() - LCL Test Sucessfull, LCL %s, freq %i, runTime %i \n",millis(), nameLCL, aqFrequency, runTime);

    log.close();
  }
  else{
    // maybe check if the file exists on the loop before calling the function
    ballonPrintln("Error opening the LCLDataFile.");
    ballonPrintln("Creating file...");
    ballonPrintln("Try again the function");
    File LCLDataFile = SD.open("/LCLDataFile.txt", FILE_WRITE);
    LCLDataFile.close();   

    File log = SD.open("/log.txt", FILE_APPEND);

      log.printf("%d: testLCLtrigger() - ERROR opening LCLDataFile, Trying again, LCL %s, freq %i, runTime %i \n",millis(), nameLCL, aqFrequency, runTime);

      log.close(); 

    testLCLParameters params = {nameLCL, aqFrequency, runTime};

    xTaskCreatePinnedToCore(
      testLCLreadSensors,  // function
      "LCL Task",          // name of the task
      10000,               // stack size in words (idk)
      &params,             // Task input parameters, from TaskParameters struct testLCLParameters
      1,                   // priority
      NULL,              // Pointer to store task handle (idk if i need)
      0);                  // core ID

    vTaskDelete(NULL);
  }
  vTaskDelete(NULL);
}

//this funtion needs testing
int tenKmTestTrigger(){
  // i need to read the last minute of velocity, make an average and then use that average
  // to compute the millis() on which to trigger the LCL tests, first perform the GAM
  // test and then perform the OBC send. Send the full data via ETH.

  int n_sample_velocity = 10;


  File HKmeasurement = SD.open("/HKmeasurement.txt", FILE_READ);

  if (!HKmeasurement) {
      File log = SD.open("/log.txt", FILE_APPEND);

      log.printf("%d: tenKmTestTrigger() - Error opening HKmeasurement.txt file.\n",millis());

      log.close();
      Serial.println("Error opening HKmeasurement.txt file.");
      return 0; // Handle this appropriately in the main function.
  }

  std::vector<float> velocityValues;
  int velocityIndex = 18;

  // Read the header line and split by commas
  String headerLine = HKmeasurement.readStringUntil('\n');
  Serial.println(headerLine);

  std::vector<String> columnNames;
  int startIndex = 0;
  int endIndex;


  // Read data rows
  while (HKmeasurement.available()) {
      String line = HKmeasurement.readStringUntil('\n');
      std::vector<String> columns;

      startIndex = 0;
      while ((endIndex = line.indexOf(",", startIndex)) != -1) {
          columns.push_back(line.substring(startIndex, endIndex));
          startIndex = endIndex + 1;
      }
      // Adding the last column value since it does not end with a comma
      columns.push_back(line.substring(startIndex));

      if (columns.size() > velocityIndex) {
          float velocity = columns[velocityIndex].toFloat();
          if (velocityValues.size() >= n_sample_velocity) {
              velocityValues.erase(velocityValues.begin());  // Remove the oldest value
          }
          velocityValues.push_back(velocity);
      }
  }

  HKmeasurement.close();

  float sum_velocity = 0;

  Serial.println("Last 10 Velocity Values:");
  for (int i = 0; i < velocityValues.size(); i++) {
      Serial.println(velocityValues[i]);
      sum_velocity += velocityValues[i];
  }

  float average_velocity = sum_velocity / velocityValues.size();

  ballonPrint("average_velocity 7km: ");
  ballonPrintln(average_velocity);

  File log = SD.open("/log.txt", FILE_APPEND);

  log.printf("%d: tenKmTestTrigger() - Average velocity 7km: %f \n",millis(), average_velocity);

  log.close();

  int distance_left = 3000;

  int tenK_wait_time = (3000 / average_velocity) * 1000; // in millis

  ballonPrint("Time to Wait: ");
  ballonPrintln(tenK_wait_time);

  log = SD.open("/log.txt", FILE_APPEND);

  log.printf("%d: tenKmTestTrigger() - Time to Wait: %d \n",millis(), tenK_wait_time);

  log.close();

  return tenK_wait_time;

}
//##############################################################################




//#####################         [ROUTINE] HK + COM         #####################

void readSensorData(){

  // To get TMP, CHK, VHK data and store the data on 2 lists, 
  // one list for the sensor ID (keyVector) the other list for the values (dataVector).
  // It stores the data on the SD card, .csv and sends it to a Server.

  ballonPrintln("inside read sensor DATA");
  std::vector<float> dataVector;
  std::vector<String> keyVector;

  float bar_value;
  float sum_pressure;
  float velocity_value;

  // get TMP values
  for (int i = 1; i < 9; i++){
    String tmp_id = String(i);
    float tmp_value = getTMP(tmp_id);

    dataVector.push_back(tmp_value);
    keyVector.push_back(tmp_id);
  }

  // get CHK values
  for (int j = 0; j < listCHK.size(); j++){
    String chk_ID = listCHK[j];
    float chk_value = getCHK(chk_ID);

    dataVector.push_back(chk_value);
    keyVector.push_back(chk_ID);    
  }

  // get VHK values
  for (int k = 0; k < listVHK.size(); k++){
    String vhk_ID = listVHK[k];
    float vhk_value = getVHK(vhk_ID);

    dataVector.push_back(vhk_value);
    keyVector.push_back(vhk_ID);
  }

  // get Pressure and TMP value from barometer IC
  for (int z = 0; z < listBAR.size(); z++){
    String bar_ID = listBAR[z];
    bar_value = SPI_read_Pressure_Data(bar_ID);

    dataVector.push_back(bar_value);
    keyVector.push_back(bar_ID);
  }


  // averaging 10 measurements of pressure
  std::vector<float> listAveragePressure;

  for (int increment = 0; increment < 10; increment++){
    float pressure = SPI_read_Pressure_Data("Pressure");

    listAveragePressure.push_back(pressure);
  }

  for (int w = 0; w < listAveragePressure.size(); w++){
    sum_pressure = sum_pressure + listAveragePressure[w];
  }

  float pressure_value = sum_pressure/10;

  //ballonPrint("Pressure 10 Average: ");
  //ballonPrintln(pressure_value);

  // calculate the ballon altitude and ascending velocity
  String altitude_ID = "Altitude";
  float altitude_value = ((1-pow(((pressure_value*10)/1013.25), 0.190284))*145366.45)*0.3048;

  dataVector.push_back(altitude_value);
  keyVector.push_back(altitude_ID);
  
  String velocity_ID = "Velocity";

  float velocity_timeDifference = (millis() - velocity_oldTime)/1000;
  //ballonPrint("velocity_timeDifference: ");
  //ballonPrintln(velocity_timeDifference);

  float altitude_difference = altitude_value - altitude_oldValue;

  //ballonPrint("Current altitude: ");
  //ballonPrintln(altitude_value);
  //ballonPrint("Old altitude: ");
  //ballonPrintln(altitude_oldValue);
  //ballonPrint("altitude_difference; ");
  //ballonPrintln(altitude_difference);


  velocity_value = altitude_difference / (velocity_timeDifference);

  //float velocity_value = (altitude_value - altitude_oldValue);  // test with 1 sec

  dataVector.push_back(velocity_value);
  keyVector.push_back(velocity_ID);


  velocity_oldTime = millis();
  altitude_oldValue = altitude_value;


  File HKmeasurement = SD.open("/HKmeasurement.txt", FILE_APPEND);

  if (HKmeasurement){
    int CHK_list_length = keyVector.size();
    ballonPrintln("Writing HK data to SD...");
    
    if (HKmeasurement.size() == 0) {
      needs_header = true;
      // Write the header (keyVector values)
      HKmeasurement.print("millis,");
      for (size_t i = 0; i < keyVector.size(); i++) {
        HKmeasurement.print(keyVector[i]);
        if (i < keyVector.size() - 1) {
          HKmeasurement.print(",");  
        }
      }
      HKmeasurement.println(); 
    }

    HKmeasurement.print(millis()); 
    HKmeasurement.print(",");

    for (size_t i = 0; i < dataVector.size(); i++) {
      HKmeasurement.print(dataVector[i]); 
      if (i < dataVector.size() - 1) {
        HKmeasurement.print(",");
      }
    }
    HKmeasurement.println(); 

    HKmeasurement.close();
    
    ballonPrintln("Successfull HK data writen to HKmeasurements.txt");

    File log = SD.open("/log.txt", FILE_APPEND);

    log.printf("%d: readSensorData() - Successfull HK data writen to HKmeasurements.txt \n",millis());

    log.close();
  }
  else {
    Serial.println("Error opening HKmeasurements.txt file.");

    File log = SD.open("/log.txt", FILE_APPEND);

    log.printf("%d: readSensorData() - Error opening HKmeasurements.txt file. \n",millis());

    log.close();
  }



  // send data do server
  if (client.connect(server, 8080)) {
    Serial.println("Server connection successful: ");
    Serial.println(client.remoteIP());


    client.print("{'sensor_id':'TH02','values':[HKData,\\n,");  

    /*if (needs_header == true){
      client.print("{'sensor_id'='TH01','values':['HKData','\n','millis','" + keyVector + "','\n','" + dataVector + "']}");
      
    }

    else{
      client.print("{'sensor_id'='THOR1','values':['HKData','\n','" + millis() + "','" + dataVector + "']}");
    }*/

    
    if (needs_header == true){
      needs_header = false;
      client.print("millis,");
      for (size_t i = 0; i < keyVector.size(); i++) {
        client.print(keyVector[i]);
        if (i < keyVector.size() - 1) {
          client.print(",");  
        }
     }
    }

    client.print("\\n,");
    client.print(millis()); 
    client.print(",");

    for (size_t i = 0; i < dataVector.size(); i++) {
      client.print(dataVector[i]); 
      if (i < dataVector.size() - 1) {
        client.print(",");
      }
    }
    client.print("]}");
    
    
    ballonPrintln("Successfull HK data writen to Server");

    File log = SD.open("/log.txt", FILE_APPEND);

    log.printf("%d: readSensorData() - Successfull HK data writen to Server \n",millis());

    log.close();

  }
  else {
    ballonPrintln("Connection to Server Failed while sending HK data.");
    File log = SD.open("/log.txt", FILE_APPEND);

    log.printf("%d: readSensorData() - Connection to Server Failed while sending HK data. \n",millis());

    log.close();
  }

  client.stop();



  ballonPrintln("");
  ballonPrintln("---------------------------");
  for (int i =0; i < dataVector.size(); i++) {
    
    ballonPrint("Key: ");
    ballonPrint(keyVector[i]);
    ballonPrint("  Value: ");
    ballonPrint(dataVector[i]);

    if (keyVector[i].startsWith("CHK")){
      ballonPrintln(" mA");
    }
    else if (keyVector[i].startsWith("VHK")){
      ballonPrintln(" V");
    }
    else if (keyVector[i].startsWith("Pressure")){
      if (dataVector[i] <= 41){
        low_pressure = true;
      }
      else{
        low_pressure = false;
      }
      ballonPrintln(" kPa");
    }
    else if (keyVector[i].startsWith("Altitude")){
      ballonPrintln(" m");
    }
    else if (keyVector[i].startsWith("Velocity")){
      ballonPrintln(" m/s");
    }
    else{
      ballonPrintln(" ºC");
    }
  }

  ballonPrintln("");
  ballonPrintln("---------------------------");
  
  dataVector.clear();
  dataVector.shrink_to_fit();
  keyVector.clear();
  keyVector.shrink_to_fit();  // dont really need this, at the end of the function the vector is cleared

  
  return;
}

void sendLCLDataFileToServer() {
  File dataFile = SD.open("/LCLDataFile.txt", FILE_READ); 

  if (!dataFile) {
    File log = SD.open("/log.txt", FILE_APPEND);

    log.printf("%d: sendLCLDataFileToServer() - Failed to open LCLDataFile.txt for reading. \n",millis());

    log.close();

    Serial.println("Failed to open LCLDataFile.txt for reading.");
    return;
  }

  if (client.connect(server, 8080)) {
    ballonPrint("Server connection successful: ");
    ballonPrintln(client.remoteIP());

    client.print("{'sensor_id':'TH01','values':[LCLDataFile,\\n,");
    
    while (dataFile.available()) {
      String data_line = dataFile.readStringUntil('\n');
      //ballonPrintln(data_line); 
      client.print(data_line);  
      client.print(",\\n,");      
    }

    
    client.print("]}");

    dataFile.close();
    ballonPrintln("LCLDataFile successfully sent to server.");

    File log = SD.open("/log.txt", FILE_APPEND);

    log.printf("%d: sendLCLDataFileToServer() - LCLDataFile successfully sent to server. \n",millis());

    log.close();

  }
  else {
    File log = SD.open("/log.txt", FILE_APPEND);

    log.printf("%d: sendLCLDataFileToServer() - Connection to Server Failed: Sending LCLDataFile. \n",millis());

    log.close();

    ballonPrintln("Connection to Server Failed: Sending LCLDataFile.");
  }

  client.stop();
}

void sendLCLTriggerFileToServer() {
  File dataFile = SD.open("/LCLTrigger.txt", FILE_READ); 

  if (!dataFile) {
    Serial.println("Failed to open LCLTrigger.txt for reading.");

    File log = SD.open("/log.txt", FILE_APPEND);

    log.printf("%d: sendLCLTriggerFileToServer() - Failed to open LCLTrigger.txt for reading. \n",millis());

    log.close();

    return;
  }

  if (client.connect(server, 8080)) {
    ballonPrint("Server connection successful: ");
    ballonPrintln(client.remoteIP());

    client.print("{'sensor_id':'TH01','values':[LCLTrigger,\\n,");
    
    while (dataFile.available()) {
      String data_line = dataFile.readStringUntil('\n');
      //ballonPrintln(data_line); 
      client.print(data_line);  
      client.print(",\\n,");      
    }

    
    client.print("]}");

    dataFile.close();
    ballonPrintln("LCLTrigger successfully sent to server.");

    File log = SD.open("/log.txt", FILE_APPEND);

    log.printf("%d: sendLCLTriggerFileToServer() - LCLTrigger successfully sent to server. \n",millis());

    log.close();

  }
  else {
    ballonPrintln("Connection to Server Failed while sending LCLTrigger.");

    File log = SD.open("/log.txt", FILE_APPEND);

    log.printf("%d: sendLCLTriggerFileToServer() - Connection to Server Failed while sending LCLTrigger. \n",millis());

    log.close();
  }

  client.stop();
}

//##############################################################################



//================= setup ===========
void setup() {
  Serial.begin(115200);
  
 
  esp32_initialsetup();       // configure ESP32 pins


  SPI.begin();

  
  ballonPrintln("\n Performing Initial Setup...");


  DUMMY_initialsetup();       // configure Dummy board GPIO ic

  PDU_initialsetup();         // configure PDU board GPIO ic
  
  SD_initialsetup();          // configure SD card
  
  ETH_initialsetup();         // configure ETH 
  

  // Add tasks to the scheduler
  runner.addTask(taskReadSensor);
  //runner.addTask(taskSendData);

  // Start tasks
  taskReadSensor.enable();
  //taskSendData.enable();

  spiMutex = xSemaphoreCreateMutex();
  if (spiMutex == NULL) {
    Serial.println("Failed to create spiMutex.");
    while (true);  // Halt execution if the mutex can't be created
  }


  if (esp32_initialsetup_check == true && DUMMY_initialsetup_check == true &&
    PDU_initialsetup_check == true && SD_initialsetup_check == true &&
    ETH_initialsetup_check == true){

    digitalWrite(LED, HIGH);
    ballonPrintln("Initial Setup done.");

    File log = SD.open("/log.txt", FILE_APPEND);

    log.printf("%d: Initial Setup Sucessfull \n",millis());

    log.close();

  }
  else{
    ballonPrintln("Initial Setup failed:");
    ballonPrint("   esp32_initialsetup_check: ");
    ballonPrintln(esp32_initialsetup_check);
    ballonPrint("   DUMMY_initialsetup_check: ");
    ballonPrintln(DUMMY_initialsetup_check);
    ballonPrint("   PDU_initialsetup_check: ");
    ballonPrintln(PDU_initialsetup_check);
    ballonPrint("   SD_initialsetup_check: ");
    ballonPrintln(SD_initialsetup_check);
    ballonPrint("   ETH_initialsetup_check: ");
    ballonPrintln(ETH_initialsetup_check);
    while(1){
      digitalWrite(LED, HIGH);
      delay(1000);
      digitalWrite(LED, LOW);
      delay(1000);
    }
  }


  // set the PDU on Power Ready mode, PDU is ready for operations
  String operation_mode = "power ready";
  changeOPmode(operation_mode);

  ballonPrint("[Operation Mode] ");
  ballonPrintln(operation_mode);

  File log = SD.open("/log.txt", FILE_APPEND);

  log.printf("%d: [Operation Mode] - %s \n",millis(), operation_mode);

  log.close();
   
   //external light signal to turn ON PDU switch
  
  int startTime = millis();

}


//============MAIN LOOP============
void loop() {
  //int waitTime_dummy = 10000; // 
  //int waitTime = 15000;
  

  runner.execute();
  //ballonPrintln("i am here");
  //digitalWrite(LED, LOW);

  //if (millis() >  20000){   //comment this for flight
  //  low_pressure = true;
  //}

  /*

  while(!Serial.available());

    String command_1 = Serial.readStringUntil('\n');

  sendDataToServer();

  ballonPrint("DONT CLICK ENTER");

  while(!Serial.available());

    String command_2 = Serial.readStringUntil('\n');
  */
  
  


  if (low_pressure == true){
    ballonPrintln("THE PRESSURE IS BELLOW 40kPa!!!!!!!!!!!!!!!!!!!!");
    
    if (log_lowpressure == false){
      log_lowpressure = true;
      File log = SD.open("/log.txt", FILE_APPEND);

      log.printf("%d: The Pressure is bellow 40kPa \n",millis());

      log.close();
    }
    float pressure_check = SPI_read_Pressure_Data("Pressure");

    if (low_pressure_firstTime == true && pressure_check < 41){  //change to this for Flight
    //if (low_pressure_firstTime == true) {
      start_opMode_tests = true;
      low_pressure_firstTime = false;
      timeNow_pressure7km = millis();
      tenK_wait_time = tenKmTestTrigger(); // for Flight
      //tenK_wait_time = 20000; // for Testing
    }
  }

  // To activate the first OP mode, Full Load
  if (millis() - timeNow_pressure7km > tenK_wait_time  && start_opMode_tests == true){
    ballonPrintln("TRYING TO CHANGE MODE!!");
    
    for (int i = 0 ; i < listOPmodes_toTest.size() ; i++){

      if (listOPmodes_toTest[i].equals("full load")){
        endOfTests = true;
      }

      int time_init_OPmode = millis();
      ballonPrint("Change Operation mode: ");
      ballonPrintln(listOPmodes_toTest[i]);
      changeOPmode(listOPmodes_toTest[i]);

      File log = SD.open("/log.txt", FILE_APPEND);

      log.printf("%d: [Operation Mode] - %s \n",millis(), listOPmodes_toTest[i]);

      log.close();

      delay(500);
      
      if (listOPmodes_toTest[i].equals("gam")){
        check_setDummyLoad = setDummyLoad("DAC2", 10, true);  // dac2 is for GAM

        if (check_setDummyLoad == true){
          File log = SD.open("/log.txt", FILE_APPEND);

          log.printf("%d: setDummyLoad() - Changed to [%s] operation mode, output: [DAC2] \n",millis(), listOPmodes_toTest[i]);

          log.close();

          check_setDummyLoad = false;
        }
        else{
          File log = SD.open("/log.txt", FILE_APPEND);

          log.printf("%d: setDummyLoad() - Error occured on Operation mode [%s]. Either I_load out of range (0-2400mA) or DAC didnt turn OFF, output: [DAC2] \n", millis(),listOPmodes_toTest[i]);

          log.close();

          check_setDummyLoad = false;
        }

      }
      else{
      check_setDummyLoad = setDummyLoad("DAC2", 10, true);  // dac2 is for GAM

      if (check_setDummyLoad == true){
        File log = SD.open("/log.txt", FILE_APPEND);

        log.printf("%d: setDummyLoad() - Changed to [%s] operation mode, output: [DAC2] \n",millis(), listOPmodes_toTest[i]);

        log.close();

        check_setDummyLoad = false;
        }
      else{
        File log = SD.open("/log.txt", FILE_APPEND);

        log.printf("%d: setDummyLoad() - Error occured on Operation mode [%s]. Either I_load out of range (0-2400mA) or DAC didnt turn OFF, output: [DAC2] \n", millis(),listOPmodes_toTest[i]);

        log.close();

        check_setDummyLoad = false;
      }



      delay(500);

      check_setDummyLoad = setDummyLoad("DAC1", 10, true);

      if (check_setDummyLoad == true){
        File log = SD.open("/log.txt", FILE_APPEND);

        log.printf("%d: setDummyLoad() - Changed to [%s] operation mode, output: [DAC1] \n",millis(), listOPmodes_toTest[i]);

        log.close();

        check_setDummyLoad = false;
        }
      else{
        File log = SD.open("/log.txt", FILE_APPEND);

        log.printf("%d: setDummyLoad() - Error occured on Operation mode [%s]. Either I_load out of range (0-2400mA) or DAC didnt turn OFF, output: [DAC1] \n", millis(),listOPmodes_toTest[i]);

        log.close();

        check_setDummyLoad = false;
      }

      }
      
      while (millis() - time_init_OPmode < op_mode_waitTime){
        runner.execute();
        delay(100);
      }

      
      String nameLCL = "VS_GAM.1";
      int aqFrequency = 1000; // Hz
      int runTime = 1000; // ms

      testLCLParameters params = {nameLCL, aqFrequency, runTime};

      xTaskCreatePinnedToCore(
        testLCLreadSensors,  // function
        "LCL Task",          // name of the task
        10000,               // stack size in words (idk)
        &params,             // Task input parameters, from TaskParameters struct testLCLParameters
        1,                   // priority
        NULL,                // Pointer to store task handle (idk if i need)
        0);                  // core ID
 
      xTaskCreatePinnedToCore(
        testLCLtrigger,
        "LCL Trigger",
        10000,
        &params,
        1,
        NULL,
        1);

      runner.execute();
      delay(10000); // CHENGE THIS SHIT
      setDummyLoad(nameLCL, 10, false);

      
      params.nameLCL = "VS_OBC";

      xTaskCreatePinnedToCore(
        testLCLreadSensors,  // function
        "LCL Task",          // name of the task
        10000,               // stack size in words (idk)
        &params,             // Task input parameters, from TaskParameters struct testLCLParameters
        1,                   // priority
        NULL,                // Pointer to store task handle (idk if i need)
        0);                  // core ID
 
      xTaskCreatePinnedToCore(
        testLCLtrigger,
        "LCL Trigger",
        10000,
        &params,
        1,
        NULL,
        1);

      delay(10000); // CHENGE THIS SHIT
      setDummyLoad(nameLCL, 10, false);

      sendLCLDataFileToServer();
      sendLCLTriggerFileToServer();
    }
  }

  
  if (endOfTests == true){
    while(1){
    // take HK aquisition ultil end of flight
      if (endOfTests_changeMode == false){
        changeOPmode("Power Save");
        endOfTests_changeMode = true;
      }
      runner.execute();
    }
  }

  delay(100);


}


//=======================================================================================================
//=======================================================================================================

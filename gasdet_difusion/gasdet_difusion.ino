#include <SPI.h>

#define _WRITE 0x40
#define _READ 0x41

//#define _WRITE_u1 0x48
//#define _READ_u1 0x49

#define SPI_MOSI 11
#define SPI_MISO 12
#define SPI_SLK 13
#define CS_U1 10
#define CS_U5 9

#define SPI_STANDARD_SETTINGS SPISettings(10000,MSBFIRST,SPI_MODE0)


#define DEBUG 1

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

String U1 = "u1";
String U5 = "u5";

int stripUsing = 0;

String selectedIC = "u5";
int selectedIC_CS = CS_U5; //starts with the U5

const byte stripBitPosition[22] = {
  0,  // index 0 unused
  1,  // strip 1 → bit 1
  0,  // strip 2 → bit 0
  3,  // strip 3 → bit 3
  2,  // strip 4 → bit 2
  5,  // strip 5 → bit 5
  4,  // strip 6 → bit 4
  7,  // strip 7 → bit 7
  6,  // strip 8 → bit 6
  1,  // strip 9 → bit 1
  0,  // strip 10 → bit 0
  3,  // strip 11 → bit 3
  2,  // strip 12 → bit 2
  5,  // strip 13 → bit 5
  4,  // strip 14 → bit 4
  7,  // strip 15 → bit 7
  6,  // strip 16 → bit 6
  1,  // strip 17 → bit 1
  0,  // strip 18 → bit 0
  3,  // strip 19 → bit 3
  2,  // strip 20 → bit 2
  5   // strip 21 → bit 5
};


void set_selected_IC(String selected_ic){
  if (selected_ic.equals("u1")){
    selectedIC = U1;
    selectedIC_CS = CS_U1;
    debugPrintln("Changed communication U1");
  }
  else if (selected_ic.equals("u5")){
    selectedIC = U5;
    selectedIC_CS = CS_U5;
    debugPrintln("Changed communication U5");
  }
}


void arduino_initialsetup(){

  pinMode(CS_U1, OUTPUT);
  digitalWrite(CS_U1, HIGH);

  pinMode(CS_U5, OUTPUT);
  digitalWrite(CS_U5, HIGH);

  set_selected_IC("u5");
  }

int getCommandNumber (String command) {
  command.toLowerCase();

  if (command.equals("initial setup")) {
    return 0;
  }
  else if (command.equals("manual op")) {
    return 1;
  }
  else if (command.equals("aquisition")) {
    return 3;
  }
  else if (command.startsWith("p")){
    int num = command.substring(1).toInt();
    if (num >= 1 && num <= 21){
      return 300 + num;
    }
  }
  else if (command.equals("all on")) {
    return 4;
  }
  


  else if (command.equals("change ic")) {
    return 101;
  }
  else if (command.equals("read miso")) {
    return 102;
  }
  else if (command.equals("write register")){ 
    return 103;
  }
  else if (command.equals("set op mode")) {
    return 2;
  }

  return -1;
}



//###################         [SPI] general functions        ###################
void SPI_write(int CS_pin, byte register_address, byte _data) {
  SPI.beginTransaction(SPI_STANDARD_SETTINGS);

  digitalWrite(CS_pin, LOW);
  byte byte1 = SPI.transfer(_WRITE);
  byte byte2 = SPI.transfer(register_address);
  byte byte3 = SPI.transfer(_data);
  digitalWrite(CS_pin, HIGH);

  SPI.endTransaction();
}

byte SPI_read(int CS_pin, byte register_address) {
  SPI.beginTransaction(SPI_STANDARD_SETTINGS);

  
  digitalWrite(CS_pin, LOW);
  byte byte1 = SPI.transfer(_READ);
  byte byte2 = SPI.transfer(register_address);
  byte byte3 = SPI.transfer(0x00);
  digitalWrite(CS_pin, HIGH);
  
  
  return byte3;

  SPI.endTransaction();
}
//##############################################################################


void diffusion_initialsetup(String IC){
  SPI.beginTransaction(SPI_STANDARD_SETTINGS);

  set_selected_IC(IC);
  SPI_write(selectedIC_CS, 0x05, 0xfe); 
  SPI_write(selectedIC_CS, 0x15, 0xfe);   
    
  SPI_write(selectedIC_CS, 0x00, 0x00);
  SPI_write(selectedIC_CS, 0x10, 0x00);
        
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
  SPI_write(selectedIC_CS, 0x1a, 0xff);

  SPI_write(selectedIC_CS, 0x1a, 0xff);
  SPI_write(selectedIC_CS, 0x19, 0xff);
  SPI_write(selectedIC_CS, 0x0a, 0xff);
  SPI_write(selectedIC_CS, 0x09, 0xff); 

  delay(1000);

  SPI_write(selectedIC_CS, 0x0a, 0x00);
  SPI_write(selectedIC_CS, 0x1a, 0x00);

  SPI_write(selectedIC_CS, 0x1a, 0x00);
  SPI_write(selectedIC_CS, 0x19, 0x00);
  SPI_write(selectedIC_CS, 0x0a, 0x00);
  SPI_write(selectedIC_CS, 0x09, 0x00); 

  SPI.endTransaction();

  debugPrint("   ");
  debugPrint(IC);
  debugPrintln(" initial setup: done...");
  
}


void changeOPmode(String mode){

  SPI.beginTransaction(SPI_STANDARD_SETTINGS);
  mode.toLowerCase();

  if (mode.equals("all off")){
    set_selected_IC(U1);
    SPI_write(selectedIC_CS, 0x09, 0x00);
    byte read_check = SPI_read(selectedIC_CS, 0x09);
  }

  else if (mode.equals("ch1")){
    set_selected_IC(U1);
    SPI_write(selectedIC_CS, 0x09, 0x01);
    byte read_check = SPI_read(selectedIC_CS, 0x09);
  }

  SPI.endTransaction();
}

byte translate_address(String command) {
  command.toLowerCase();
  if (command.length() != 4) {
    debugPrint("Invalid Register Address \"");
    debugPrint(command);
    debugPrintln("\". Please write it as \"0xAA\".");
    return -1;
  }
  else {
    int digit1, digit2;
      
    if (command[2] >= 97 && command[2] <= 103) {
      digit1 = (command[2] - 87)*16;
    }
    else {
      digit1 = (command[2] - 48)*16;
    }
    
    if (command[3] >= 97 && command[3] <= 103) {
      digit2 = (command[3] - 87);
    }
    else {
      digit2 = (command[3] - 48);
    }
     
    return digit1|digit2;
  }
}

byte translate_data(String command) {
  command.toLowerCase();
  byte _data = 0;
  for (int i=0, shift = 7; i<command.length(); i++, shift--) {
    if (command[i] == '1') {
      _data = _data | (1 << shift);
    }
    else {
      _data = _data | (0 << shift);
    }
  }
  
  //debugPrint("Data byte: ");
  //debugPrintln(_data, BIN);
  
  return _data;
}

void setup() {
  Serial.begin(115200);

  SPI.begin();

  arduino_initialsetup();
  diffusion_initialsetup("u1");
  diffusion_initialsetup("u5");

  debugPrintln("[DONE] Initial Setup");
}

void loop() {

  debugPrintln("=================================================================");
  debugPrint("Selected IC: ");
  debugPrintln(selectedIC);
  debugPrintln("=================================================================\n");
  
  debugPrintln("Available Commands:");
  debugPrintln("\"initial setup\", \"manual op \", \" set op mode\",\" aquisition\"");
  // Wait for command
  debugPrint("Input command: ");
  while (!Serial.available());

  String command = Serial.readStringUntil('\n');  
  debugPrintln(command);
  command.toLowerCase();

  int command_n = getCommandNumber(command);  

  byte register_address;

  // Perform Initial Setup
  if (command_n == 0) {

    debugPrintln("Performing Initial Setup...");
    arduino_initialsetup();
    diffusion_initialsetup("u1");
    diffusion_initialsetup("u5");
    debugPrintln("[DONE] Initial Setup");

  }
  // Read Alert Pin
  else if (command_n == 1) {
    debugPrintln("Available Commands: ");
    debugPrintln("\"Change Ic\",\"Read Miso\",\"Write Register\"");
    debugPrint("Command to use: ");
    while(!Serial.available());

    String command_1 = Serial.readStringUntil('\n');
    debugPrintln(command_1);
    command_1.toLowerCase();

    //debugPrint("debug command_1: ");
    //debugPrintln(command_1);

    int command_1_n = getCommandNumber(command_1);

    //debugPrint("debug command_1_n: ");
    //debugPrintln(command_1_n);

    // Change IC
    if (command_1_n == 101){
      debugPrintln("Chose the IC:");
      debugPrintln("U1");
      debugPrintln("U5");
      debugPrintln("");
      while(!Serial.available());
      command = Serial.readStringUntil('\n');
      command.toLowerCase();

      set_selected_IC(command);

    }

    // Read MISO
    else if(command_1_n == 102){

      debugPrintln("Which register to read? 0xAA");
      while (!Serial.available());
      command = Serial.readStringUntil('\n');

      debugPrint("Reading Register ");
      debugPrintln(command);
      // Convert address into number
      register_address = translate_address(command);

      debugPrint("CS to use: ");
      debugPrintln(selectedIC_CS);
    
      SPI_read(selectedIC_CS, register_address);
    }

    // Write Register
    else if(command_1_n == 103){

      debugPrint("Which register to write? [0x0AA] -> ");
      while(!Serial.available());
      command = Serial.readStringUntil('\n');
      debugPrintln(command);
    
      register_address = translate_address(command);

      debugPrint("Byte to send? [bbbbbbbb] ->");
      while(!Serial.available());
      command = Serial.readStringUntil('\n');
      debugPrintln(command);
      byte _data = translate_data(command);

      SPI_write(selectedIC_CS, register_address, _data);
    }
  }
  else if (command_n == 2) {
    debugPrintln("Not yet implemented");
  }
  
  else if (command_n == 3) {
    debugPrintln("Enter strip to read (format: pXX), or type \"done\" to return:");

    while (true) {
      debugPrintln("Which strip do you want to read? (format: pXX)");
      while(!Serial.available());
      String command_3 = Serial.readStringUntil('\n');
      debugPrintln(command_3);
      command_3.toLowerCase();

      if (command_3.equals("done")) {
        if (stripUsing >= 1 && stripUsing <= 8){
          set_selected_IC("u1");
          SPI_write(selectedIC_CS, translate_address("0x09"), 0x00);
        }
        else if (stripUsing >= 9 && stripUsing <= 16){
          set_selected_IC("u1");
          SPI_write(selectedIC_CS, translate_address("0x19"), 0x00);
        }
        else if (stripUsing >= 17 && stripUsing <= 21){
          set_selected_IC("u5");
          SPI_write(selectedIC_CS, translate_address("0x09"), 0x00);
        }
        stripUsing = 0;
        debugPrintln("Returning to main menu...");
        break;
      }

      if (command_3.length()== 3 && command_3.charAt(0) == 'p'){
        int strip = command_3.substring(1).toInt();

        if (strip < 1 || strip > 21) {
          debugPrintln("Invalid strip number. Please enter between p01 and p21.");
          continue;
        }

        if (stripUsing >= 1 && stripUsing <= 8){
          SPI_write(selectedIC_CS, translate_address("0x09"), 0x00);
        }

        else if (stripUsing >= 9 && stripUsing <= 16){
          SPI_write(selectedIC_CS, translate_address("0x19"), 0x00);
        }

        else if (stripUsing >= 17 && stripUsing <= 21){
          SPI_write(selectedIC_CS, translate_address("0x09"), 0x00);
        }

        byte _data = 1 << stripBitPosition[strip];
        stripUsing = strip;

        if (strip >= 1 && strip <= 8){
          set_selected_IC("u1");
          SPI_write(selectedIC_CS, translate_address("0x09"), _data);
        }

        else if (strip >= 9 && strip <= 16){
          set_selected_IC("u1");
          SPI_write(selectedIC_CS, translate_address("0x19"), _data);
        }

        else{
          set_selected_IC("u5");
          SPI_write(selectedIC_CS, translate_address("0x09"), _data);
        }
      }
      else {
        debugPrintln("Invalid input. Use format pXX or type \"done\".");
      }
    }
  }
  else if (command_n == 4) {
    set_selected_IC(U1);
    SPI_write(selectedIC_CS, translate_address("0x09"), 0xff);
    SPI_write(selectedIC_CS, translate_address("0x19"), 0xff);
    set_selected_IC(U5);
    SPI_write(selectedIC_CS, translate_address("0x09"), translate_address("0x2f"));
  }
}

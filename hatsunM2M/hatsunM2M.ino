/* Author : Vikas Gaikwad
 * Date : 10 Nov. 2016
 * 
 *  Description : Read all the Data from MODBUS Slaves.
   Hardware Connections : Arduino MEGA SERIAL2(Hardware Serial) 
                          RX2 -----> R0 (MAX485 Chip Adapter)
                          TX2 -----> DI (MAX485 Chip Adapter)
                          D2  -----> DE/RE (MAX485 Chip Adapter)
   MODBUS  Device Settings : 

   SLAVE 1 : Milk Chiller Panel 
   Slave ID : 1, Baud Rate : 9600, Parity : NONE, Stop bit : 1    

   SLAVE 2 : Energy Meter
   Slave ID : 2, Baud Rate: 9600, Parity :None, Stop bit : 1
 * 
 * MODBUS Electrical Connections: 
 * 1> MILK Chiller Serial(RS 232) --> 485 Converter Pin 1 --> A of MAX485 Chip Adapter.
 *                                    485 Converter Pin 2 --> B of MAX485 Chip Adapter.        
 * 2> Energy Meter (RS485 Signal) --> A (Green Wire) -->  B of MAX485 Chip Adapter. 
 *                                    B (Yellow Wire) --> A of MAX485 Chip Adapter.
 *   
 *   
 */


#include <SimpleModbusMaster.h>
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include "RTClib.h"
#include "sim900.h"     
#include "serverConfg.h"  
#include "sdcardConfig.h"        

String getlogTime();


sim900_GPRS myGateway;    // Gateway object
RTC_DS3231 rtc;           // RTC instance



#define baud 9600
#define timeout 1000
#define polling 200
#define retry_count 20

#define TxEnablePin 2 

#define LED 13
const uint8_t ChipSelect = 53 ;   //SD Card ChipSelect pin. Dont Change


#define TOTAL_NO_OF_REGISTERS 81        // prev.cnt = 33 Control Panel Resisters = 17 , Energy Meter Resisters = 16

enum
{
  PACKET1,
  
  PACKET2,    // CMC Energy meter
  PACKET3,
  PACKET4,
  PACKET5,
  PACKET6,
  PACKET7,
  PACKET8,
  PACKET9,
  
  PACKET10,    // Gyser Energy Meter
  PACKET11,
  PACKET12,
  PACKET13,
  PACKET14,
  PACKET15,
  PACKET16,
  PACKET17,
  
  PACKET18, // Bore Pump 
  PACKET19,
  PACKET20,
  PACKET21,
  PACKET22,
  PACKET23,
  PACKET24,
  PACKET25,

  PACKET26,     // 3 Phase Energy Meter 
  PACKET27,
  PACKET28,
  PACKET29,
  PACKET30,
  PACKET31,
  PACKET32,
  PACKET33,
  
  TOTAL_NO_OF_PACKETS // leave this last entry
};

Packet packets[TOTAL_NO_OF_PACKETS];

int regs[TOTAL_NO_OF_REGISTERS];     // All the Data from Modbus Resisters gets stored here in this buffer.

void setup()
{
  pinMode(ChipSelect,OUTPUT);
  Serial.begin(9600);
  Serial2.begin(9600);
  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (1);
  }

  if (rtc.lostPower()) {
    Serial.println("RTC lost power, lets set the time!");
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  }
  
   modbus_construct(&packets[PACKET1], 1, READ_HOLDING_REGISTERS, 4, 17, 0);  // Control Panel Resisters [40005 - 40021]
  // modbus_construct(&packets[PACKET10], 1, READ_HOLDING_REGISTERS, 21,11,17 );  // Control Panel Resisters [400022 - 40032] Faults 
   // ENERGY METER PACKET
  
  // Slave 2 Energy Meter [CMC]
    
    modbus_construct(&packets[PACKET2], 2,READ_INPUT_REGISTERS , 0, 2, 17);    //30001-2 Line Voltage reg[17-18]       HB :17278 LB :44628
    modbus_construct(&packets[PACKET3], 2,READ_INPUT_REGISTERS , 6, 2, 19);    // 30007-8 Line Current reg[19-20]      
    modbus_construct(&packets[PACKET4], 2,READ_INPUT_REGISTERS , 42,2, 21);    // 30043-44 Device voltage reg[21-22]
    modbus_construct(&packets[PACKET5], 2,READ_INPUT_REGISTERS , 46,2, 23);    // 30047-48 Device Current reg[23-24]
    modbus_construct(&packets[PACKET6], 2,READ_INPUT_REGISTERS , 72,2, 25);    // 30073-74 Power Consumption  reg[25-26]   Reading 16384(73)[IMP] History Value
 
    modbus_construct(&packets[PACKET7], 2,READ_INPUT_REGISTERS , 84,2, 27);    // 30089-90 Sys Power(W) reg[27-28]  // 84 Live Value of Watts
    modbus_construct(&packets[PACKET8], 2,READ_INPUT_REGISTERS , 226,2,29);    // 30227-28 Device Run Hr reg[29-30]     
    modbus_construct(&packets[PACKET9], 2,READ_INPUT_REGISTERS , 228,2,31);    // 30227-28 Power Available Time reg[31-32]    

// Slave 3 Energy Meter [ Gyser]
  
    modbus_construct(&packets[PACKET10], 3,READ_INPUT_REGISTERS , 0, 2, 33);    //30001-2 Line Voltage reg[17-18]       HB :17278 LB :44628
    modbus_construct(&packets[PACKET11], 3,READ_INPUT_REGISTERS , 6, 2, 35);    // 30007-8 Line Current reg[19-20]      
    modbus_construct(&packets[PACKET12], 3,READ_INPUT_REGISTERS , 42,2, 37);    // 30043-44 Device voltage reg[21-22]
    modbus_construct(&packets[PACKET13], 3,READ_INPUT_REGISTERS , 46,2, 39);    // 30047-48 Device Current reg[23-24]
    modbus_construct(&packets[PACKET14], 3,READ_INPUT_REGISTERS , 72,2, 41);    // 30073-74 Power Consumption  reg[25-26]   Reading 16384(73)[IMP] History Value
 
    modbus_construct(&packets[PACKET15], 3,READ_INPUT_REGISTERS , 50,2, 43);    // 30089-90 Sys Power(W) reg[27-28]  //86 --> 50 Live Value
    modbus_construct(&packets[PACKET16], 3,READ_INPUT_REGISTERS , 226,2,45);    // 30227-28 Device Run Hr reg[29-30]     
    modbus_construct(&packets[PACKET17], 3,READ_INPUT_REGISTERS , 228,2,47);    // 30227-28 Power Available Time reg[31-32]    

// Slave 4 Energy Meter [ Bore Pump]
  
    modbus_construct(&packets[PACKET18], 4,READ_INPUT_REGISTERS , 0, 2, 49);    //30001-2 Line Voltage reg[17-18]       HB :17278 LB :44628
    modbus_construct(&packets[PACKET19], 4,READ_INPUT_REGISTERS , 6, 2, 51);    // 30007-8 Line Current reg[19-20]      
    modbus_construct(&packets[PACKET20], 4,READ_INPUT_REGISTERS , 42,2, 53);    // 30043-44 Device voltage reg[21-22]
    modbus_construct(&packets[PACKET21], 4,READ_INPUT_REGISTERS , 46,2, 55);    // 30047-48 Device Current reg[23-24]
    modbus_construct(&packets[PACKET22], 4,READ_INPUT_REGISTERS , 72,2, 57);    // 30073-74 Power Consumption  reg[25-26]   Reading 16384(73)[IMP] History Value
 
    modbus_construct(&packets[PACKET23], 4,READ_INPUT_REGISTERS , 50,2, 59);    // 30089-90 Sys Power(W) reg[27-28]  //86 --> 50 Live Value
    modbus_construct(&packets[PACKET24], 4,READ_INPUT_REGISTERS , 226,2,61);    // 30227-28 Device Run Hr reg[29-30]     
    modbus_construct(&packets[PACKET25], 4,READ_INPUT_REGISTERS , 228,2,63);    // 30227-28 Power Available Time reg[31-32]    
    
// Slave 5 3 Phase Energy Meter [Total Conumptions]
  
    modbus_construct(&packets[PACKET26], 5,READ_INPUT_REGISTERS , 0, 2, 65);    //30001-2 Line Voltage reg[17-18]       HB :17278 LB :44628
    modbus_construct(&packets[PACKET27], 5,READ_INPUT_REGISTERS , 6, 2, 67);    // 30007-8 Line Current reg[19-20]      
    modbus_construct(&packets[PACKET28], 5,READ_INPUT_REGISTERS , 42,2, 69);    // 30043-44 Device voltage reg[21-22]
    modbus_construct(&packets[PACKET29], 5,READ_INPUT_REGISTERS , 46,2, 71);    // 30047-48 Device Current reg[23-24]
    modbus_construct(&packets[PACKET30], 5,READ_INPUT_REGISTERS , 72,2, 73);    // 30073-74 Power Consumption  reg[25-26]   Reading 16384(73)[IMP] History Value
 
    modbus_construct(&packets[PACKET31], 5,READ_INPUT_REGISTERS , 50,2, 75);    // 30089-90 Sys Power(W) reg[27-28]  //86 --> 50 Live Value
    modbus_construct(&packets[PACKET32], 5,READ_INPUT_REGISTERS , 226,2,77);    // 30227-28 Device Run Hr reg[29-30]     
    modbus_construct(&packets[PACKET33], 5,READ_INPUT_REGISTERS , 228,2,79);    // 30227-28 Power Available Time reg[31-32]    

   
  //modbus_construct(&packets[PACKET2], 1, PRESET_MULTIPLE_REGISTERS, 0, 10, 0);
  
  modbus_configure(&Serial2, baud, SERIAL_8N1, timeout, polling, retry_count, TxEnablePin, packets, TOTAL_NO_OF_PACKETS, regs);
  delay(100);
  //pinMode(LED, OUTPUT);

  // Setup GSM Module 
  Serial1.begin(9600);             // To connect SIM900A and send AT Commands
  //myGateway.power_on();           // POWER ON GSM Module for communication 
  //myGateway.httpInit();          //Initialize http connections (please change APN as per your Network Operator)
  
/************** SD Card Init/Startup Code ***********************/
 isSDCardCheck("chiller.csv");     //provide a File Name to Store log of ModBus Devices
 pinMode(A12,OUTPUT);
 digitalWrite(A12,LOW);
  
}

void loop()
{     
  DateTime now = rtc.now();
   //DateTime now = rtc.now();
  for(int i=0;i<400;i++)
  {
    modbus_update();
    delay(10); //Serial.println(i);
  }
//  if(now.second() == 30)
//  { 
//    Serial.println("RESET");
//    asm volatile ("  jmp 0"); 
//    //Reset the code
//  }

  digitalWrite(A12,HIGH);
 
  float battery_Temp = regs[0]/10.0;
  float milk_Temp = regs[1]/10.0;
  float auxillary_Temp = regs[2]/10.0;
  uint8_t battery_Volt = regs[3];
  uint8_t ac_Volt = regs[4];
  float compressor_Current = regs[5]/10.0;
  float pump_Current = regs[6]/10.0;
  boolean charg_pump_Relay = regs[7];
  boolean condensor_Relay = regs[8];
  boolean compressor_Relay = regs[9];
  boolean inverter_Relay = regs[10];
  boolean agitator_Relay = regs[11];
  boolean tank_Relay = regs[12];
  boolean shiva_Relay = regs[13];
  boolean discharge_pump_Relay = regs[14];
  uint32_t compressor_run_Hour = regs[16];
     
  Serial.print("Battery Temp.: ");Serial.println(battery_Temp,DEC);
  Serial.print("Milk Temp.: ");Serial.println(milk_Temp,DEC);
  Serial.print("Auxillary Temp.: ");Serial.println(auxillary_Temp,DEC);
  Serial.print("Battery Volt.: ");Serial.println(battery_Volt,DEC);
  Serial.print("AC Voltage.: ");Serial.println(ac_Volt,DEC);
  Serial.print("Copmressor Curr.: ");Serial.println(compressor_Current,DEC);
  Serial.print("Pump Current.: ");Serial.println(pump_Current,DEC);
  Serial.print("Ch Pump Relay.: ");Serial.println(charg_pump_Relay,DEC);
  Serial.print("Condensor Relay.: ");Serial.println(condensor_Relay,DEC);
  Serial.print("Compressor Relay.: ");Serial.println(compressor_Relay,DEC);
  Serial.print("Inverter Relay.: ");Serial.println(inverter_Relay,DEC);
  Serial.print("Agitator Relay.: ");Serial.println(agitator_Relay,DEC);
  Serial.print("Tank Relay.: ");Serial.println(tank_Relay,DEC);
  Serial.print("Shiva Relay.: ");Serial.println(shiva_Relay,DEC);
  //Serial.print("Discharge Pump Relay.: ");Serial.println(regs[14],DEC);
  Serial.print("Discharge Pump Relay.: ");Serial.println(discharge_pump_Relay,DEC);
  Serial.print("Compressor Run Hr.: ");Serial.println(compressor_run_Hour,DEC);
  delay(20);
 
    // Energy Meter Readings 
  Serial.println("************ Energy Meter Readings ************* ");
 /* 
  Serial.print("Line Voltages[HB].: ");Serial.println(regs[17],HEX);
  Serial.print("Line Voltages[LB].: ");Serial.println(regs[18],HEX);
  Serial.print("Line Current [HB].: ");Serial.println(regs[19],HEX);
  Serial.print("Line Current [LB].: ");Serial.println(regs[20],HEX);
  Serial.print("Device Voltage [HB].: ");Serial.println(regs[21],HEX);
  Serial.print("DEvice Voltage [LB].: ");Serial.println(regs[22],HEX);
  Serial.print("Device Current [HB].: ");Serial.println(regs[23],HEX);
  Serial.print("DEvice Current [LB].: ");Serial.println(regs[24],HEX);
  Serial.print("POWER[Sys W] [HB].: ");Serial.println(regs[25],HEX);
  Serial.print("POWER[Sys W] [LB].: ");Serial.println(regs[26],HEX);
  Serial.print("Power Consumption [HB].: ");Serial.println(regs[27],HEX);
  Serial.print("Power Consumption [LB].: ");Serial.println(regs[28],HEX);
  Serial.print("Device Run Hr [HB].: ");Serial.println(regs[29],HEX);
  Serial.print("DEvice Run Hr [LB].: ");Serial.println(regs[30],HEX);
  Serial.print("Power Available Time [HB].: ");Serial.println(regs[31],HEX);
  Serial.print("Power Available Time [LB].: ");Serial.println(regs[32],HEX);
 */ 
 
  delay(50);
  float em2_lineVolts = myGateway.hextofloat(regs[17],regs[18]);
  float em2_lineCurrent = myGateway.hextofloat(regs[19],regs[20]);
  float em2_deviceVolts = myGateway.hextofloat(regs[21],regs[22]);
  float em2_deviceCurrent = myGateway.hextofloat(regs[23],regs[24]);
  float em2_power = myGateway.hextofloat(regs[25],regs[26]);
  float em2_powerConsump = myGateway.hextofloat(regs[27],regs[28]);
  float em2_deviceRunHr = myGateway.hextofloat(regs[29],regs[30]);
  float em2_powerAvailableTime = myGateway.hextofloat(regs[31],regs[32]);
 
  //float energy_meter[] = { lineVolts,lineCurrent,deviceVolts,deviceCurrent,power,powerConsump,deviceRunHr,powerAvailable };

  Serial.print("EM2_line Volts:");Serial.println(em2_lineVolts);
  Serial.print("EM2_line Current:");Serial.println(em2_lineCurrent);
  Serial.print("EM2_device Volts:");Serial.println(em2_deviceVolts);
  Serial.print("EM2_device Current:");Serial.println(em2_deviceCurrent);
  Serial.print("EM2_Power:");Serial.println(em2_power);
  Serial.print("EM2_Power Consump:");Serial.println(em2_powerConsump);
  Serial.print("EM2_Device Run Hr:");Serial.println(em2_deviceRunHr);
  Serial.print("EM2_powerAvailableTime:");Serial.println(em2_powerAvailableTime);

// Energy Meter 3 
  Serial.println("***********************************************************************");
  
  float em3_lineVolts = myGateway.hextofloat(regs[33 ],regs[34]);
  float em3_lineCurrent = myGateway.hextofloat(regs[35],regs[36]);
  float em3_deviceVolts = myGateway.hextofloat(regs[37],regs[38]);
  float em3_deviceCurrent = myGateway.hextofloat(regs[39],regs[40]);
  float em3_power = myGateway.hextofloat(regs[41],regs[42]);
  float em3_powerConsump = myGateway.hextofloat(regs[43],regs[44]);
  float em3_deviceRunHr = myGateway.hextofloat(regs[45],regs[46]);
  float em3_powerAvailableTime = myGateway.hextofloat(regs[47],regs[48]);
 
  Serial.print("EM3_line Volts:");Serial.println(em3_lineVolts);
  Serial.print("EM3_line Current:");Serial.println(em3_lineCurrent);
  Serial.print("EM3_device Volts:");Serial.println(em3_deviceVolts);
  Serial.print("EM3_device Current:");Serial.println(em3_deviceCurrent);
  Serial.print("EM3_Power:");Serial.println(em3_power);
  Serial.print("EM3_Power Consump:");Serial.println(em3_powerConsump);
  Serial.print("EM3_Device Run Hr:");Serial.println(em3_deviceRunHr);
  Serial.print("EM3_powerAvailableTime:");Serial.println(em3_powerAvailableTime);
 
// Energy Meter 4 
  Serial.println("***********************************************************************");
  
  float em4_lineVolts = myGateway.hextofloat(regs[49 ],regs[50]);
  float em4_lineCurrent = myGateway.hextofloat(regs[51],regs[52]);
  float em4_deviceVolts = myGateway.hextofloat(regs[53],regs[54]);
  float em4_deviceCurrent = myGateway.hextofloat(regs[55],regs[56]);
  float em4_power = myGateway.hextofloat(regs[57],regs[58]);
  float em4_powerConsump = myGateway.hextofloat(regs[59],regs[60]);
  float em4_deviceRunHr = myGateway.hextofloat(regs[61],regs[62]);
  float em4_powerAvailableTime = myGateway.hextofloat(regs[63],regs[64]);
 
  Serial.print("EM4_line Volts:");Serial.println(em4_lineVolts);
  Serial.print("EM4_line Current:");Serial.println(em4_lineCurrent);
  Serial.print("EM4_device Volts:");Serial.println(em4_deviceVolts);
  Serial.print("EM4_device Current:");Serial.println(em4_deviceCurrent);
  Serial.print("EM4_Power:");Serial.println(em4_power);
  Serial.print("EM4_Power Consump:");Serial.println(em4_powerConsump);
  Serial.print("EM4_Device Run Hr:");Serial.println(em4_deviceRunHr);
  Serial.print("EM4_powerAvailableTime:");Serial.println(em4_powerAvailableTime);
 
// 
//  for(int i=17;i<65;i++)
//  {
//    Serial.println(regs[i]);
//  }
//  
  Serial.println("**** done ****");
  digitalWrite(A12,LOW);
  delay(200);
  
}

String getlogTime()
{   
   DateTime now = rtc.now();
   String logTime ; 
      
   logTime += now.day();
   logTime += "/";
   logTime += now.month();
   logTime += "/";
   logTime += now.year();
   logTime += "@";
   logTime += now.hour();
   logTime += ":";
   logTime += now.minute();
   logTime += ":";
   logTime += now.second();
   
   //Serial.print("Time:");Serial.println(logTime);
   //delay(2000);
   return logTime;
}



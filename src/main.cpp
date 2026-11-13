//Mazda RX-8 Electronic Power Steering Enabler

#include <Arduino.h>
#include <SPI.h>
#include "mcp_can.h"


//Pins:
MCP_CAN CAN0(10); //SPI Chip Select for MCP2515 
#define CAN0_INT 2 //Interrupt from MCP2515 


// The EPS Module requires the following messages with IDs to be sent:

//0x81, 0x201, 0x212 (eps receives errors)

// The EPS outputs 0x300 to the cluster for feedback. 0x00: normal op. 0x80: failure
// Bits are ordered from LSB (1-indexed).

/*
0x81    (4B): Steering angle       D0: ?   D1: 0xEF if centered, 0x00 if not.   D2-D3 : actual angle (0xFDE (LEFT) to 0x021E (Right))

0x201   (8B): RPM and Speed        D0-D1 : RPM (actual RPM * 3.85)                 D2: 0xFF   D3: 0xFF  D4-D5: VSS ( (actual VSS + 100) * 10000)  D6: TPS (.5% increments 0-200 (0x00-0xC8))  D7: 0xFF

0x212:  (7B): ABS and DSC system   D0-D1 : 0x00                                 D2: bit3 - DSC MIL   D4: bit4 - ABS MIL  bit7 - Brake failure MIL  D5: bit5- TC OFF MIL bit6 - TC Active  D6: 0x00


*/

//for 0x212 just send all 0x00 7 byte messages (no fault)


#define id_STEERING_ANGLE 0x81
#define id_RPM_SPEED 0x201
#define id_TRAC_STATUS 0x212

#define id_EPS_STATUS 0x300

#define len_STEERING_ANGLE 4
#define len_RPM_SPEED 8
#define len_TRAC_STATUS 7

#define len_EPS_STATUS 1


#define mask_EPS_FAIL 0x80 //MSB is the fail bit

#define STEERING_ANGLE_EPSILON 2 // steering deadzone from center.


//TPS pos should be retrieved in .5% definition from ECU / Analog read, ranging from 0-200 


unsigned short steering_angle; //0 = right, 100 = left.
unsigned short steering_center = 50U;
unsigned short rpm;
unsigned short tps; //0-200 in .5% increments.
unsigned short vss_kph;
bool eps_error = 0;


byte data_trac_nofail[7] = {0x00U,0x00U,0x00U,0x00U,0x00U,0x00U,0x00U};

byte txBuf[8] = {0x00U,0x00U,0x00U,0x00U,0x00U,0x00U,0x00U,0x00U};


unsigned long int receiveID;
unsigned char len = 8;
unsigned char rxBuf[8];

unsigned long int loop_cnt = 0U;
short send_can_loop_freq = 5;

void signalEPSFault(byte can_data);

void assemble_steering_angle(byte* buf);

void assemble_rpm_speed(byte* buf);

void clearTransferBuffer(byte* buf,unsigned int len);

void setup() {

  pinMode(LED_BUILTIN,OUTPUT);
  // put your setup code here, to run once:
  Serial.begin(115200);

  if(CAN0.begin(MCP_ANY,CAN_500KBPS,MCP_16MHZ) == CAN_OK){
    Serial.println("CAN0 online.");
    CAN0.setMode(MCP_NORMAL);
  }
  else{
    Serial.println("Error: CAN init.");
    CAN0.setMode(MCP_NORMAL);
  }

  //placeholders

  steering_angle = 50U; 
  rpm = 1000U; 
  tps = 0U;
  vss_kph = 0U;
}

void loop() {
  //send out messages
  if(loop_cnt % send_can_loop_freq == 0){
    CAN0.sendMsgBuf(id_TRAC_STATUS,len_TRAC_STATUS,data_trac_nofail); //0x212
    Serial.println("ABS/DSC Data (0x212) sent.");
    assemble_rpm_speed(txBuf);
    CAN0.sendMsgBuf(id_RPM_SPEED,len_RPM_SPEED,txBuf); //0x201
    Serial.println("Engine Data (0x201) sent.");
    assemble_steering_angle(txBuf);
    CAN0.sendMsgBuf(id_STEERING_ANGLE,len_STEERING_ANGLE,txBuf); //0x81
    Serial.println("Steering angle Data (0x81) sent.");

  }



//reading incoming messages

  if(!digitalRead(CAN0_INT)){ // read on interrupt
    CAN0.readMsgBuf(&receiveID,&len,rxBuf);

    switch(receiveID){
      case id_EPS_STATUS: // receive 0x300.
        signalEPSFault(rxBuf[0]);
        break;
      default:
      //ignore other IDS.
      break;
    }
  }
  delay(50);

  loop_cnt++;
}

void clearTransferBuffer(byte* buf,unsigned int len){
  for(size_t i = 0; i < len; ++i){
    buf[i] = (byte)0x00;
  }
}

// parses EPS status message and illuminates onboard LED on fault.
void signalEPSFault(byte can_data){
  eps_error = can_data & mask_EPS_FAIL; 
  eps_error? digitalWrite(LED_BUILTIN,HIGH) : digitalWrite(LED_BUILTIN,LOW); // illuminate onboard LED based on fail signal received from EPS ECU.
}

//assembles message 0x81
void assemble_steering_angle(byte* buf){

  clearTransferBuffer(buf,8);

  uint16_t dataSteeringAngle = (uint32_t(0xFDE1 - 0x021E) * steering_angle / 100) + 0x021E;// rescale steering angle to the specified range.
  buf[0] = 0x00U;
  if(steering_center - STEERING_ANGLE_EPSILON <= steering_angle && steering_center + STEERING_ANGLE_EPSILON >= steering_angle){ //check for center position, with deadzone
    buf[1] = 0xEFU;
  }
  else{
    buf[1] = 0x00U;
  }

  buf[2] = (byte)(dataSteeringAngle >> 8);
  buf[3] = (byte)(dataSteeringAngle & 0xFFU);
  
}

//assembles message 0x201
void assemble_rpm_speed(byte* buf){
  
  clearTransferBuffer(buf,8);

  uint16_t dataRPM = (uint16_t)rpm * 4U; //3.85??
  uint16_t dataVSS = (uint16_t)((vss_kph * 100) + 10000);
  uint8_t dataTPS = tps;

  buf[0] = (byte)(dataRPM >> 8); //higher byte
  buf[1] = (byte)(dataRPM & 0xFFU); //lower byte
  buf[2] = (byte)0xFFU;
  buf[3] = (byte)0xFFU;
  buf[4] = (byte)(dataVSS >> 8);
  buf[5] = (byte)(dataVSS & 0xFFU);
  buf[6] = (byte)(tps);
  buf[7] = (byte)(0xFFU);
}


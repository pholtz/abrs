/////////////////////////////////////////////////////////////////////////
//This program now controls the logic for the Bike Rack state machine.
//This FSM consists of only several basic states. Further implementation
//may require substates to handle complexity.
//Paul Holtz 7/23/2014 @3:50pm
/////////////////////////////////////////////////////////////////////////
//This program interfaces the Atmega328 with the MCP 2515 CAN controller.
//View the datasheet for more information on SPI interface Commands and
//timing diagrams.
//http://ww1.microchip.com/downloads/en/DeviceDoc/21801e.pdf
//Paul Holtz 7/11/3014 @ 9:43am
/////////////////////////////////////////////////////////////////////////


#include <SPI.h>
#include <Adafruit_PN532.h>
#include "Timer.h"

#define NFC_SENSOR
#define MCP2515

#define NUM_TOWERS_ONLINE 1

/////////////////////////////////////////////////////////////////////////


/*******************************************
***************  CAN STUFF  ****************
********************************************/
//CAN MESSAGE ID'S
const byte canid_bike_status_request =     0x10;
const byte canid_bike_status_response =    0x11;
const byte canid_ack =                     0x12;
const byte canid_nack =                    0x13;
const byte canid_unlock_bike_request =     0x14;
const byte canid_flag_bike_request =       0x15;

//MCP2515 INSTRUCTIONS
const byte reset_instruction =       B11000000;
const byte read_instruction =        B00000011;
const byte write_instruction =       B00000010;
const byte read_status_instruction = B10100000;
const byte bit_modify_instruction =  B00000101;

//MCP2515 REGISTERS
//message reception registers
const byte rxb0_sidh = 0x61;
const byte rxb0_sidl = 0x62;
const byte rxb0_dlc = 0x65;
const byte rxb0_d0 = 0x66;
//message transmission registers
const byte txb0_sidh = 0x31;
const byte txb0_sidl = 0x32;
const byte txb0_dlc = 0x35;
const byte txb0_d0 = 0x36;
//utility registers
const byte canstat = 0x0E;
const byte canctrl = 0x0F;
const byte caninte = 0x2B;
const byte canintf = 0x2C;

/************************************
************ NFC SENSOR *************
************************************/
#ifdef NFC_SENSOR
  //MEGA2560 PINS
  #define SCK   (38)
  #define MOSI  (40)
  #define SS1   (39)
  #define MISO  (41)
  #define SS2   (42)
  #define SS3   (43)
  #define SS4   (44)
  #define SS5   (45)
  //Super janky object-array declaration of NFC objects
  Adafruit_PN532 nfc[5] = {
    Adafruit_PN532(SCK, MISO, MOSI, SS1),
    Adafruit_PN532(SCK, MISO, MOSI, SS2),
    Adafruit_PN532(SCK, MISO, MOSI, SS3),
    Adafruit_PN532(SCK, MISO, MOSI, SS4),
    Adafruit_PN532(SCK, MISO, MOSI, SS5)
  };
#endif

/************************************
********* BIKE RACK VARIABLES  ******
************************************/
//MAIN STATE MACHINE STATES
#define IDLE                   0
#define UPDATE_BIKE_STATUS     1
#define UNLOCK_BIKE            2
#define FLAG_BIKE              3

//INDIVIDUAL BIKE STATES
#define BIKE_PRESENT_LOCKED    0
#define BIKE_PRESENT_UNLOCKED  1
#define BIKE_NOT_PRESENT       2
#define BIKE_ACCEPT_RETURN     3

//PIN MAPPING
const int mcp2515_cs = 53;
//bike solenoids
const int sol[5] = {22, 23, 24, 25, 26};
//position switches
const int pswitch[5] = {28, 29, 30, 31, 32};

//Flags & Vars
byte controller_main_state;
byte bike_main_state[5];
byte bike_status;
byte count;
byte bike_unlock_number;
byte bike_flag_number;
byte bike_unlock_flag[5];
byte bike_unlock_result[5];


Timer diagnostic_display;

//Function Prototypes
void mcp2515_init();
void reset_controller();
void clear_rxb0();
void clear_rxb1();
void clear_txb0();
uint8_t read_from_controller(const byte &address);
void write_to_controller(byte address, byte data_out);
void send_CAN_message(byte canid, byte d0, byte buffer);
boolean receive_CAN_message(byte canid);
byte read_status();
boolean unlock_bike(byte bike_num);
boolean flag_bike(byte bike_num);
void print_mcp2515_info();
void bit_modify(byte address, byte mask, byte data);

/************
**  SETUP  **
************/
void setup()
{
  //Init and Raise CS pin for the CAN controller
  pinMode(mcp2515_cs, OUTPUT);
  digitalWrite(mcp2515_cs, HIGH);
  
  //Init the timer event
  int print_diagnostic_event = diagnostic_display.every(2000, print_mcp2515_info);
  
  //Init the serial connection with pc
  Serial.begin(115200);
  Serial.println("Hello!");
  
/***************
**  NFC INIT  **
***************/
#ifdef NFC_SENSOR
  
  //NFC INITIALIZATION
  for(int i = 0; i < NUM_TOWERS_ONLINE; i++)
  {
    nfc[i].begin();
  
    uint32_t versiondata = nfc[i].getFirmwareVersion();
    if (! versiondata) {
      Serial.print("Didn't find PN53x board");
      while (1); // halt
    }
    // Got ok data, print it out!
    Serial.print("Found chip PN5"); Serial.println((versiondata>>24) & 0xFF, HEX); 
    Serial.print("Firmware ver. "); Serial.print((versiondata>>16) & 0xFF, DEC); 
    Serial.print('.'); Serial.println((versiondata>>8) & 0xFF, DEC);
    
    // configure board to read RFID tags
    nfc[i].SAMConfig();
    
    Serial.println("Waiting for an ISO14443A Card ...");
  }
#endif
  
/***************
**  SPI INIT  **
***************/
#ifdef MCP2515
  //CAN Controller is using Arduino SPI
  //Initialize that Library and set up bus
  SPI.begin();
  SPI.setDataMode(SPI_MODE0);
  SPI.setClockDivider(SPI_CLOCK_DIV8);
  SPI.setBitOrder(MSBFIRST);  
#endif
  
  //Init position switches, locking solenoids, and bike states
  for(int i = 0; i < 5; i++)
  {
    //Init Position Switches
    pinMode(pswitch[i], INPUT);
    //Init Locking Solenoids
    pinMode(sol[i], OUTPUT);
    digitalWrite(sol[i], LOW);
    //Init Bike States
    bike_main_state[i] = BIKE_PRESENT_LOCKED;
    //Clear Bike Unlock Flags
    bike_unlock_flag[i] = 0;
    bike_unlock_result[i] = 0;
  }
  
  //initialize flags
  count = 0;
  
  //initialize the rack state machine
  controller_main_state = IDLE;
  
#ifdef MCP2515
  //always reset the CAN controller on startup
  mcp2515_init();
#endif
  
  delay(100);
}


/****************
**  MAIN LOOP  **
****************/
void loop()
{ 
#ifdef MCP2515        
    //Master state machine for the bike rack controller  
    switch(controller_main_state)
    {
      //When in this state, the controller will update the bike_status_update timer,
      //check the CAN bus for an unlock message from the kiosk, and monitor sensors
      //for an active state. The state machine should sit in this state most of the time.
      default:
      case IDLE:
        
        
        /////////////////////
        //STATE TRANSITIONS//
        /////////////////////   
        //BIKE STATUS REQUEST
        if(receive_CAN_message(canid_bike_status_request))
        {
          //clear RXB0
          clear_rxb0();
          //clear CANINTF.RX0IF
          bit_modify(canintf, 0x01, 0x00);
          
          //switch state
          controller_main_state = UPDATE_BIKE_STATUS;
          Serial.println("CAN: Received a Bike Status Request");
        }
        //UNLOCK BIKE REQUEST
        else if(receive_CAN_message(canid_unlock_bike_request))
        {
          //store the incoming CAN data so we know which bike to unlock
          bike_unlock_number = read_from_controller(rxb0_d0);
          //clear the receive buffer and CANINTF.RX0IF so new messages can be received
          clear_rxb0();
          bit_modify(canintf, 0x01, 0x00);
          
          //switch state
          controller_main_state = UNLOCK_BIKE;
          bike_unlock_flag[bike_unlock_number - 1] = 1;
          Serial.println("CAN: Received a Bike Unlock Request");
        }
        //FLAG BIKE REQUEST
        else if(receive_CAN_message(canid_flag_bike_request))
        {
          //store the incoming CAN data byte
          bike_flag_number = read_from_controller(rxb0_d0);
          //clear the receive buffer and CANINTF.RX0IF
          clear_rxb0();
          bit_modify(canintf, 0x01, 0x00);
          
          //switch state
          controller_main_state = FLAG_BIKE;
          Serial.println("CAN: Received a Flag Bike Request");
        }
        

        //Serial.println("STATE: IDLE");
        break;
      
      
      //The controller should be put into this state when the kiosk requests a bike status update.
      case UPDATE_BIKE_STATUS:

        //Acquire bike data
        update_bike_status();

        //send the bike status message over CAN
        send_CAN_message(canid_bike_status_response, bike_status, 0);
        
        //Block until CAN message has been successfully sent, then clear buffer and interrupt
        while((read_from_controller(canintf) & 0x04) != 0x04);
        bit_modify(canintf, 0x04, 0x00);
        clear_txb0();
        
        controller_main_state = IDLE;
                
        //Serial.println("STATE: UPDATE_BIKE_STATUS");
        Serial.println("Sent a Bike Status Update");
        //Serial.println(bike_status, BIN);
        break;
        
        
      //The controller is put into this state when the kiosk sends the unlock bike CAN message.
      case UNLOCK_BIKE:
        
        //Remove me when pswitches and nfc is in place on tower
        /*
        if(unlock_bike(bike_unlock_number))
        {
          //ack kiosk -- bike successfully unlocked
          send_CAN_message(canid_ack, 0x00, 0);
          //Set the new bike state for that bike
          bike_main_state[bike_unlock_number - 1] = BIKE_NOT_PRESENT;
          Serial.println("Successfully Unlocked a Bike");
        }
        else
        {
          //nack kiosk -- bike not unlocked
          send_CAN_message(canid_nack, 0x00, 0);
          Serial.println("Could not Successfully Unlock Bike");
        }
        */
        
        //Wait for response from Bike State Machine
        for(int i = 0; i < 5; i++)
        {
          if(bike_unlock_result[i] == 1)  //bike successfully unlocked
          {
            controller_main_state = IDLE;
            send_CAN_message(canid_ack, 0x00, 0);
            
            //Print Diagnostic Info
            Serial.println("Successfully Unlocked a Bike");
            //Clear bike flags
            bike_unlock_result[i] = 0;
            bike_unlock_number = 0;
          }
          else if(bike_unlock_result[i] == 2)  //bike unsuccessfully unlocked
          {
            controller_main_state = IDLE;
            send_CAN_message(canid_nack, 0x00, 0);
            
            //Print Diagnostic Info
            Serial.println("Could not Successfully Unlock Bike");
            //Clear bike flags
            bike_unlock_result[i] = 0;
            bike_unlock_number = 0;
          }
        }
        
        break;
        
      case FLAG_BIKE:
        if(flag_bike(bike_flag_number))
        {
          send_CAN_message(canid_ack, 0x00, 0);
          Serial.println("Successfully flagged a bike");
        }
        else
        {
          send_CAN_message(canid_nack, 0x00, 0);
          Serial.println("Could not successfully flag bike");
        }
        
        controller_main_state = IDLE;
        break;
    }
#endif
  
  //Run the bike state machine
  update_bike_state();
  
  //update the serial diagnostic display
  diagnostic_display.update();

}


/************************************
*****  BIKE LOGIC FUNCTIONS  ********
************************************/

//The Bike State Machine
void update_bike_state()
{
  uint8_t uid[] = { 0, 0, 0, 0, 0, 0, 0 };  // Buffer to store the returned UID
  uint8_t uidLength;                        // Length of the UID (4 or 7 bytes depending on ISO14443A card type)
  
  //Run the state machine for each bike in the rack
  for(int i = 0; i < NUM_TOWERS_ONLINE; i++)
  {
    if(bike_main_state[i] == BIKE_PRESENT_LOCKED)
    {
      //Set the state output
      digitalWrite(sol[i], LOW);
      
      //Check transition conditions
      if(bike_unlock_flag[i] == 1)
      {
        bike_unlock_flag[i] = 0;
        bike_main_state[i] = BIKE_PRESENT_UNLOCKED;
      }
    }
    else if(bike_main_state[i] == BIKE_PRESENT_UNLOCKED)
    {
      //Set the state output
      digitalWrite(sol[i], HIGH);
      
      //Check transition conditions
      if(!nfc[i].readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength) && digitalRead(pswitch[i]) == HIGH)
      {
        bike_unlock_result[i] = 1;  //report success to control state machine
        bike_main_state[i] = BIKE_NOT_PRESENT;
      }
      //TODO: TIMEOUT AFTER TIME LIMIT?
      
    }
    //Process Bike Returns
    else if(bike_main_state[i] == BIKE_NOT_PRESENT)
    {
      //Set the state output
      digitalWrite(sol[i], LOW);
      
      //Check transition conditions
      if(digitalRead(pswitch[i]) == LOW)
      {
        bike_main_state[i] = BIKE_ACCEPT_RETURN;
      }
    }
    else if(bike_main_state[i] == BIKE_ACCEPT_RETURN)
    {
      //Set the state output
      digitalWrite(sol[i], HIGH);
      
      //Check transition conditions
      if(nfc[i].readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength) && digitalRead(pswitch[i]) == LOW)
      {
        bike_main_state[i] = BIKE_PRESENT_LOCKED;
      }
      else if(!nfc[i].readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength) && digitalRead(pswitch[i]) == HIGH)
      {
        bike_main_state[i] = BIKE_NOT_PRESENT;
      }
    }
  }
  
}


boolean flag_bike(byte bike_num)
{
  return false;
}


void update_bike_status()
{
#ifdef NFC_SENSOR
//  //poll the sensors to get a current reading on which bikes are present in the rack
  uint8_t uid[] = { 0, 0, 0, 0, 0, 0, 0 };  // Buffer to store the returned UID
  uint8_t uidLength;                        // Length of the UID (4 or 7 bytes depending on ISO14443A card type)
    
  // Wait for an ISO14443A type cards (Mifare, etc.).  When one is found
  // 'uid' will be populated with the UID, and uidLength will indicate
  // if the uid is 4 bytes (Mifare Classic) or 7 bytes (Mifare Ultralight)
  for(int i = 0; i < NUM_TOWERS_ONLINE; i++)
  {
    if(nfc[i].readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength))
    {
      //Bike i is present
      bike_status = bike_status | (0x01 << i);
    }
    else
    {
      bike_status = bike_status & (0xFF ^ (0x01 << i));
    }
  }
//  //update the bike_status variable
//  
#endif
}


//Deprecated -- unlocking will be handled by bike state machine
boolean unlock_bike(byte bike_num)
{
  /*
  uint8_t uid[] = { 0, 0, 0, 0, 0, 0, 0 };  // Buffer to store the returned UID
  uint8_t uidLength;                        // Length of the UID (4 or 7 bytes depending on ISO14443A card type)
  
  for(int i = 0; i < 5; i++)
  {
    if(bike_num == (i + 1) && nfc[i].readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength))
    {
      digitalWrite(sol[i], HIGH);
      //delay(5000);
      bike_main_state[i] = BIKE_PRESENT_UNLOCKED;
      //Function blocks for now...obviously this will need to be removed in the future.
      while(nfc[i].readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength));
      digitalWrite(sol[i], LOW);
      //mark as rented
      bike_main_state[i] = BIKE_NOT_PRESENT;
      
      return true;
    }
  }
  */
  return false;
}


void print_mcp2515_info()
{
#ifdef MCP2515
    if(controller_main_state == IDLE)
    {
      Serial.println("Main State: IDLE");
    }
    else if(controller_main_state == UPDATE_BIKE_STATUS)
    {
      Serial.println("Main State: UPDATE_BIKE_STATUS");
    }
    else if(controller_main_state == UNLOCK_BIKE)
    {
      Serial.println("Main State: UNLOCK_BIKE");
    }
    else if(controller_main_state == FLAG_BIKE)
    {
      Serial.println("Main State: FLAG_BIKE");
    }
    
    Serial.println("Bike Statuses:");
    for(int i = 0; i < 5; i++)
    {
      if(bike_main_state[i] == BIKE_PRESENT_LOCKED)
      {
        Serial.println("BIKE_PRESENT_LOCKED");
      }
      else if(bike_main_state[i] == BIKE_PRESENT_UNLOCKED)
      {
        Serial.println("BIKE_PRESENT_UNLOCKED");
      }
      else if(bike_main_state[i] == BIKE_NOT_PRESENT)
      {
        Serial.println("BIKE_NOT_PRESENT");
      }
      else if(bike_main_state[i] == BIKE_ACCEPT_RETURN)
      {
        Serial.println("BIKE_ACCEPT_RETURN");
      }
    }
    
    //read CANSTAT register
    Serial.println("CANSTAT REGISTER");
    Serial.println(read_from_controller(0x0E), BIN);
    //read status register
    Serial.println("STATUS REGISTER");
    Serial.println(read_status(), BIN);;
    //read CANINTF register
    Serial.println("CANINTF REGISTER");
    Serial.println(read_from_controller(0x2C), BIN);
#endif

#ifdef NFC_SENSOR
  uint8_t success;
  uint8_t uid[] = { 0, 0, 0, 0, 0, 0, 0 };  // Buffer to store the returned UID
  uint8_t uidLength;                        // Length of the UID (4 or 7 bytes depending on ISO14443A card type)
    
  // Wait for an ISO14443A type cards (Mifare, etc.).  When one is found
  // 'uid' will be populated with the UID, and uidLength will indicate
  // if the uid is 4 bytes (Mifare Classic) or 7 bytes (Mifare Ultralight)
  success = nfc[0].readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength);
  
  if (success) {
    // Display some basic information about the card
    Serial.println("Found an ISO14443A card");
    Serial.print("  UID Length: ");Serial.print(uidLength, DEC);Serial.println(" bytes");
    Serial.print("  UID Value: ");
    nfc[0].PrintHex(uid, uidLength);
    
    if (uidLength == 4)
    {
      // We probably have a Mifare Classic card ... 
      uint32_t cardid = uid[0];
      cardid <<= 8;
      cardid |= uid[1];
      cardid <<= 8;
      cardid |= uid[2];  
      cardid <<= 8;
      cardid |= uid[3]; 
      Serial.print("Seems to be a Mifare Classic card #");
      Serial.println(cardid);
    }
    Serial.println("");
  }
#endif

    Serial.println("loop\n");

}










/////////////////////////////////////////////////////////////////////
//MCP2515 CAN CONTROLLER FUNCTIONS  -- DO NOT MODIFY UNLESS NECESSARY
/////////////////////////////////////////////////////////////////////

void mcp2515_init()
{
  reset_controller();
  delay(500);
  
  //these settings work, but leave them out for now
//  //set the BRP(baud rate prescaler) and Phase Segs 1 and 2
//  //BRP = 10, SJW = 0
//  write_to_controller(0x2A, B01010010);
//  
//  //PRSEG = 1, PHSEG = 1, SAM = 0, BTLMODE = 1
//  write_to_controller(0x29, B10010001);
//  
//  //PHSEG2 = 1, WAKFIL = 0, SOF = 0
//  write_to_controller(0x28, B10000000);
  //Serial.println(read_from_controller(canstat), BIN);

  //enable CANINTE
  write_to_controller(0x2B, 0xFF);

  //request normal operation mode
  write_to_controller(0x0F, B00000000);
  
  //turn off RXB0 filtering for now -- RXB1 will still filter
  write_to_controller(0x60, B01100000);
  
  //clear the receive buffers RXB0 and RXB1
  clear_rxb0();
  clear_rxb1();
  
  //Serial.println(read_from_controller(canstat), BIN);
}

//erase all received message data in Buffer0
void clear_rxb0()
{
  write_to_controller(0x61, 0x00);  //RXB0SIDH
  write_to_controller(0x62, 0x00);  //RXB0SIDL
  write_to_controller(0x63, 0x00);  //RXB0EID8
  write_to_controller(0x64, 0x00);  //RXB0EID0
  write_to_controller(0x65, 0x00);  //RXB0DLC
  write_to_controller(0x66, 0x00);  //RXB0D0
  write_to_controller(0x67, 0x00);  //RXB0D1
  write_to_controller(0x68, 0x00);  //RXB0D2
  write_to_controller(0x69, 0x00);  //RXB0D3
  write_to_controller(0x6A, 0x00);  //RXB0D4
  write_to_controller(0x6B, 0x00);  //RXB0D5
  write_to_controller(0x6C, 0x00);  //RXB0D6
  write_to_controller(0x6D, 0x00);  //RXB0D7
}

//erase all received message data in Buffer1
void clear_rxb1()
{
  write_to_controller(0x71, 0x00);  //RXB1SIDH
  write_to_controller(0x72, 0x00);  //RXB1SIDL
  write_to_controller(0x73, 0x00);  //RXB1EID8
  write_to_controller(0x74, 0x00);  //RXB1EID0
  write_to_controller(0x75, 0x00);  //RXB1DLC
  write_to_controller(0x76, 0x00);  //RXB1D0
  write_to_controller(0x77, 0x00);  //RXB1D1
  write_to_controller(0x78, 0x00);  //RXB1D2
  write_to_controller(0x79, 0x00);  //RXB1D3
  write_to_controller(0x7A, 0x00);  //RXB1D4
  write_to_controller(0x7B, 0x00);  //RXB1D5
  write_to_controller(0x7C, 0x00);  //RXB1D6
  write_to_controller(0x7D, 0x00);  //RXB1D7
}

void clear_txb0()
{
  write_to_controller(0x31, 0x00);  //TXB0SIDH
  write_to_controller(0x32, 0x00);  //TXB0SIDL
  write_to_controller(0x33, 0x00);  //TXB0EID8
  write_to_controller(0x34, 0x00);  //TXB0EID0
  write_to_controller(0x35, 0x00);  //TXB0DLC
  write_to_controller(0x36, 0x00);  //TXB0D0
  write_to_controller(0x37, 0x00);  //TXB0D1
  write_to_controller(0x38, 0x00);  //TXB0D2
  write_to_controller(0x39, 0x00);  //TXB0D3
  write_to_controller(0x3A, 0x00);  //TXB0D4
  write_to_controller(0x3B, 0x00);  //TXB0D5
  write_to_controller(0x3C, 0x00);  //TXB0D6
  write_to_controller(0x3D, 0x00);  //TXB0D7
}

//to reset the MCP2515, simply send the reset Byte instruction B00000011
//after lowering CS. Once it has been sent, raise CS.
void reset_controller()
{
  //prepare the MCP 2515 to receive an instruction
  digitalWrite(mcp2515_cs, LOW);
  
  //send the reset instruction byte
  SPI.transfer(reset_instruction);
  
  //end the instruction by raising CS
  digitalWrite(mcp2515_cs, HIGH);
  
  delay(10);
}

//to read from the MCP2515, first lower CS. Then send the read Byte
//instruction 00000011, followed by the address of the register you
//would like to read. The MCP2515 will then send out the contents
//of the register on the MISO line.

//addressing is as follows:
//read_from_controller(0x0E) would read address 00001110
//which is the location of the canstat register. Interestingly,
//any value 0 - F can be used as the higher order address bits
//for this register. This also applies for the canctrl register.
//I would imagine this was done on purpose.
uint8_t read_from_controller(const byte &address)
{
  //prepare the MCP2515 to receive an instruction
  digitalWrite(mcp2515_cs, LOW);
  
  //send the read instruction byte to the MCP2515
  SPI.transfer(read_instruction);
  
  //send the address you would like to read
  SPI.transfer(address);
  
  //send a zero and read the value returned by the MCP2515
  byte data_in = SPI.transfer(0x00);
  
  //end the instruction by raising CS
  digitalWrite(mcp2515_cs, HIGH);
  
  return data_in;
}

void write_to_controller(byte address, byte data_out)
{
  //prepare the MCP2515 to receive an instruction
  digitalWrite(mcp2515_cs, LOW);
  
  //send the write instruction byte to the MCP2515
  SPI.transfer(write_instruction);
  
  //send the address of the register you would like to write to
  SPI.transfer(address);
  
  //put the data you would like to write on the MOSI line
  SPI.transfer(data_out);
  
  //terminate the instruction by raising CS
  digitalWrite(mcp2515_cs, HIGH);
}

byte read_status()
{
  digitalWrite(mcp2515_cs, LOW);
  SPI.transfer(read_status_instruction);
  byte status = SPI.transfer(0x00);
  digitalWrite(mcp2515_cs, HIGH);
  return status;
}

void bit_modify(byte address, byte mask, byte data)
{
  digitalWrite(mcp2515_cs, LOW);
  SPI.transfer(bit_modify_instruction);
  SPI.transfer(address);
  SPI.transfer(mask);
  SPI.transfer(data);
  digitalWrite(mcp2515_cs, HIGH);
}

void request_to_send(byte buffer)
{
  digitalWrite(mcp2515_cs, LOW);
  
  if(buffer == 0)
  {
    SPI.transfer(10000001);
  }
  else if(buffer == 1)
  {
    SPI.transfer(10000010);
  }
  else if(buffer == 2)
  {
    SPI.transfer(10000100);
  }
  
  digitalWrite(mcp2515_cs, HIGH);
}

//For now, all CAN messages sent and received by the kiosk and display are limited to 1B of data.
//Thus, DLC will always be 0x01 and the data will always be stored in Data0. This may change in
//future revisions. This function is written with the kiosk as the intended recipient.
void send_CAN_message(byte canid, byte d0, byte buffer)
{
  switch(buffer)
  {
    default:
    case 0:
      write_to_controller(0x31, canid);    //pack the SID_H reg with the CAN ID
      write_to_controller(0x32, 0x00);     //pack the SID_L reg for kiosk as intended recipient
      write_to_controller(0x35, 0x01);     //pack the DLC reg with a 1
      write_to_controller(0x36, d0);       //pack the Data0 reg
      request_to_send(0);
      break;
      
    case 1:
      write_to_controller(0x41, canid);
      write_to_controller(0x42, 0x00);
      write_to_controller(0x45, 0x01);
      write_to_controller(0x46, d0);
      request_to_send(1);
      break;
      
    case 2:
      write_to_controller(0x51, canid);
      write_to_controller(0x52, 0x00);
      write_to_controller(0x55, 0x01);
      write_to_controller(0x56, d0);
      request_to_send(2);
      break;
  }
}

boolean receive_CAN_message(byte canid)
{
  if((read_from_controller(canintf) & 0x01) == 0x01)  //message received in RXB0
  {

    if(read_from_controller(rxb0_sidl) >> 5 == 0x01)  //msg has Rack 1 ID code
    {
      if(read_from_controller(rxb0_sidh) == canid)
      {
        return true;
      }
    }
  }
  return false;
}



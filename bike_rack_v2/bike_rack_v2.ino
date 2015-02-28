#include <SPI.h>
#include "Timer.h"
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



#define NFC_SENSOR
#define MCP2515

#include <Adafruit_PN532.h>

/////////////////////////////////////////////////////////////////////////

#ifdef NFC_SENSOR
  //MEGA2560 PINS
  #define SCK  (38)
  #define MOSI (40)
  #define SS   (39)
  #define MISO (41)
  Adafruit_PN532 nfc(SCK, MISO, MOSI, SS);
#endif

//MAIN STATE MACHINE STATES
#define IDLE                   0
#define UPDATE_BIKE_STATUS     1
#define UNLOCK_BIKE            2
#define ACCEPT_RETURN          3

//CAN MESSAGE ID'S
const byte canid_bike_status_request =     0x10;
const byte canid_bike_status_response =    0x11;
const byte canid_ack =                     0x12;
const byte canid_nack =                    0x13;
const byte canid_unlock_bike_request =     0x14;
const byte canid_unlock_bike_response =    0x15;

//PIN MAPPING
const int chip_select = 53;
const int idle_led = 30;
const int update_bike_status_led = 31;
const int unlock_bike_led = 32;
const int accept_return_led = 33;
const int interrupt_led = 44;

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

//Flags & Vars
byte send_once;
byte led_flag;
byte rack_main_state;
byte rxb0d0;
byte bike_status;
byte count;
byte bike_unlock_number;

Timer timer;

//Function Prototypes
void mcp2515_init();
void reset_controller();
void clear_rxb0();
void clear_rxb1();
void clear_txb0();
void generic_interrupt();
uint8_t read_from_controller(const byte &address);
void write_to_controller(byte address, byte data_out);
void send_CAN_message(byte canid, byte d0, byte buffer);
boolean receive_CAN_message(byte canid);
byte read_status();
boolean unlock_bike(byte bike_num);
void print_mcp2515_info();
void bit_modify(byte address, byte mask, byte data);

void setup()
{
  //Serial.begin(9600);
  pinMode(chip_select, OUTPUT);
  digitalWrite(chip_select, HIGH);
  
  int tickEvent = timer.every(2000, print_mcp2515_info);
  
  Serial.begin(115200);
  Serial.println("Hello!");
  
#ifdef NFC_SENSOR
  
  //NFC INITIALIZATION

  nfc.begin();

  uint32_t versiondata = nfc.getFirmwareVersion();
  if (! versiondata) {
    Serial.print("Didn't find PN53x board");
    while (1); // halt
  }
  // Got ok data, print it out!
  Serial.print("Found chip PN5"); Serial.println((versiondata>>24) & 0xFF, HEX); 
  Serial.print("Firmware ver. "); Serial.print((versiondata>>16) & 0xFF, DEC); 
  Serial.print('.'); Serial.println((versiondata>>8) & 0xFF, DEC);
  
  // configure board to read RFID tags
  nfc.SAMConfig();
  
  Serial.println("Waiting for an ISO14443A Card ...");
  
#endif

#ifdef MCP2515
  SPI.begin();
  SPI.setDataMode(SPI_MODE0);
  SPI.setClockDivider(SPI_CLOCK_DIV8);
  SPI.setBitOrder(MSBFIRST);  
  
#endif
  
  //attach an interrupt to the generic interrupt pin on the MCP2515
  attachInterrupt(2, generic_interrupt, CHANGE);
  
  //set up the output LED's
  pinMode(idle_led, OUTPUT);
  pinMode(update_bike_status_led, OUTPUT);
  pinMode(unlock_bike_led, OUTPUT);
  pinMode(accept_return_led, OUTPUT);
  pinMode(interrupt_led, OUTPUT);

  //write the leds low
  digitalWrite(idle_led, LOW);
  digitalWrite(update_bike_status_led, LOW);
  digitalWrite(unlock_bike_led, LOW);
  digitalWrite(accept_return_led, LOW);
  digitalWrite(interrupt_led, LOW);
  
  //initialize flags
  send_once = 0;
  led_flag = 0;
  count = 0;
  
  //initialize the state machine
  rack_main_state = IDLE;
  
#ifdef MCP2515
  //always reset the CAN controller on startup
  mcp2515_init();
#endif
  
  delay(100);
}

void loop()
{ 
#ifdef MCP2515        
    //Master state machine for the bike rack controller  
    switch(rack_main_state)
    {
      //When in this state, the controller will update the bike_status_update timer,
      //check the CAN bus for an unlock message from the kiosk, and monitor sensors
      //for an active state. The state machine should sit in this state most of the time.
      default:
      case IDLE:
        digitalWrite(idle_led, HIGH);
        digitalWrite(update_bike_status_led, LOW);
        digitalWrite(unlock_bike_led, LOW);
        digitalWrite(accept_return_led, LOW);
        
        //Check sensors for an active state
        //If a return is detected, put controller into accept_return state
        
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
          rack_main_state = UPDATE_BIKE_STATUS;
          Serial.println("Received a Bike Status Request");
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
          rack_main_state = UNLOCK_BIKE;
          Serial.println("Received a Bike Unlock Request");
        }
        

        //Serial.println("STATE: IDLE");
        break;
      
      
      //The controller should be put into this state when the kiosk requests a bike status update.
      //This state is similar to the periodic status update that the rack controller will perform
      //on its own, however the resulting CAN message has a different CAN ID code so the kiosk
      //can differentiate between the two.
      case UPDATE_BIKE_STATUS:
        digitalWrite(idle_led, LOW);
        digitalWrite(update_bike_status_led, HIGH);
        digitalWrite(unlock_bike_led, LOW);
        digitalWrite(accept_return_led, LOW);
        //poll sensors, get a current reading from each

        //compile sensor data into a bike status
        bike_status = count;
        count++;
        //pack the bike status message

        //send the bike status message over CAN
        send_CAN_message(canid_bike_status_response, bike_status, 0);
        
        //Block until CAN message has been successfully sent, then clear buffer and interrupt
        while((read_from_controller(canintf) & 0x04) != 0x04);
        bit_modify(canintf, 0x04, 0x00);
        clear_txb0();
        
        rack_main_state = IDLE;
                
        //Serial.println("STATE: UPDATE_BIKE_STATUS");
        Serial.println("Sent a Bike Status Update");
        break;
        
        
      //The controller is put into this state when the kiosk sends the unlock bike CAN message.
      //The unlock bike message should contain data as to which bike should be unlocked. The
      //controller should read this data and drive the lock for the selected bike, after first
      //checking that the requested bike is locked in place. If the requested bike is not found,
      //the rack should nack the controller so it knows that a misunderstanding has taken place.
      case UNLOCK_BIKE:
        digitalWrite(idle_led, LOW);
        digitalWrite(update_bike_status_led, LOW);
        digitalWrite(unlock_bike_led, HIGH);
        digitalWrite(accept_return_led, LOW);
        //simulate unlocking a bike with an led
        
        if(unlock_bike(bike_unlock_number))
        {
          //ack kiosk -- bike successfully unlocked
          send_CAN_message(canid_ack, 0x00, 0);
          Serial.println("Successfully Unlocked a Bike");
        }
        else
        {
          //nack kiosk -- bike not unlocked
          send_CAN_message(canid_nack, 0x00, 0);
          Serial.println("Could not Successfully Unlock Bike");
        }
        
        rack_main_state = IDLE;
        //Serial.println("STATE: UNLOCK_BIKE");
        break;
        
        
      case ACCEPT_RETURN:
        digitalWrite(idle_led, LOW);
        digitalWrite(update_bike_status_led, LOW);
        digitalWrite(unlock_bike_led, LOW);
        digitalWrite(accept_return_led, HIGH);
        
        //TODO: Allow for the return of a bike based on change in switch state and NFC tag present/!present
        
        //Serial.println("STATE: ACCEPT_RETURN");
        break;
      
      
    }
#endif
    

/*
    //read CANSTAT register
    Serial.println("CANSTAT REGISTER");
    Serial.println(read_from_controller(0x0E), BIN);
    //read status register
    Serial.println("STATUS REGISTER");
    Serial.println(read_status(), BIN);;
    //read CANINTF register
    Serial.println("CANINTF REGISTER");
    Serial.println(read_from_controller(0x2C), BIN);

    Serial.println("loop\n");
    delay(2000);
*/

  timer.update();

}

void update_bike_status()
{
//  //poll the sensors to get a current reading on which bikes are present in the rack
//  
//  //update the bike_status variable
//  
//  //send this data to the kiosk over CAN
//  
}

//Called everytime an interrupt is generated on the INT pin (active low) on the MCP2515
void generic_interrupt()
{
  //digitalWrite(led, HIGH);
  //write_to_controller(canintf, 0x00);
  digitalWrite(interrupt_led, HIGH);
}

void print_mcp2515_info()
{
#ifdef MCP2515
    if(rack_main_state == IDLE)
    {
      Serial.println("State: IDLE");
    }
    else if(rack_main_state == UPDATE_BIKE_STATUS)
    {
      Serial.println("State: UPDATE_BIKE_STATUS");
    }
    else if(rack_main_state == UNLOCK_BIKE)
    {
      Serial.println("State: UNLOCK_BIKE");
    }
    else if(rack_main_state == ACCEPT_RETURN)
    {
      Serial.println("State: ACCEPT_RETURN");
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
  success = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength);
  
  if (success) {
    // Display some basic information about the card
    Serial.println("Found an ISO14443A card");
    Serial.print("  UID Length: ");Serial.print(uidLength, DEC);Serial.println(" bytes");
    Serial.print("  UID Value: ");
    nfc.PrintHex(uid, uidLength);
    
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

///////////////////////////////////////////////////////
//MCP2515 CAN CONTROLLER FUNCTIONS
///////////////////////////////////////////////////////

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
  Serial.println(read_from_controller(canstat), BIN);

  //enable CANINTE
  write_to_controller(0x2B, 0xFF);

  //request normal operation mode
  write_to_controller(0x0F, B00000000);
  
  //turn off RXB0 filtering for now -- RXB1 will still filter
  write_to_controller(0x60, B01100000);
  
  //clear the receive buffers RXB0 and RXB1
  clear_rxb0();
  clear_rxb1();
  
  Serial.println(read_from_controller(canstat), BIN);
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
  digitalWrite(chip_select, LOW);
  
  //send the reset instruction byte
  SPI.transfer(reset_instruction);
  
  //end the instruction by raising CS
  digitalWrite(chip_select, HIGH);
  
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
  digitalWrite(chip_select, LOW);
  
  //send the read instruction byte to the MCP2515
  SPI.transfer(read_instruction);
  
  //send the address you would like to read
  SPI.transfer(address);
  
  //send a zero and read the value returned by the MCP2515
  byte data_in = SPI.transfer(0x00);
  
  //end the instruction by raising CS
  digitalWrite(chip_select, HIGH);
  
  return data_in;
}

void write_to_controller(byte address, byte data_out)
{
  //prepare the MCP2515 to receive an instruction
  digitalWrite(chip_select, LOW);
  
  //send the write instruction byte to the MCP2515
  SPI.transfer(write_instruction);
  
  //send the address of the register you would like to write to
  SPI.transfer(address);
  
  //put the data you would like to write on the MOSI line
  SPI.transfer(data_out);
  
  //terminate the instruction by raising CS
  digitalWrite(chip_select, HIGH);
}

byte read_status()
{
  digitalWrite(chip_select, LOW);
  SPI.transfer(read_status_instruction);
  byte status = SPI.transfer(0x00);
  digitalWrite(chip_select, HIGH);
  return status;
}

void bit_modify(byte address, byte mask, byte data)
{
  digitalWrite(chip_select, LOW);
  SPI.transfer(bit_modify_instruction);
  SPI.transfer(address);
  SPI.transfer(mask);
  SPI.transfer(data);
  digitalWrite(chip_select, HIGH);
}

void request_to_send(byte buffer)
{
  digitalWrite(chip_select, LOW);
  
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
  
  digitalWrite(chip_select, HIGH);
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


boolean unlock_bike(byte bike_num)
{
  return false;
}


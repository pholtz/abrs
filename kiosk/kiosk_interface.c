//ABRS Team 1
//Communication with the CAN controller
//Kiosk Controller
//Authored by Paul Holtz @2:17pm 2/6/2015
/////////////////////////////////////////

// After installing bcm2835, you can build this 
// with something like:
// gcc -o spi spi.c -l bcm2835
// sudo ./spi

#include <bcm2835.h>
#include <stdio.h>
#include <stdbool.h>

//MCP2515 INSTRUCTIONS
const uint8_t reset_instruction = 			0b11000000;
const uint8_t read_instruction = 			0b00000011;
const uint8_t write_instruction = 			0b00000010;
const uint8_t read_status_instruction = 	0b10100000;
const uint8_t bit_modify_instruction = 		0b00000101;

//MCP2515 REGISTERS
//message reception registers
const uint8_t rxb0_sidh = 	0x61;
const uint8_t rxb0_sidl = 	0x62;
const uint8_t rxb0_dlc = 	0x65;
const uint8_t rxb0_d0 = 	0x66;
//message transmission registers
const uint8_t txb0_sidh = 	0x31;
const uint8_t txb0_sidl = 	0x32;
const uint8_t txb0_dlc = 	0x35;
const uint8_t txb0_d0 = 	0x36;
//utility registers
const uint8_t canstat = 	0x0E;
const uint8_t canctrl = 	0x0F;
const uint8_t caninte = 	0x2B;
const uint8_t canintf = 	0x2C;
//CAN ID's
const uint8_t canid_bike_status_request = 	0x10;
const uint8_t canid_bike_status_response = 	0x11;
const uint8_t canid_ack = 					0x12;
const uint8_t canid_nack = 					0x13;
const uint8_t canid_unlock_bike_request = 	0x14;
const uint8_t canid_unlock_bike_response = 	0x15;

//Global Kiosk Variables
#define IDLE					0
#define BIKE_STATUS_REQUEST		1
#define BIKE_STATUS_RESPONSE	2
#define UNLOCK_BIKE				3
#define REPORT_FAULTY			4
uint8_t kiosk_main_state = IDLE;
////////////////////////////////
uint8_t bike_status;

//Function Prototypes
void mcp2515_init();
void reset_mcp2515();
uint8_t mcp2515_read(uint8_t address);
void mcp2515_write(uint8_t address, uint8_t data_out);
uint8_t mcp2515_status();
void clear_rxb0();
void clear_rxb1();
void clear_txb0();
void mcp2515_bit_modify(uint8_t address, uint8_t mask, uint8_t data);
void request_to_send(uint8_t buffer);
void send_CAN_message(uint8_t canid, uint8_t rack_num, uint8_t d0, uint8_t buffer);
bool receive_CAN_message(uint8_t canid);
uint8_t get_bike_status(uint8_t rack_num);

#define MCP2515_CS RPI_GPIO_P1_22

int main(int argc, char **argv)
{
    // If you call this, it will not actually access the GPIO
// Use for testing
//        bcm2835_set_debug(1);

      if (!bcm2835_init())
	return 1;
	
	bcm2835_gpio_fsel(MCP2515_CS, BCM2835_GPIO_FSEL_OUTP);

    bcm2835_spi_begin();
    bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);      	// The default
    bcm2835_spi_setDataMode(BCM2835_SPI_MODE0);                   	// The default
    bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_256);		//Set clock rate to 1MHz
    //bcm2835_spi_chipSelect(BCM2835_SPI_CS0);                      	// The default
    //bcm2835_spi_setChipSelectPolarity(BCM2835_SPI_CS0, LOW);      	// the default
    bcm2835_spi_chipSelect(BCM2835_SPI_CS_NONE);
    bcm2835_spi_setChipSelectPolarity(BCM2835_SPI_CS_NONE, LOW);

    // Send a byte to the slave and simultaneously read a byte back from the slave
    // If you tie MISO to MOSI, you should read back what was sent
    //uint8_t data = bcm2835_spi_transfer(0x23);
    //printf("Read from SPI: %02X\n", data);
    
    mcp2515_init();

    uint8_t data_in = mcp2515_read(canstat);
    printf("CAN Controller status: 0x%02X\n", data_in);
    
    
    while(1)
    {
		
		switch(kiosk_main_state)
		{
			default:
			case IDLE:
				printf("KIOSK STATE: IDLE\n"); 
				
				//kiosk_main_state = BIKE_STATUS_REQUEST;
				printf("bike_status: 0x%02X\n", get_bike_status(1));
			break;
			
			
			case BIKE_STATUS_REQUEST:
				printf("KIOSK STATE: BIKE_STATUS_REQUEST\n");
				//Send the Bike Status Request Message over CAN
				send_CAN_message(canid_bike_status_request, 1, 0x00, 0);
				kiosk_main_state = BIKE_STATUS_RESPONSE;
			break;
			
			
			case BIKE_STATUS_RESPONSE:
				printf("KIOSK STATE: BIKE_STATUS_RESPONSE\n");
			    //wait for a response
			    if((mcp2515_read(canintf) & 0x01) == 0x01)
			    {
					if(mcp2515_read(rxb0_sidl) >> 5 == 0x00)
					{
						if(mcp2515_read(rxb0_sidh) == canid_bike_status_response)
						{
							bike_status = mcp2515_read(rxb0_d0);
							printf("bike_status: 0x%02X\n", bike_status);
							kiosk_main_state = IDLE;
						}
					}
					//clear CANINTF -- we are done with the received message
					//mcp2515_write(canintf, 0x00);
					mcp2515_bit_modify(canintf, 0x03, 0x00);
					//clear RXB0
					clear_rxb0();
				}
			break;
			
			
			case UNLOCK_BIKE:
				printf("KIOSK STATE: UNLOCK_BIKE\n");
				//get the bike status -- rack 1 for now
				bike_status = get_bike_status(1);
				if(bike_status == 0)
				{
					printf("No Bikes Present...\n");
				}
				else 	//unlock first available bike in the line
				{
					if((bike_status & 0x01) == 0x01)		//unlock bike 1
					{
						send_CAN_message(canid_unlock_bike_request, 1, 0x01, 0);
					}
					else if((bike_status & 0x02) == 0x02)	//unlock bike 2
					{
						send_CAN_message(canid_unlock_bike_request, 1, 0x02, 0);
					}
					else if((bike_status & 0x04) == 0x04)	//unlock bike 3
					{
						send_CAN_message(canid_unlock_bike_request, 1, 0x04, 0);
					}
					else if((bike_status & 0x08) == 0x08)	//unlock bike 4
					{
						send_CAN_message(canid_unlock_bike_request, 1, 0x08, 0);
					}
					else if((bike_status & 0x10) == 0x10)	//unlock bike 5
					{
						send_CAN_message(canid_unlock_bike_request, 1, 0x10, 0);
					}
					
					//Wait for message to be sent and BLOCK
					while((mcp2515_read(canintf) & 0x04) != 0x04);
					
					//Once message is sent, reset the interrupt and clear TXB0
					mcp2515_bit_modify(canintf, 0x04, 0x00);
					clear_txb0();
					
					//BLOCK until the Bike Status Response is received
					while(!receive_CAN_message(canid_ack) && !receive_CAN_message(canid_nack));
					
					if(mcp2515_read(rxb0_sidh) == canid_ack)
					{
						printf("Bike Successfully Unlocked\n");
					}
					else
					{
						printf("Bike could not be Unlocked\n");
					}
					
					//clear CANINTF -- we are done with the received message
					mcp2515_bit_modify(canintf, 0x03, 0x00);
					
					//clear RXB0
					clear_rxb0();
				}
				kiosk_main_state = IDLE;
				
			break;
			
			
			case REPORT_FAULTY:
				printf("KIOSK STATE: REPORT_FAULTY\n");
			break;
		}
		
		//print diagnostics
		printf("canstat: 0x%02X\n", mcp2515_read(canstat));
        printf("status:  0x%02X\n", mcp2515_status());
        printf("canintf: 0x%02X\n\n", mcp2515_read(canintf));
        bcm2835_delay(5000);
	}


	//close SPI and BCM2835 Library before exiting
    bcm2835_spi_end();
    bcm2835_close();
    return 0;
}



///////////////////////////////////////////////////////////////////
////////////////////     MCP2515 FUNCTIONS     ////////////////////
///////////////////////////////////////////////////////////////////
void mcp2515_init()
{
    reset_mcp2515();
    mcp2515_write(caninte, 0xFF);		//Turn on Interrupt Enable
    mcp2515_write(canctrl, 0x00);		//req normal operation mode
    mcp2515_write(0x60, 0b01100000);	//turn off filtering on rxb0
}

void reset_mcp2515()
{
	bcm2835_gpio_write(MCP2515_CS, LOW);
	
    bcm2835_spi_transfer(reset_instruction);
    
    bcm2835_gpio_write(MCP2515_CS, HIGH);
    //char buf[1] = {reset_instruction};
    //bcm2835_spi_transfern(buf, 1);
    bcm2835_delay(100);
}

/////////////////////////////////////////////////////////////////////
//Reads the register contents at the specified address on the MCP2515
/////////////////////////////////////////////////////////////////////
uint8_t mcp2515_read(uint8_t address)
{
    //char buf[3] = {read_instruction, address, 0x00};
    bcm2835_gpio_write(MCP2515_CS, LOW);
    
    bcm2835_spi_transfer(read_instruction);
    bcm2835_spi_transfer(address);
    uint8_t data_in = bcm2835_spi_transfer(0x00);
    
    bcm2835_gpio_write(MCP2515_CS, HIGH);
    return data_in;
    //bcm2835_spi_transfern(buf, 3);
    //return buf[0];
}

/////////////////////////////////////////////////////////////////////
//Writes the data_out byte at the specified address on the MCP2515.
/////////////////////////////////////////////////////////////////////
void mcp2515_write(uint8_t address, uint8_t data_out)
{
    //char buf[3] = {write_instruction, address, data_out};
    bcm2835_gpio_write(MCP2515_CS, LOW);
    
    bcm2835_spi_transfer(write_instruction);
    bcm2835_spi_transfer(address);
    bcm2835_spi_transfer(data_out);
    
    bcm2835_gpio_write(MCP2515_CS, HIGH);
    //bcm2835_spi_transfern(buf, 3);
}

/////////////////////////////////////////////////////////////////////
//Returns a status byte from the CAN controller with this format:
//7 - CANINTF.TX2IF
//6 - TXB2CNTRL.TXREQ
//5 - CANINTF.TX1IF
//4 - TXB1CNTRL.TXREQ
//3 - CANINTF.TX0IF
//2 - TXB0CNTRL.TXREQ
//1 - CANINTF.RX1IF
//0 - CANINTF.RX0IF
/////////////////////////////////////////////////////////////////////
uint8_t mcp2515_status()
{
	bcm2835_gpio_write(MCP2515_CS, LOW);
	
    bcm2835_spi_transfer(read_status_instruction);
    uint8_t data_in = bcm2835_spi_transfer(0x00);
    
    bcm2835_gpio_write(MCP2515_CS, HIGH);
    return data_in;
}

void clear_rxb0()
{
	mcp2515_write(0x61, 0x00);	//RXB0SIDH
	mcp2515_write(0x62, 0x00);	//RXB0SIDL
	mcp2515_write(0x63, 0x00);	//RXB0EID8
	mcp2515_write(0x64, 0x00);	//RXB0EID0
	mcp2515_write(0x65, 0x00);	//RXB0DLC
	mcp2515_write(0x66, 0x00);	//RXB0D0
	mcp2515_write(0x67, 0x00);	//RXB0D1
	mcp2515_write(0x68, 0x00);	//RXB0D2
	mcp2515_write(0x69, 0x00);	//RXB0D3
	mcp2515_write(0x6A, 0x00);	//RXB0D4
	mcp2515_write(0x6B, 0x00);	//RXB0D5
	mcp2515_write(0x6C, 0x00);	//RXB0D6
	mcp2515_write(0x6D, 0x00);	//RXB0D7
}

void clear_rxb1()
{
	mcp2515_write(0x71, 0x00);	//RXB1SIDH
	mcp2515_write(0x72, 0x00);	//RXB1SIDL
	mcp2515_write(0x73, 0x00);	//RXB1EID8
	mcp2515_write(0x74, 0x00);	//RXB1EID0
	mcp2515_write(0x75, 0x00);	//RXB1DLC
	mcp2515_write(0x76, 0x00);	//RXB1D0
	mcp2515_write(0x77, 0x00);	//RXB1D1
	mcp2515_write(0x78, 0x00);	//RXB1D2
	mcp2515_write(0x79, 0x00);	//RXB1D3
	mcp2515_write(0x7A, 0x00);	//RXB1D4
	mcp2515_write(0x7B, 0x00);	//RXB1D5
	mcp2515_write(0x7C, 0x00);	//RXB1D6
	mcp2515_write(0x7D, 0x00);	//RXB1D7
}

void clear_txb0()
{
	mcp2515_write(0x31, 0x00);	//TXB0SIDH
	mcp2515_write(0x32, 0x00);	//TXB0SIDL
	mcp2515_write(0x33, 0x00);	//TXB0EID8
	mcp2515_write(0x34, 0x00);	//TXB0EID0
	mcp2515_write(0x35, 0x00);	//TXB0DLC
	mcp2515_write(0x36, 0x00);	//TXB0D0
	mcp2515_write(0x37, 0x00);	//TXB0D1
	mcp2515_write(0x38, 0x00);	//TXB0D2
	mcp2515_write(0x39, 0x00);	//TXB0D3
	mcp2515_write(0x3A, 0x00);	//TXB0D4
	mcp2515_write(0x3B, 0x00);	//TXB0D5
	mcp2515_write(0x3C, 0x00);	//TXB0D6
	mcp2515_write(0x3D, 0x00);	//TXB0D7
}

void mcp2515_bit_modify(uint8_t address, uint8_t mask, uint8_t data)
{
	bcm2835_gpio_write(MCP2515_CS, LOW);
	
	bcm2835_spi_transfer(bit_modify_instruction);
	bcm2835_spi_transfer(address);
	bcm2835_spi_transfer(mask);
	bcm2835_spi_transfer(data);
	
	bcm2835_gpio_write(MCP2515_CS, HIGH);
}

////////////////////////////////////////////////////////////////////
//Utility function for the send_CAN_message function. This function
//is what sends the actual request to the MCP2515 to flush the buffer
//data out onto the CAN line.
////////////////////////////////////////////////////////////////////
void request_to_send(uint8_t buffer)
{
	bcm2835_gpio_write(MCP2515_CS, LOW);
	
    if(buffer == 0)
    {
		bcm2835_spi_transfer(0b10000001);
    }
    else if(buffer == 1)
    {
		bcm2835_spi_transfer(0b10000010);
    }
    else if(buffer == 2)
    {
		bcm2835_spi_transfer(0b10000100);
    }
    else
    {
		bcm2835_spi_transfer(0x00);
	}
	
    bcm2835_gpio_write(MCP2515_CS, HIGH);
}

///////////////////////////////////////////////////////////////////
//This is the standard function to put a CAN message on the bus.
//Specify a CAN ID (i.e. bike status request, unlock, etc.), a byte
//of data, and which buffer it should be placed in. The function
//takes care of the request to send.
///////////////////////////////////////////////////////////////////
void send_CAN_message(uint8_t canid, uint8_t rack_num, uint8_t d0, uint8_t buffer)
{
    switch(buffer)
    {
	default:
	case 0:				//Transmit Buffer 1
	    mcp2515_write(0x31, canid);				//pack the SID_H register
	    mcp2515_write(0x32, rack_num << 5);		//pack the SID_L register
	    mcp2515_write(0x35, 0x01);				//pack the DLC register
	    mcp2515_write(0x36, d0);				//pack the Data0 register
	    request_to_send(0);
	break;

	case 1:				//Transmit Buffer 2
	    mcp2515_write(0x41, canid);				//pack the SID_H register
	    mcp2515_write(0x32, rack_num << 5);		//pack the SID_L register
	    mcp2515_write(0x45, 0x01);				//pack the DLC register
	    mcp2515_write(0x46, d0);				//pack the Data0 register
	    request_to_send(1);
	break;

	case 2:				//Transmit Buffer 3
	    mcp2515_write(0x51, canid);				//pack the SID_H register
	    mcp2515_write(0x32, rack_num << 5);		//pack the SID_L register
	    mcp2515_write(0x55, 0x01);				//pack the DLC register
	    mcp2515_write(0x56, d0);				//pack the Data0 register
	    request_to_send(2);
	break;
    }
}

//This non-blocking function can be called anytime a CAN message is expected.
//When the desired message is detected, it will return true. Otherwise, it 
//will return false.
bool receive_CAN_message(uint8_t canid)
{
	if((mcp2515_read(canintf) & 0x01) == 0x01)		//message received in RXB0
	{
		if(mcp2515_read(rxb0_sidl) >> 5 == 0x00)	//message has kiosk ID code
		{
			if(mcp2515_read(rxb0_sidh) == canid)	//message has desired CAN ID
			{
				return true;
			}
		}
	}
	return false;
}

uint8_t get_bike_status(uint8_t rack_num)
{
	uint8_t bike_status;
	
	//send a bike status request to desired rack on TXB0
	//Data Byte is zero because bike status requests do not contain data
	send_CAN_message(canid_bike_status_request, rack_num, 0, 0);
	
	//Wait for message to be sent and BLOCK
	while((mcp2515_read(canintf) & 0x04) != 0x04);
	
	//Once message is sent, reset the interrupt and clear TXB0
	mcp2515_bit_modify(canintf, 0x04, 0x00);
	clear_txb0();
	
	//BLOCK until the Bike Status Response is received
	while(!receive_CAN_message(canid_bike_status_response));
	
	//Store the bike status
	bike_status = mcp2515_read(rxb0_d0);
	
	//clear CANINTF -- we are done with the received message
	mcp2515_bit_modify(canintf, 0x03, 0x00);
	
	//clear RXB0
	clear_rxb0();
	
	//Return the Data Byte containing the bike status
	return bike_status;
}



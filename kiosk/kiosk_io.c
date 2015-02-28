//ABRS Team 1
//Kiosk input output control
//Kiosk Controller
//Authored by Adam Schin @2:17pm 2/17/2015
/////////////////////////////////////////

// build with gcc -o name name.c -l bcm2835

#include <bcm2835.h>
#include <stdio.h>

////////////////////////////////
//	INPUTS
//Inptut on RPI pin GPIO 7 (GPIO04)
#define PININ1 RPI_GPIO_P1_10
#define PININ2 RPI_GPIO_P1_11
#define PININ3 RPI_GPIO_P1_26
#define PININ4 RPI_GPIO_P1_15

//	OUTPUTS
//Output on RPI pin 11 (which is GPIO pin 17)
#define PINOUT1 RPI_GPIO_P1_12
#define PINOUT2 RPI_GPIO_P1_16
#define PINOUT3 RPI_GPIO_P1_18
#define PINOUT4 RPI_GPIO_P1_24

//	Global Variables
//
uint8_t cur_col = 1;
uint8_t cur_state = 1;
uint8_t num_bikes = 1;


//Function Prototypes
uint8_t printnumber(uint8_t);
void holdpress(uint8_t);
void changeCol();
uint8_t detectPress();
void handleState(uint8_t);
void handleIdle(uint8_t);
void handleSellect(uint8_t);
void handleConfirmation(uint8_t);



////////////////////////////////////////////////////////////
////////////////////      MAIN          ////////////////////
////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{
	
    if(!bcm2835_init())
	return 1;

    //set pin to input
    bcm2835_gpio_fsel(PININ1, BCM2835_GPIO_FSEL_INPT);
    bcm2835_gpio_fsel(PININ2, BCM2835_GPIO_FSEL_INPT);
    bcm2835_gpio_fsel(PININ3, BCM2835_GPIO_FSEL_INPT);
    bcm2835_gpio_fsel(PININ4, BCM2835_GPIO_FSEL_INPT);
    //with a pullup
    bcm2835_gpio_set_pud(PININ1, BCM2835_GPIO_PUD_DOWN);
    bcm2835_gpio_set_pud(PININ2, BCM2835_GPIO_PUD_DOWN);
    bcm2835_gpio_set_pud(PININ3, BCM2835_GPIO_PUD_DOWN);
    bcm2835_gpio_set_pud(PININ4, BCM2835_GPIO_PUD_DOWN);

    //set pin to output
    bcm2835_gpio_fsel(PINOUT1, BCM2835_GPIO_FSEL_OUTP);
    bcm2835_gpio_fsel(PINOUT2, BCM2835_GPIO_FSEL_OUTP);
    bcm2835_gpio_fsel(PINOUT3, BCM2835_GPIO_FSEL_OUTP);
    bcm2835_gpio_fsel(PINOUT4, BCM2835_GPIO_FSEL_OUTP);
    
    while(1)
    {
	changeCol();

	handleState(detectPress());
    }
    bcm2835_close();
    return 0;
}



////////////////////////////////////////////////////////////
////////////////////      FUNCTIONS     ////////////////////
////////////////////////////////////////////////////////////
uint8_t printnumber(uint8_t n)
{
	uint8_t pressed = 0;
    	// n is the row the was pressed
	// cur_col is the current column that is high
	if(n == 1)
	{
		if(cur_col == 1)
			pressed = 1;
		else if(cur_col == 2)
			pressed = 4;
		else if(cur_col == 3)
			pressed = 7;
		else
			pressed = 10;
	}
	else if(n == 2)
	{
		if(cur_col == 1)
			pressed = 2;
		else if(cur_col == 2)
			pressed = 5;
		else if(cur_col == 3)
			pressed = 8;
		else
			pressed = 0;
	}
	else if(n == 3)
	{
		if(cur_col == 1)
			pressed = 3;
		else if(cur_col == 2)
			pressed = 6;
		else if(cur_col == 3)
			pressed = 9;
		else
			pressed = 11;
	}
	else
	{
		if(cur_col == 1)
			pressed = 12;
		else if(cur_col == 2)
			pressed = 13;
		else if(cur_col == 3)
			pressed = 14;
		else
			pressed = 15;
	}
	printf("%d pressed\n", pressed);
	holdpress(n);
	return pressed;
}

void holdpress(uint8_t n)
{
	bcm2835_delay(100);
	if(n == 1){
		while(bcm2835_gpio_lev(PININ1))
			bcm2835_delay(10);
	}
	else if(n == 2){
		while(bcm2835_gpio_lev(PININ2))
			bcm2835_delay(10);
	}
	else if(n == 3){
		while(bcm2835_gpio_lev(PININ3))
			bcm2835_delay(10);
	}
	else if(n == 4){
		while(bcm2835_gpio_lev(PININ4))
			bcm2835_delay(10);
	}
	bcm2835_delay(100);
		
}

void changeCol()
{
	if(cur_col == 1)
	{
		bcm2835_gpio_write(PINOUT1, LOW);
		bcm2835_gpio_write(PINOUT2, HIGH);
		cur_col = 2;
		bcm2835_delay(10);
	}
	else if(cur_col == 2)
	{
		bcm2835_gpio_write(PINOUT2, LOW);
		bcm2835_gpio_write(PINOUT3, HIGH);
		cur_col = 3;
		bcm2835_delay(10);
	}
	else if(cur_col == 3)
	{
		bcm2835_gpio_write(PINOUT3, LOW);
		bcm2835_gpio_write(PINOUT4, HIGH);
		cur_col = 4;
		bcm2835_delay(10);
	}
	else
	{
		bcm2835_gpio_write(PINOUT4, LOW);
		bcm2835_gpio_write(PINOUT1, HIGH);
		cur_col = 1;
		bcm2835_delay(10);
	}

}

uint8_t detectPress()
{
	uint8_t val1 = bcm2835_gpio_lev(PININ1);
	uint8_t val2 = bcm2835_gpio_lev(PININ2);
	uint8_t val3 = bcm2835_gpio_lev(PININ3);
	uint8_t val4 = bcm2835_gpio_lev(PININ4);
	if(val1){
		return printnumber(1);
	}
	else if(val2){
		return printnumber(2);
	}
	else if(val3){
		return printnumber(3);
	}
	else if(val4){
		return printnumber(4);
	}
	return 16;	
}


void handleState(uint8_t s)
{
	if(s != 16)
	{
		if(cur_state == 1)//idle state
			handleIdle(s);
		else if(cur_state == 2)//Bike sellect
			handleSellect(s);
		else if(cur_state == 3)//confirmation
			handleConfirmation(s);
	}
}

void handleIdle(uint8_t s)
{
	if(s == 11 || s == 14)
	{
		printf("How many bikes would you like to rent?\n");
		num_bikes = 1;
		cur_state = 2;
	}
}

void handleSellect(uint8_t s)
{
	if(s == 10 || s == 15)
	{
		printf("Transaction Canceled\n");
		cur_state = 1;
	}
	else if(s == 11 || s == 14)
	{
		printf("You have selected %d bikes\n", num_bikes);
		cur_state = 3;
	}
	else if(s == 12)
	{
		num_bikes += 1;
		printf("%d Bikes\n", num_bikes);
	}
	else if(s == 13)
	{
		if(num_bikes !=1)
			num_bikes -= 1;
		printf("%d Bikes\n", num_bikes);
	}
	else if(s != 0)
	{
		num_bikes = s;
		printf("%d Bikes\n", num_bikes);
	}
}

void handleConfirmation(uint8_t s)
{
	if(s == 10 || s == 15)
	{
		printf("How many bikes would you like to rent?\n");
		cur_state = 2;
	}
	if(s == 11 || s == 14)
	{
		printf("Thank You\n");
		cur_state = 1;
	}
}
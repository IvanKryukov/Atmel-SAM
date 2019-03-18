/*
 * SAMD21_GCCInterrupt_Proj.c
 *
 * Created: 18.12.2018 15:27:49
 * Author : User
 */ 
#include "sam.h"

typedef enum
{
	INTERNAL_SRC = 0,
	EXTERNAL_8M = 1,
	EXTERNAL_32K = 2,
} TIMER_typedef;

/***** SPI DEFINITIONS *****/
#define SPI_CLK_FREQ 8000000
#define SPI_BAUD 50000
#define GPIO_GROUP_SS  0         // PORT group of PA13 (PORTA = PORT group 0)
#define GPIO_MAP_SS    PORT_PA13 // PA13 bit position macro (1<<13)
#define SPI_SERCOM SERCOM5
#define SPI_SERCOM_PINS_PORT_GROUP 1 // (PORTB = PORT group 1)
#define SPI_SERCOM_SCK_PIN		PORT_PB23
#define SPI_SERCOM_MOSI_PIN		PORT_PB22
#define SPI_SERCOM_MISO_PIN		PORT_PB02

#define PORTA		0
#define PORTB		1

void Clock_Init(void)
{
	SYSCTRL->OSC8M.bit.PRESC = 0;                          // no prescaler (is 8 on reset)
	SYSCTRL->OSC8M.reg |= 1 << SYSCTRL_OSC8M_ENABLE_Pos;   // enable source

	GCLK->GENDIV.bit.ID = 0x01;                            // select GCLK_GEN[1]
	GCLK->GENDIV.bit.DIV = 0;                              // no prescaler

	GCLK->GENCTRL.bit.ID = 0x01;                           // select GCLK_GEN[1]
	GCLK->GENCTRL.reg |= GCLK_GENCTRL_SRC_OSC8M;           // OSC8M source
	GCLK->GENCTRL.bit.GENEN = 1;                           // enable generator

	GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID_SERCOM0_CORE;      // SERCOM0 peripheral channel
	GCLK->CLKCTRL.reg |= GCLK_CLKCTRL_GEN_GCLK1;           // select source GCLK_GEN[1]
	GCLK->CLKCTRL.bit.CLKEN = 1;                           // enable generic clock

	PM->APBCSEL.bit.APBCDIV = 0;                           // no prescaler
	PM->APBCMASK.bit.SERCOM0_ = 1;                         // enable SERCOM0 interface
}
/*
void Timer_Init(void)
{
	// Configure asynchronous clock source
	GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID_TCC2_TC3_Val;   // select TC3 peripheral channel
	GCLK->CLKCTRL.reg |= GCLK_CLKCTRL_GEN_GCLK0;		// select source GCLK_GEN[0]
	GCLK->CLKCTRL.bit.CLKEN = 1;						// enable TC3 generic clock

	// Configure synchronous bus clock
	PM->APBCSEL.bit.APBCDIV = 0;			// no prescaler
	PM->APBCMASK.bit.TC3_ = 1;				// enable TC3 interface
	TC3->COUNT16.CTRLA.bit.MODE = 0x0;		// Configure Count Mode (16-bit)
	TC3->COUNT16.CTRLA.bit.PRESCALER = 0x1;	// Configure Prescaler for divide by 2 (500kHz clock to COUNT)
	TC3->COUNT16.CTRLA.bit.WAVEGEN = 0x1;	// Configure TC3 Compare Mode for compare channel 0 // "Match Frequency" operation
	TC3->COUNT16.CC[0].reg = 50000;			// Initialize compare value for 100mS @ 500kHz
	TC3->COUNT16.INTENSET.bit.MC0 = 0x1;	// Enable TC3 compare mode interrupt generation // Enable match interrupts on compare channel 0
	TC3->COUNT16.CTRLA.bit.ENABLE = 1;		// Enable TC3
	while(TC3->COUNT16.STATUS.bit.SYNCBUSY == 1);	// Wait until TC3 is enabled
	
	
	NVIC_SetPriority(TC3_IRQn, 3);		//Set TC3 Interrupt Priority to Level 3	
	NVIC_EnableIRQ(TC3_IRQn);			// Enable TC3 NVIC Interrupt Line
}

void USART_Init(void)
{
	
}

void USART_Transmit(void)
{
	
}

void USART_Receive(void)
{
	
}


void SPI_Init(void)
{	
	PORT->Group[SPI_SERCOM_PINS_PORT_GROUP].WRCONFIG.reg=
		PORT_WRCONFIG_WRPINCFG |										// Enables the configuration of PINCFG
		PORT_WRCONFIG_WRPMUX |											// Enables the configuration of the PMUX for the selected pins
		PORT_WRCONFIG_PMUX(MUX_PB02D_SERCOM5_PAD0) |					// Bulk configuration for PMUX "C" for SERCOM5
		PORT_WRCONFIG_PMUXEN |											// Enables the PMUX for the pins
		PORT_WRCONFIG_HWSEL |											// Select the correct pin configurations for 16-31
		PORT_WRCONFIG_INEN |											// Enable input
		PORT_WRCONFIG_PINMASK((uint16_t)((SPI_SERCOM_MISO_PIN) >> 2));	// Selecting which pin is configured // This pin needs to shift to fit the 16 bit macro requirements
	
	PORT->Group[SPI_SERCOM_PINS_PORT_GROUP].WRCONFIG.reg=				
		PORT_WRCONFIG_WRPINCFG |										// Enables the configuration of PINCFG
		PORT_WRCONFIG_WRPMUX |											// Enables the configuration of the PMUX for the selected pins
		PORT_WRCONFIG_PMUX(MUX_PB22D_SERCOM5_PAD2) |					// Bulk configuration for PMUX
		PORT_WRCONFIG_PMUXEN |											// Enables the PMUX for the pins
		PORT_WRCONFIG_HWSEL |											// Select the correct pin configurations for 16-31
		PORT_WRCONFIG_PINMASK((uint16_t)((SPI_SERCOM_MOSI_PIN | SPI_SERCOM_SCK_PIN) >> 16));	// Selecting which pin is configured
	
	
	PORT->Group[GPIO_GROUP_SS].OUTSET.reg = GPIO_MAP_SS;	// Pre-setting Slave Select (SS) high
	PORT->Group[GPIO_GROUP_SS].DIRSET.reg = GPIO_MAP_SS;	// Setting SPI Slave Select as an output
	
	PM->APBCMASK.reg |= PM_APBCMASK_SERCOM5; //Enable the SERCOM5 under the PM
	
	GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(SERCOM5_GCLK_ID_CORE) |	GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN(0); // Generic Clock to the SERCOM Peripheral Using GCLK
	
	// Configure SERCOM5 as SPI Master	
	SPI_SERCOM->SPI.CTRLA.bit.ENABLE = 0;	// Disable the SERCOM SPI module	
	SPI_SERCOM->SPI.CTRLA.reg = SERCOM_SPI_CTRLA_MODE_SPI_MASTER | SERCOM_SPI_CTRLA_DOPO(1);	// DOPO is set to PAD[2,3]
	uint16_t BAUD_REG = ((float)SPI_CLK_FREQ / (float)(2 * SPI_BAUD)) - 1;	// Calculate BAUD value	
	SPI_SERCOM->SPI.BAUD.reg = SERCOM_SPI_BAUD_BAUD(BAUD_REG);				// Set the SPI baud rate
	SPI_SERCOM->SPI.CTRLA.reg |= SERCOM_SPI_CTRLA_ENABLE;					// Enable the Sercom SPI
	
	// What for synchronization of SERCOM SPI registers between the clock domains
	while(SPI_SERCOM->SPI.SYNCBUSY.bit.ENABLE);
}

void SPI_Transmit(uint8_t write_data)
{
	write_data = SPI_SERCOM->SPI.DATA.reg;
}

uint8_t SPI_Receive(void)
{
	uint8_t write_data;
	write_data = SPI_SERCOM->SPI.DATA.reg;
	while(SPI_SERCOM->SPI.SYNCBUSY.bit.ENABLE);
	return write_data;
}
*/
void GPIO_Init(void)
{
	//	Configure ports as outputs
	PORT->Group[PORTA].DIRSET.reg |= PORT_PA08 | PORT_PA17 | PORT_PA27;	// set pins PA27, PA08 as OUTPUT
	PORT->Group[PORTB].DIRSET.reg |= PORT_PB03;	// set pin PB03 as OUTPUT
	
	PORT->Group[PORTA].OUTCLR.reg |= PORT_PA08 | PORT_PA17 | PORT_PA27;	// set pins PA27, PA08 to LOW
	PORT->Group[PORTB].OUTCLR.reg |= PORT_PB03;	// set pin PB03 to LOW
}

void EIC_Init(void)
{
	// Configure interrupt port (D8)
	PORT->Group[PORTA].WRCONFIG.reg=
	PORT_WRCONFIG_WRPINCFG	|										// Enables the configuration of PINCFG
	PORT_WRCONFIG_WRPMUX	|										// Enables the configuration of the PMUX for the selected pins
	PORT_WRCONFIG_PMUX(MUX_PA06A_EIC_EXTINT6) |						// Bulk configuration for PMUX EIC for PA06
	PORT_WRCONFIG_PMUXEN	|											// Enables the PMUX for the pins
	PORT_WRCONFIG_HWSEL		|											// Select the correct pin configurations for 16-31
	PORT_WRCONFIG_PINMASK(PORT_PA06)|
	PORT_WRCONFIG_INEN;											// Enable input
	
	PM->APBAMASK.bit.EIC_ = 1;
	//PM->APBAMASK.reg |= PM_APBAMASK_EIC;
	GCLK->CLKCTRL.reg |= GCLK_CLKCTRL_ID(EIC_GCLK_ID) |	GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN(0);
	
	EIC->CONFIG[0].reg |= EIC_CONFIG_SENSE6_FALL | EIC_CONFIG_FILTEN6;
	EIC->INTENSET.reg |= EIC_INTENSET_EXTINT6;
	
	EIC->CTRL.reg |= EIC_CTRL_ENABLE;	// Enable EIC
	while (EIC->STATUS.bit.SYNCBUSY == 1);
	
	NVIC_SetPriority(EIC_IRQn, 1);		//Set TC3 Interrupt Priority to Level 3	
	NVIC_EnableIRQ(EIC_IRQn);
}

void TC3_Handler(void)
{
	TC3->COUNT16.INTFLAG.reg |= 0b00010000;	// Acknowledge the interrupt (clear MC0 interrupt flag to re-arm)
}

uint8_t tgl = 0;
void EIC_Handler(void)
{	
	tgl++;
	if (tgl % 2 == 0) PORT->Group[PORTB].OUTCLR.reg |= PORT_PB03;
	else PORT->Group[PORTB].OUTSET.reg |= PORT_PB03;
	
	if (tgl > 254) tgl = 0;
	EIC->INTFLAG.reg |= EIC_INTFLAG_EXTINT6;	// clear interrupt flag
}

int main(void)
{
    SystemInit();	
	GPIO_Init();
	EIC_Init();
	
	__enable_irq(); // Enable Interrupts 

    while (1) 
    {
		PORT->Group[PORTA].OUTSET.reg |= PORT_PA17;	// set pin PA17 to LOW
		PORT->Group[PORTA].OUTSET.reg |= PORT_PA08;	// set pin PA08 to LOW
		PORT->Group[PORTA].OUTSET.reg |= PORT_PA27;	// set pin PA27 to HIGH	(TX LED)
		//PORT->Group[PORTB].OUTCLR.reg |= PORT_PB03;	// set pin PB03 to LOW	(RX LED)
		for (int i = 0; i < 50000; i++);
		PORT->Group[PORTA].OUTCLR.reg |= PORT_PA17;	// set pin PA17 to LOW
		PORT->Group[PORTA].OUTCLR.reg |= PORT_PA08;	// set pin PA08 to HIGH
		PORT->Group[PORTA].OUTCLR.reg |= PORT_PA27;	// set pin PA27 to LOW	(TX LED)
		//PORT->Group[PORTB].OUTSET.reg |= PORT_PB03;	// set pin PB03 to HIGH	(RX LED)
		for (int i = 0; i < 50000; i++);
    }
}

#include "stm8.h"
#include <string.h>
#define SET(x, y)   (x) |= (y)
#define UNSET(x, y) (x) &= ~(y)
#define READ(x, y)  ((x) & (y))
#define BMP180_ADDR	0x77
#define BH1750_ADDR	0x23
#define MCP4725_ADDR	0x62
#define SI7021_ADDR	0x40
#define HMC5883L_ADDR	0x1e
#define I2C_READ        1
#define I2C_WRITE       0
#define OSS		3
#define PC3		3
#define PC4		4
#define CSN		3
#define CE		4
#define BH1750_HRES2_ONETIME	0x23
#define BH1750_POWERON		0x01
#define MCP4725_CMD_WRITEDAC	0x40
#define SI7021_READ_HUMIDITY	0xe5
#define HMC5883L_CRA		0x00
#define HMC5883L_CRB		0x01
#define HMC5883L_MODE		0x02
typedef unsigned char UCHAR;
void delayTenMicro (void) {
   char a;
   for (a = 0; a < 50; ++a)
      __asm__("nop");
}
UCHAR write_spi (UCHAR value) {
   UCHAR ret;
   delayTenMicro ();
   SPI_DR = value;
   delayTenMicro ();
   while ((SPI_SR & TXE) == 0);
   delayTenMicro ();
   while ((SPI_SR & RXNE) == 0);
   delayTenMicro ();
   ret = SPI_DR;
   return (ret);
}
UCHAR write_spi_reg (UCHAR reg, UCHAR value) {
   UCHAR ret;
   PC_ODR &= ~(1 << CSN);
   ret = write_spi (reg);
   if (reg != NOP && reg != FLUSH_RX && reg != FLUSH_TX)
      write_spi (value);
   else
      delayTenMicro ();
   PC_ODR |= (1 << CSN);
   return (ret);
}
UCHAR read_spi_reg (UCHAR reg) {
   UCHAR ret;
   PC_ODR &= ~(1 << CSN);
   ret = write_spi (reg);
   if (reg != NOP && reg != FLUSH_RX && reg != FLUSH_TX)
      ret = write_spi (NOP);
   else
      delayTenMicro ();
   PC_ODR |= (1 << CSN);
   return (ret);
}
UCHAR write_spi_buf (UCHAR reg, UCHAR *array, UCHAR len) {
   UCHAR ret, n;
   PC_ODR &= ~(1 << CSN);
   ret = write_spi (reg);
   for (n = 0; n < len; ++n)
      write_spi (array[n]);
   PC_ODR |= (1 << CSN);
   return (ret);
}
UCHAR read_spi_buf (UCHAR reg, UCHAR *array, UCHAR len) {
   UCHAR ret, n;
   PC_ODR &= ~(1 << CSN);
   ret = write_spi (reg);
   for (n = 0; n < len; ++n)
      array[n] = write_spi (NOP);
   PC_ODR |= (1 << CSN);
   return (ret);
}
void InitializeSPI () {
   SPI_CR1 = MSBFIRST | SPI_ENABLE | BR_DIV256 | MASTER | CPOL0 | CPHA0;
   SPI_CR2 = BDM_2LINE | CRCEN_OFF | CRCNEXT_TXBUF | FULL_DUPLEX | SSM_DISABLE;
   SPI_ICR = TXIE_MASKED | RXIE_MASKED | ERRIE_MASKED | WKIE_MASKED;
   PC_DDR = (1 << PC3) | (1 << PC4); // output mode
   PC_CR1 = (1 << PC3) | (1 << PC4); // push-pull
   PC_CR2 = (1 << PC3) | (1 << PC4); // up to 10MHz speed
   PC_ODR != (1 << CSN);
   PC_ODR &= ~(1 << CE);
}
void InitializeSystemClock() {
   CLK_ICKR = 0;                       //  Reset the Internal Clock Register.
   CLK_ICKR = CLK_HSIEN;               //  Enable the HSI.
   CLK_ECKR = 0;                       //  Disable the external clock.
   while ((CLK_ICKR & CLK_HSIRDY) == 0);       //  Wait for the HSI to be ready for use.
   CLK_CKDIVR = 0;                     //  Ensure the clocks are running at full speed.
   CLK_PCKENR1 = 0xff;                 //  Enable all peripheral clocks.
   CLK_PCKENR2 = 0xff;                 //  Ditto.
   CLK_CCOR = 0;                       //  Turn off CCO.
   CLK_HSITRIMR = 0;                   //  Turn off any HSIU trimming.
   CLK_SWIMCCR = 0;                    //  Set SWIM to run at clock / 2.
   CLK_SWR = 0xe1;                     //  Use HSI as the clock source.
   CLK_SWCR = 0;                       //  Reset the clock switch control register.
   CLK_SWCR = CLK_SWEN;                //  Enable switching.
   while ((CLK_SWCR & CLK_SWBSY) != 0);        //  Pause while the clock switch is busy.
}
void delay (int time_ms) {
   volatile long int x;
   for (x = 0; x < 1036*time_ms; ++x)
      __asm__("nop");
}
void i2c_read (unsigned char *x) {
   while ((I2C_SR1 & I2C_RXNE) == 0);
   *x = I2C_DR;
}
void i2c_set_nak (void) {
   I2C_CR2 &= ~I2C_ACK;
}
void i2c_set_stop (void) {
   I2C_CR2 |= I2C_STOP;
}
void i2c_send_reg (UCHAR addr) {
   volatile int reg;
   reg = I2C_SR1;
   reg = I2C_SR3;
   I2C_DR = addr;
   while ((I2C_SR1 & I2C_TXE) == 0);
}
void i2c_send_address (UCHAR addr, UCHAR mode) {
   volatile int reg;
   reg = I2C_SR1;
   I2C_DR = (addr << 1) | mode;
   if (mode == I2C_READ) {
      I2C_OARL = 0;
      I2C_OARH = 0;
   }
   while ((I2C_SR1 & I2C_ADDR) == 0);
   if (mode == I2C_READ)
      UNSET (I2C_SR1, I2C_ADDR);
}
void i2c_set_start_ack (void) {
   I2C_CR2 = I2C_ACK | I2C_START;
   while ((I2C_SR1 & I2C_SB) == 0);
}
//
//  Send a message to the debug port (UART1).
//
void UARTPrintF (char *message) {
   char *ch = message;
   while (*ch) {
      UART1_DR = (unsigned char) *ch;     //  Put the next character into the data transmission register.
      while ((UART1_SR & SR_TXE) == 0);   //  Wait for transmission to complete.
      ch++;                               //  Grab the next character.
   }
}
void print_byte_hex (unsigned char buffer) {
   unsigned char message[8];
   int a, b;
   a = (buffer >> 4);
   if (a > 9)
      a = a + 'a' - 10;
   else
      a += '0'; 
   b = buffer & 0x0f;
   if (b > 9)
      b = b + 'a' - 10;
   else
      b += '0'; 
   message[0] = a;
   message[1] = b;
   message[2] = 0;
   UARTPrintF (message);
}
unsigned char i2c_read_register (UCHAR addr, UCHAR rg) {
   volatile UCHAR reg;
   UCHAR x;
   i2c_set_start_ack ();
   i2c_send_address (addr, I2C_WRITE);
   i2c_send_reg (rg);
   i2c_set_start_ack ();
   i2c_send_address (addr, I2C_READ);
   reg = I2C_SR1;
   reg = I2C_SR3;
   i2c_set_nak ();
   i2c_set_stop ();
   i2c_read (&x);
   return (x);
}
   
void InitializeI2C (void) {
   I2C_CR1 = 0;   //  Disable I2C before configuration starts. PE bit is bit 0
   //
   //  Setup the clock information.
   //
   I2C_FREQR = 16;                     //  Set the internal clock frequency (MHz).
   UNSET (I2C_CCRH, I2C_FS);           //  I2C running is standard mode.
   I2C_CCRL = 0x10;                    //  SCL clock speed is 500 kHz.
   I2C_CCRH &= 0xf0;	// Clears lower 4 bits "CCR"
   //
   //  Set the address of this device.
   //
   UNSET (I2C_OARH, I2C_ADDMODE);      //  7 bit address mode.
   SET (I2C_OARH, I2C_ADDCONF);        //  Docs say this must always be 1.
   //
   //  Setup the bus characteristics.
   //
   I2C_TRISER = 17;
   //
   //  Turn on the interrupts.
   //
   //I2C_ITR = I2C_ITBUFEN | I2C_ITEVTEN | I2C_ITERREN; //  Buffer, event and error interrupts enabled
   //
   //  Configuration complete so turn the peripheral on.
   //
   I2C_CR1 = I2C_PE;	// Enables port
   //
   //  Enter master mode.
   //
}
   
void InitializeUART() {
    //
    //  Clear the Idle Line Detected bit in the status register by a read
    //  to the UART1_SR register followed by a Read to the UART1_DR register.
    //
    unsigned char tmp = UART1_SR;
    tmp = UART1_DR;
    //
    //  Reset the UART registers to the reset values.
    //
    UART1_CR1 = 0;
    UART1_CR2 = 0;
    UART1_CR4 = 0;
    UART1_CR3 = 0;
    UART1_CR5 = 0;
    UART1_GTR = 0;
    UART1_PSCR = 0;
    //
    //  Now setup the port to 115200,n,8,1.
    //
    UNSET (UART1_CR1, CR1_M);        //  8 Data bits.
    UNSET (UART1_CR1, CR1_PCEN);     //  Disable parity.
    UNSET (UART1_CR3, CR3_STOPH);    //  1 stop bit.
    UNSET (UART1_CR3, CR3_STOPL);    //  1 stop bit.
    UART1_BRR2 = 0x0a;      //  Set the baud rate registers to 115200 baud
    UART1_BRR1 = 0x08;      //  based upon a 16 MHz system clock.
    //
    //  Disable the transmitter and receiver.
    //
    UNSET (UART1_CR2, CR2_TEN);      //  Disable transmit.
    UNSET (UART1_CR2, CR2_REN);      //  Disable receive.
    //
    //  Set the clock polarity, lock phase and last bit clock pulse.
    //
    SET (UART1_CR3, CR3_CPOL);
    SET (UART1_CR3, CR3_CPHA);
    SET (UART1_CR3, CR3_LBCL);
    //
    //  Turn on the UART transmit, receive and the UART clock.
    //
    SET (UART1_CR2, CR2_TEN);
    SET (UART1_CR2, CR2_REN);
    UART1_CR3 = CR3_CLKEN;
}

int main () {
   short voltage = 1900;
   UCHAR x, a;
   UCHAR rx_addr_p1[]  = { 0xd2, 0xf0, 0xf0, 0xf0, 0xf0 };
   UCHAR tx_addr[]     = { 0xe1, 0xf0, 0xf0, 0xf0, 0xf0 };
   UCHAR tx_payload[33];
   volatile int reg, x1, y1, z1;
   InitializeSystemClock();
   InitializeUART();
   InitializeI2C();
   InitializeSPI ();

// Get the NRF24L01 ready
   reg = write_spi_reg (W_REGISTER + SETUP_AW, AW5);
   reg = write_spi_buf (W_REGISTER + TX_ADDR, tx_addr, 5);
   reg = write_spi_buf (W_REGISTER + RX_ADDR_P0, tx_addr, 5);
   reg = write_spi_buf (W_REGISTER + RX_ADDR_P1, rx_addr_p1, 5);
   reg = write_spi_reg (W_REGISTER + EN_AA, ENAA_P5 | ENAA_P4 | ENAA_P3 | ENAA_P2 | ENAA_P1 | ENAA_P0);
   reg = write_spi_reg (W_REGISTER + EN_RXADDR, ERX_P5 | ERX_P4 | ERX_P3 | ERX_P2 | ERX_P1 | ERX_P0);
   reg = write_spi_reg (W_REGISTER + SETUP_RETR, ARD4000 | ARC15);
   reg = write_spi_reg (W_REGISTER + RF_CH, 92);
   reg = write_spi_reg (W_REGISTER + RF_SETUP, RF_DR_LOW | RF_PWR_MED | 1);
   reg = write_spi_reg (W_REGISTER + CONFIG, EN_CRC | CRCO | PWR_UP | PTX);
   delay(1); // KEF confirmed needed!

   x = i2c_read_register (BMP180_ADDR, 0xd0);
   memset (tx_payload, 0, sizeof(tx_payload));

while (1) {
// Load the calibration data from the BMP180
//   for (a = 0xaa; a <= 0xbf; ++a) {
//      x = i2c_read_register (BMP180_ADDR, a);
//      tx_payload[a - 0xaa] = x;
//   }

   tx_payload[0] = 0xf0;
   tx_payload[1] = 0x01;

   i2c_set_start_ack ();
   i2c_send_address (BMP180_ADDR, I2C_WRITE);
   i2c_send_reg (0xf4);
   i2c_send_reg (0x2e);
   i2c_set_stop ();
   delay (5);
   for (a = 0xf6; a <= 0xf7; ++a) {
      i2c_set_start_ack ();
      i2c_send_address (BMP180_ADDR, I2C_WRITE);
      i2c_send_reg (a);
      i2c_set_start_ack ();
      i2c_send_address (BMP180_ADDR, I2C_READ);
      reg = I2C_SR1;
      reg = I2C_SR3;
      i2c_set_nak ();
      i2c_set_stop ();
      i2c_read (&x);
      tx_payload[a - 0xf6 + 2] = x;
   }

   i2c_set_start_ack ();
   i2c_send_address (BMP180_ADDR, I2C_WRITE);
   i2c_send_reg (0xf4);
   i2c_send_reg ((OSS << 6) | 0x34);
   i2c_set_stop ();
   delay (30);
   for (a = 0xf6; a <= 0xf8; ++a) {
      i2c_set_start_ack ();
      i2c_send_address (BMP180_ADDR, I2C_WRITE);
      i2c_send_reg (a);
      i2c_set_start_ack ();
      i2c_send_address (BMP180_ADDR, I2C_READ);
      reg = I2C_SR1;
      reg = I2C_SR3;
      i2c_set_nak ();
      i2c_set_stop ();
      i2c_read (&x);
      tx_payload[a - 0xf6 + 4] = x;
   }

// Read the RH from the SI7021
   i2c_set_start_ack ();
   i2c_send_address (SI7021_ADDR, I2C_WRITE);
   i2c_send_reg (SI7021_READ_HUMIDITY);
   //i2c_set_stop ();
   //delay(1);
   i2c_set_start_ack ();
   i2c_send_address (SI7021_ADDR, I2C_READ);
   reg = I2C_SR1;
   reg = I2C_SR3;
   i2c_read (&tx_payload[7]);
   reg = I2C_SR1;
   reg = I2C_SR3;
   i2c_set_nak ();
   i2c_set_stop ();
   i2c_read (&tx_payload[8]);

// Read the LUX from the BH1750
   i2c_set_start_ack ();
   i2c_send_address (BH1750_ADDR, I2C_WRITE);
   i2c_send_reg (BH1750_POWERON);
   i2c_set_stop ();
   delay(1);
   i2c_set_start_ack ();
   i2c_send_address (BH1750_ADDR, I2C_WRITE);
   i2c_send_reg (BH1750_HRES2_ONETIME);
   i2c_set_stop ();
   for (x1 = 0; x1 < 70; ++x1)
      for (y1 = 0; y1 < 70; ++y1)
         for (z1 = 0; z1 < 70; ++z1)
            __asm__("nop");
   i2c_set_start_ack ();
   i2c_send_address (BH1750_ADDR, I2C_READ);
   reg = I2C_SR1;
   reg = I2C_SR3;
   i2c_read (&tx_payload[9]);
   reg = I2C_SR1;
   reg = I2C_SR3;
   i2c_set_nak ();
   i2c_set_stop ();
   i2c_read (&tx_payload[10]);

// Adjust the MCP4725 voltage output
   i2c_set_start_ack ();
   i2c_send_address (MCP4725_ADDR, I2C_WRITE);
   i2c_send_reg (MCP4725_CMD_WRITEDAC);
   i2c_send_reg (voltage >> 4);
   i2c_send_reg ((voltage & 0x0f) << 4);
   i2c_set_stop ();
   voltage = voltage + 50;
   if (voltage == 2300)
      voltage = 1900;

// Read the HMC5883L compass
   i2c_set_start_ack();
   i2c_send_address (HMC5883L_ADDR, I2C_WRITE);
   i2c_send_reg (HMC5883L_CRA);
   i2c_send_reg (0x70);
   i2c_set_start_ack();
   i2c_send_address (HMC5883L_ADDR, I2C_WRITE);
   i2c_send_reg (HMC5883L_CRB);
   i2c_send_reg (0x00);
   i2c_set_start_ack();
   i2c_send_address (HMC5883L_ADDR, I2C_WRITE);
   i2c_send_reg (HMC5883L_MODE);
   i2c_send_reg (0x01);
   i2c_set_stop ();
   delay(3);
   i2c_set_start_ack ();
   i2c_send_address (HMC5883L_ADDR, I2C_READ);
   //i2c_send_reg (0x06);
   for (a = 0; a < 6; ++a) {
      reg = I2C_SR1;
      reg = I2C_SR3;
      if (a == 5) {
         i2c_set_nak ();
         i2c_set_stop ();
      }
      i2c_read (&tx_payload[a+11]);
   }
   

   write_spi_reg (W_REGISTER + STATUS, RX_DR | TX_DS | MAX_RT);
   reg = write_spi_reg (FLUSH_TX, 0);
   delayTenMicro();
   reg = write_spi_buf (W_TX_PAYLOAD, tx_payload, 32);
   delayTenMicro();
   PC_ODR |= (1 << CE);
   delayTenMicro();
   PC_ODR &= ~(1 << CE);

// Delay before looping back
   for (x1 = 0; x1 < 50; ++x1)
      for (y1 = 0; y1 < 50; ++y1)
         for (z1 = 0; z1 < 50; ++z1)
            __asm__("nop");
}
}

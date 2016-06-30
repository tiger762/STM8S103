SENSORS for stm8s103f3p6

Hello!

This is my "bare metal" software package for doing fun things with the stm8s103f3p6 as sold on Ebay for under a Dollar each.

Currently, I have support for these I2C sensors:

BMP180 (pressure and temperature)
Si7021 (humidity)
BH1750FVI (LUX meter)
HMC5883L (magnetic 3-axis compass)
MCP4725 (12-bit DAC. Not a sensor but is a n extremely useful I2C device)

I also am using an NRF24L01 to transmit sensor data to a Raspberry Pi. The NRF uses SPI to communicate.

I am using hardware I2C and SPI. Not "bit-banging".

As far as software packages, you'll need at the very least, the SDCC compiler and a way to upload the resulting IHX file to the device:

On my Raspberry Pi running Raspbian:

apt-get install sdcc

(optional if you want serial comms back to the host) apt-get install minicom

git clone http://github.com/vdudouyt/stm8flash
cd stm8flash
make
make install

The hardware programmer is sold on Ebay for around $4 each. Look for "st-link v2". Note that a hardware programmer is required to program the stm8 family. There is not a serial uploader installed like the stm32 has.

Optional but extremely useful:
Saleae Logic Analyzer (About $7 each) -- to see up to 8 channels of digital waveform. EXTREMELY useful for debugging I2C. Seriously...
PL2303 USB-to-serial cable/converter (about $2 each) -- to be able to "print" from the microcontroller to a minicom terminal

I like to use a Raspberry Pi also with an NRF24L01 to act as server / recipient of the remote sensor data. On the RPi, I can store the data to MySQL database or at the very least, do the "heavy lifting" of converting raw temp/press to calibrated temp/press. 

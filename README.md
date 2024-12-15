# A4 - SPI Digital Analog Converter

## Reference Materials
STM32L4xxxx Reference Manual (RM) – SPI
STM32L476xx Datasheet (DS) – Pinouts
NUCLEO-L476RG Users Manual (UM) – Pin Diagram (L476RG)
Microchip MCP4921 Datasheet

## SPI - Serial Peripheral Interface
Serial Peripheral Interface (SPI) is a common full-duplex, 4-wire bus used by a variety of digital devices. The STM32L4 provides multiple SPI peripherals. Configuration options can be reviewed in the corresponding RM chapter. 

## SPI Signal Naming Convention
SPI signals have historically been labeled using offensive and noninclusive master / slave terminology. There has been an ongoing effort to change the terminology in the electronics industry. While many peripherals have already adopted new terminology (including the DAC we use), most microcontroller documentation continues to use antiquated terminology. In this class I will be following the naming convention proposed by the Open-Source Hardware Association. 

## ST Terminology | Replacement Terminology
MOSI (Master Out / Slave In) | PICO (Peripheral In / Controller Out)
MISO (Master In / Slave Out) | POCI (Peripheral Out / Controller In)
SS (Slave Select) | CS (Chip Select)

## DAC - Digital to Analog Converter
For this assignment you will use a Microchip DAC (digital to analog converter). The MCP4921 is a 12-bit DAC, allowing for 4096 different output voltages, including 0v. The MCP4921 interfaces with an MCU via SPI. The DAC is a write only device so only 3 of the 4-wires from SPI are necessary. Details on how the MCP4921 is controlled can be found in its datasheet. Figure 1 below shows a sample timing diagram for setting the DAC to output a voltage corresponding to the 12-bit value 0x6A7, unbuffered, and with a gain of 1.

## Instructions
### Interface STM32L4 and MCP4921
Connect the MCP4921 DAC to the STM32L4 via SPI. The STM32L4x6 datasheet and NUCLEO-L4x6 user manual can be used to find which pins are selectable for SPI operations. 
Use 3.3 V for power and references for the MCP4921. No 5 V signals are necessary and should be avoided to prevent any unintended overvoltage on the STM32L4 pins.
Write some functions to utilize the DAC and verify they function properly
- DAC_init - initialize the SPI peripheral to communicate with the DAC
- DAC_write - write a 12-bit value to the DAC
- DAC_volt_conv - convert a voltage value into a 12-bit value to control the DAC
Verify the timing of the CS signal by using an oscilloscope or logic analyzer to view CS, SCLK, and SDI on the DAC. CS should go low before SCLK starts and high after all SCLK pulses have stopped.
### Write a program to control the DAC with the keypad
The user will input 3 key presses to form a voltage and the program will control the DAC to set that output voltage. For example, entering ② ⑦ ⑤ should cause the output voltage to be 2.75 V. The maximum ideal output from the DAC is 3.3 V, so input values greater than 330 should output the max voltage (ideally 3.3 V). 
Measure the output voltage with a multimeter and compare it to the specified voltage. Use this measurement to calibrate your DAC function. The output should be within 10 mV of the entered voltage for all values from 000 to 330. (At the max, the DAC will not be able to fully reach 3.3  V since 3.3 V is used for VDD and VREF)

## Hints
- Avoid the use of floats by treating voltage variables in units of mv. So 3.3 V becomes 3300 mV. 
- The DAC may not output a voltage according to the ideal equation in the datasheet. So the conversion equation may need to be adjusted. This is a process called calibration. A technical note in the lab manual describes a process that can be used to calibrate a better fit.

## Deliverables
- Demonstrate your working program to the instructor or lab assistant
- Show at least 3 voltages across the range 0-3.0V (ie 020, 175, 293)
- Create a single pdf document (not a full lab report) containing the following:
- Image of single transmission to the DAC including CS, SCLK, and SDI
- Comment what data was being transmitted in the capture
- Properly formatted C source code
  - Properly formatted implies properly tabbed with sufficient comments and adjusted so code doesn’t wrap, excess white space removed for a compact presentation, following accepted code formatting standards, header comments at the top of each new file to assist the reader in quickly interpreting what they are looking at, etc.



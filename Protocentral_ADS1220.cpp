#include <Arduino.h>
#include "Protocentral_ADS1220.h"
#include <SPI.h>


 
void Protocentral_ADS1220::writeRegister(uint8_t address, uint8_t value)
{
  SPI.setDataMode(SPI_MODE1);
  digitalWrite(ADS1220_CS_PIN,LOW);
  delay(1);
  SPI.transfer(WREG|(address<<2));      	
  SPI.transfer(value); 
  delay(1);
  digitalWrite(ADS1220_CS_PIN,HIGH);
}  

uint8_t Protocentral_ADS1220::readRegister(uint8_t address)
{
  uint8_t data;
  SPI.setDataMode(SPI_MODE1);
  digitalWrite(ADS1220_CS_PIN,LOW);
  delay(1);
  SPI.transfer(RREG|(address<<2));      	
  data = SPI.transfer(SPI_MASTER_DUMMY); 
  delay(1);
  digitalWrite(ADS1220_CS_PIN,HIGH);

  return data;
}  


void Protocentral_ADS1220::begin()
{
  static char data;

  //Serial.begin(9600);	        	//115200 57600
  SPI.begin();                           // wake up the SPI bus.
  //SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE1);
	
  delay(100);
  SPI_Reset();                                            
  delay(100);                                                    

  digitalWrite(ADS1220_CS_PIN,LOW);
  /*SPI.transfer(WREG);           //WREG command (43h, 08h, 04h, 10h, and 00h)
  SPI.transfer(0x01);      	
  SPI.transfer(0x04);     
  SPI.transfer(0x10);    
  SPI.transfer(0x00);   
  */


  Config_Reg0 = 0b00110101;
	/*  Config Reg 0 Settings:
		Input multiplexer configuration
			These bits configure the input multiplexer.
			For settings where AINN = AVSS, the PGA must be disabled (PGA_BYPASS = 1)
			and only gains 1, 2, and 4 can be used.
			0000 : AINP = AIN0, AINN = AIN1 (default)
			0001 : AINP = AIN0, AINN = AIN2
			0010 : AINP = AIN0, AINN = AIN3
			0011 : AINP = AIN1, AINN = AIN2
			0100 : AINP = AIN1, AINN = AIN3
			0101 : AINP = AIN2, AINN = AIN3
			0110 : AINP = AIN1, AINN = AIN0
			0111 : AINP = AIN3, AINN = AIN2
			1000 : AINP = AIN0, AINN = AVSS
			1001 : AINP = AIN1, AINN = AVSS
			1010 : AINP = AIN2, AINN = AVSS
			1011 : AINP = AIN3, AINN = AVSS
			1100 : (V(REFPx) – V(REFNx)) / 4 monitor (PGA bypassed)
			1101 : (AVDD – AVSS) / 4 monitor (PGA bypassed)
			1110 : AINP and AINN shorted to (AVDD + AVSS) / 2
			1111 : Reserved

		Gain configuration
			These bits configure the device gain.
			Gains 1, 2, and 4 can be used without the PGA. In this case, gain is obtained by
			a switched-capacitor structure.
			000 : Gain = 1 (default)
			001 : Gain = 2
			010 : Gain = 4
			011 : Gain = 8
			100 : Gain = 16
			101 : Gain = 32
			110 : Gain = 64
			111 : Gain = 128

		Disables and bypasses the internal low-noise PGA
			Disabling the PGA reduces overall power consumption and allows the commonmode
			voltage range (VCM) to span from AVSS – 0.1 V to AVDD + 0.1 V.
			The PGA can only be disabled for gains 1, 2, and 4.
			The PGA is always enabled for gain settings 8 to 128, regardless of the
			PGA_BYPASS setting.
			0 : PGA enabled (default)
			1 : PGA disabled and bypassed
	*/


  Config_Reg1 = 0b10100100;
	/*  Config Reg 1 Settings:
		Data rate
			These bits control the data rate setting depending on the selected operating
			mode. Table 18 lists the bit settings for normal, duty-cycle, and turbo mode.
			Normal Mode: 001 = 45 SPS, 010 = 90 SPS, 011 = 175 SPS 

		Operating mode
			These bits control the operating mode the device operates in.
			00 : Normal mode (256-kHz modulator clock, default)
			01 : Duty-cycle mode (internal duty cycle of 1:4)
			10 : Turbo mode (512-kHz modulator clock)
			11 : Reserved

		Conversion mode
			This bit sets the conversion mode for the device.
			0 : Single-shot mode (default)
			1 : Continuous conversion mode

		Temperature sensor mode
			This bit enables the internal temperature sensor and puts the device in
			temperature sensor mode.
			The settings of configuration register 0 have no effect and the device uses the
			internal reference for measurement when temperature sensor mode is enabled.
			0 : Disables temperature sensor (default)
			1 : Enables temperature sensor

		Burn-out current sources
			This bit controls the 10-µA, burn-out current sources.
			The burn-out current sources can be used to detect sensor faults such as wire
			breaks and shorted sensors.
			0 : Current sources off (default)
			1 : Current sources on
	*/


  Config_Reg2 = 0b11000000;
	/*  Config Reg 2 Settings:
		Voltage reference selection
			These bits select the voltage reference source that is used for the conversion.
			00 : Internal 2.048-V reference selected (default)
			01 : External reference selected using dedicated REFP0 and REFN0 inputs
			10 : External reference selected using AIN0/REFP1 and AIN3/REFN1 inputs
			11 : Analog supply (AVDD – AVSS) used as reference

		FIR filter configuration
			These bits configure the filter coefficients for the internal FIR filter.
			These bits only affect the 20-SPS setting in normal mode and 5-SPS setting in
			duty-cycle mode.
			00 : No 50-Hz or 60-Hz rejection (default)
			01 : Simultaneous 50-Hz and 60-Hz rejection
			10 : 50-Hz rejection only
			11 : 60-Hz rejection only

		Low-side power switch configuration
			This bit configures the behavior of the low-side switch connected between
			AIN3/REFN1 and AVSS.
			0 : Switch is always open (default)
			1 : Switch automatically closes when the START/SYNC command is sent and
			opens when the POWERDOWN command is issued

		IDAC current setting
			These bits set the current for both IDAC1 and IDAC2 excitation current sources.
			000 : Off (default)
			001 : 10 µA
			010 : 50 µA
			011 : 100 µA
			100 : 250 µA
			101 : 500 µA
			110 : 1000 µA
			111 : 1500 µA
	*/


  Config_Reg3 = 0b00000000;
  	/*  Config Reg 3 Settings:
		IDAC1 routing configuration
			These bits select the channel where IDAC1 is routed to.
			000 : IDAC1 disabled (default)
			001 : IDAC1 connected to AIN0/REFP1
			010 : IDAC1 connected to AIN1
			011 : IDAC1 connected to AIN2
			100 : IDAC1 connected to AIN3/REFN1
			101 : IDAC1 connected to REFP0
			110 : IDAC1 connected to REFN0
			111 : Reserved

		IDAC2 routing configuration
			These bits select the channel where IDAC2 is routed to.
			000 : IDAC2 disabled (default)
			001 : IDAC2 connected to AIN0/REFP1
			010 : IDAC2 connected to AIN1
			011 : IDAC2 connected to AIN2
			100 : IDAC2 connected to AIN3/REFN1
			101 : IDAC2 connected to REFP0
			110 : IDAC2 connected to REFN0
			111 : Reserved

		DRDY mode
			This bit controls the behavior of the DOUT/DRDY pin when new data are ready.
			0 : Only the dedicated DRDY pin is used to indicate when data are ready (default)
			1 : Data ready is indicated simultaneously on DOUT/DRDY and DRDY
			
		Reserved
			Always write 0
	*/
  writeRegister( CONFIG_REG0_ADDRESS , Config_Reg0);
  writeRegister( CONFIG_REG1_ADDRESS , Config_Reg1);
  writeRegister( CONFIG_REG2_ADDRESS , Config_Reg2);
  writeRegister( CONFIG_REG3_ADDRESS , Config_Reg3);
  delay(100);
  /*
  SPI.transfer(RREG);           //RREG
  data = SPI.transfer(SPI_MASTER_DUMMY);
  //Serial.println(data);
  data = SPI.transfer(SPI_MASTER_DUMMY); 
  //Serial.println(data);
  data = SPI.transfer(SPI_MASTER_DUMMY); 
  //Serial.println(data);
  data = SPI.transfer(SPI_MASTER_DUMMY); 
  //Serial.println(data);
  */

  Config_Reg0 = readRegister(CONFIG_REG0_ADDRESS);
  Config_Reg1 = readRegister(CONFIG_REG1_ADDRESS);
  Config_Reg2 = readRegister(CONFIG_REG2_ADDRESS);
  Config_Reg3 = readRegister(CONFIG_REG3_ADDRESS);

  //Serial.println("Config_Reg : ");
  //Serial.println(Config_Reg0,HEX);
  //Serial.println(Config_Reg1,HEX);
  //Serial.println(Config_Reg2,HEX);
  //Serial.println(Config_Reg3,HEX);
  //Serial.println(" ");
  digitalWrite(ADS1220_CS_PIN,HIGH); //release chip, signal end transfer

  SPI_Start();
  delay(100);

}

void Protocentral_ADS1220::SPI_Command(unsigned char data_in)
{
  SPI.setDataMode(SPI_MODE1);
  digitalWrite(ADS1220_CS_PIN, LOW);
  delay(2);
  digitalWrite(ADS1220_CS_PIN, HIGH);
  delay(2);
  digitalWrite(ADS1220_CS_PIN, LOW);
  delay(2);
  SPI.transfer(data_in);
  delay(2);
  digitalWrite(ADS1220_CS_PIN, HIGH);
}

void Protocentral_ADS1220::SPI_Reset()
{
  SPI_Command(RESET);		                    			
}

void Protocentral_ADS1220::SPI_Start()
{
  SPI_Command(START);
}


Protocentral_ADS1220::Protocentral_ADS1220() 		// Constructors 
{
  //Serial.begin(9600);	        	//115200 57600
  //Serial.println("ads1220 class declared");
  NewDataAvailable = false;
}

void Protocentral_ADS1220::PGA_ON(void)
{	 
  Config_Reg0 &= ~_BV(0);
  writeRegister(CONFIG_REG0_ADDRESS,Config_Reg0);	
}

void Protocentral_ADS1220::PGA_OFF(void)
{	 
  Config_Reg0 |= _BV(0);
  writeRegister(CONFIG_REG0_ADDRESS,Config_Reg0);	
}

void Protocentral_ADS1220::Continuous_conversion_mode_ON(void)
{
  Config_Reg1 |= _BV(2);
  writeRegister(CONFIG_REG1_ADDRESS,Config_Reg1);
}

void Protocentral_ADS1220::Single_shot_mode_ON(void)
{
  Config_Reg1 &= ~_BV(2);
  writeRegister(CONFIG_REG1_ADDRESS,Config_Reg1);
}


void Protocentral_ADS1220::set_data_rate(int datarate)
{
  Config_Reg1 &= ~REG_CONFIG_DR_MASK;
  
  switch(datarate)
  {
    case(DR_20SPS):
      Config_Reg1 |= REG_CONFIG_DR_20SPS; 
      break;
    case(DR_45SPS):
      Config_Reg1 |= REG_CONFIG_DR_45SPS; 
      break;
    case(DR_90SPS):
      Config_Reg1 |= REG_CONFIG_DR_90SPS; 
      break;
    case(DR_175SPS):
      Config_Reg1 |= REG_CONFIG_DR_175SPS; 
      break;
    case(DR_330SPS):
      Config_Reg1 |= REG_CONFIG_DR_330SPS; 
      break;
    case(DR_600SPS):
      Config_Reg1 |= REG_CONFIG_DR_600SPS; 
      break;
    case(DR_1000SPS):
      Config_Reg1 |= REG_CONFIG_DR_1000SPS; 
      break;
  }

  writeRegister(CONFIG_REG1_ADDRESS,Config_Reg1);
}


void Protocentral_ADS1220::set_pga_gain(int pgagain)
{
  Config_Reg0 &= ~REG_CONFIG_PGA_GAIN_MASK;

  switch(pgagain)
  {
    case(PGA_GAIN_1):
      Config_Reg0 |= REG_CONFIG_PGA_GAIN_1 ; 
      break;
    case(PGA_GAIN_2):
      Config_Reg0 |= REG_CONFIG_PGA_GAIN_2; 
      break;
    case(PGA_GAIN_4):
      Config_Reg0 |= REG_CONFIG_PGA_GAIN_4; 
      break;
    case(PGA_GAIN_8):
      Config_Reg0 |= REG_CONFIG_PGA_GAIN_8; 
      break;
    case(PGA_GAIN_16):
      Config_Reg0 |= REG_CONFIG_PGA_GAIN_16; 
      break;
    case(PGA_GAIN_32):
      Config_Reg0 |= REG_CONFIG_PGA_GAIN_32; 
      break;
    case(PGA_GAIN_64):
      Config_Reg0 |= REG_CONFIG_PGA_GAIN_64; 
      break;
    case(PGA_GAIN_128):
      Config_Reg0 |= REG_CONFIG_PGA_GAIN_128; 
      break;
  }
  
  writeRegister(CONFIG_REG0_ADDRESS,Config_Reg0);
}

void Protocentral_ADS1220::set_mpx_pins(int mpx_pins)
{
  Config_Reg0 &= 0b00001111;
  Config_Reg0 |= mpx_pins;
  writeRegister(CONFIG_REG0_ADDRESS,Config_Reg0);
}

uint8_t * Protocentral_ADS1220::get_config_reg()
{
  static uint8_t config_Buff[4];

  Config_Reg0 = readRegister(CONFIG_REG0_ADDRESS);
  Config_Reg1 = readRegister(CONFIG_REG1_ADDRESS);
  Config_Reg2 = readRegister(CONFIG_REG2_ADDRESS);
  Config_Reg3 = readRegister(CONFIG_REG3_ADDRESS);

  config_Buff[0] = Config_Reg0 ; 
  config_Buff[1] = Config_Reg1 ;
  config_Buff[2] = Config_Reg2 ;
  config_Buff[3] = Config_Reg3 ;

  return config_Buff;
}


uint8_t * Protocentral_ADS1220::Read_Data()
{
  static byte SPI_Buff[3];

  if((digitalRead(ADS1220_DRDY_PIN)) == LOW)             //        Wait for DRDY to transition low
  {
     SPI.setDataMode(SPI_MODE1);
  	digitalWrite(ADS1220_CS_PIN,LOW);                         //Take CS low
  	delayMicroseconds(100);
  	for (int i = 0; i < 3; i++)
  	{ 
  	  SPI_Buff[i] = SPI.transfer(SPI_MASTER_DUMMY);
  	}
  	delayMicroseconds(100);
  	digitalWrite(ADS1220_CS_PIN,HIGH);                  //  Clear CS to high
  	NewDataAvailable = true;
  }
  	
  return SPI_Buff;
}

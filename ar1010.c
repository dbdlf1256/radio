#include <stdio.h>

// AR1010 Slave Address
#define AR1010_ADDR	0x10
#define AR1010_ADDR_W	(AR1010_ADDR << 1) | 0x00 // 0x20
#define AR1010_ADDR_R	(AR1010_ADDR << 1) | 0X01 // 0x21

// AR1010 Register
#define R_0		0x00	// xo_en, ENABLE
#define R_1		0x01	// stc_int_en, deemp, mono, smute, hmute
#define R_2		0x02	// TUNE, CHAN
#define R_3		0x03	// SEEKUP, SEEK, SPACE, BAND, VOLUME
#define R_10		0x0A	// seek_wrap
#define R_11		0x0B	// hilo_side, hiloctrl_b1, hiloctrl_b2
#define R_13		0x0C	// AR_GPIO3, AR_GPIO2, AR_GPIO1
#define R_14		0x0D	// VOLUEM2
#define R_15		0x0E	// RDSs
#define R_SSI		0x12	// RSSI, IF_CNT
#define R_STATUS	0x13 	// READCHAN, STC, SF, ST
#define R_DEVID		0x1B	// VERSION, MFID
#define R_CHIPID	0x1C	// CHIPID

// AR1010 R_0 BIT
#define R0_xo_en	0x0030
#define R0_ENABLE	0x0000

// AR1010 R_1 BIT
#define R1_stc_int_en	0x0020
#define R1_deemp	0x0010
#define R1_mono		0x0008
#define R1_smute	0x0004
#define R1_hmute	0x0002

// AR1010 R_2 BIT
#define R2_TUNE		0x0200
#define R2_CHAN		0x01FF

// AR1010 R_3 BIT
#define R3_SEEKUP	0x8000
#define R3_SEEK		0x4000
#define R3_SPACE	0x2000
#define R3_BAND		0x1800
#define R3_VOLUME	0x0780
#define R3_SEEKTH	0x007F

// AR1010 R_10 BIT
#define R10_seek_wrap	0x0008	

// AR1010 R_11 BIT
#define R11_hilo_side	0x8000
#define R11_hiloctrl_b1	0x0004
#define R11_hiloctrl_b2	0x0000

// AR1010 R_13 BIT
#define R13_GPIO3	0x0003
#define R13_GPIO2	0x000C
#define R13_GPIO1	0x0030

// AR1010 R_14 BIT
#define R14_VOLUME2	0xF000	// 4bit i don't know location

// AR1010 R_15 BIT
// RDS Register But AR1010 has not RDS function
// #define R15_	

// AR1010 R_SSI BIT
#define RSSI_IF_CNT	0x01FF
#define RSSI_RSSI	0xFE00

// AR1010 R_STATUS BIT
#define RSTATUS_READCHAN	0xFF80
#define RSTATUS_STC	0x0020
#define RSTATUS_SF	0x0010
#define RSTATUS_ST	0x0008

// AR1010 R_DEVICE BIT
#define RDEVICE_VERSION	0x0FFF
#define RDEVICE_MFID	0xF000

// AR1010 R_CHIPID BIT
#define RCHIPID_CHIPNO	0xFFFF

// Volume Step Value
#define AR1010_VOL_STEP0	0x0F
#define AR1010_VOL_STEP1	0xCF
#define AR1010_VOL_STEP2	0xDF
#define AR1010_VOL_STEP3	0xFF
#define AR1010_VOL_STEP4	0xCB
#define AR1010_VOL_STEP5	0xDB
#define AR1010_VOL_STEP6	0xFB
#define AR1010_VOL_STEP7	0xFA
#define AR1010_VOL_STEP8	0xF9
#define AR1010_VOL_STEP9	0xF8
#define AR1010_VOL_STEP10	0xF7
#define AR1010_VOL_STEP11	0xD6
#define AR1010_VOL_STEP12	0xE6
#define AR1010_VOL_STEP13	0xF6
#define AR1010_VOL_STEP14	0xE3
#define AR1010_VOL_STEP15	0xF3
#define AR1010_VOL_STEP16	0xF2
#define AR1010_VOL_STEP17	0xF1
#define AR1010_VOL_STEP18	0xF0

// AR1010 Default Register Value
#define AR1010_W_REG_SIZE	18
const unsigned short ar1010DefualtRegValIn[AR1010_W_REG_SIZE] = {
	0xFFFB, // R0: 1111 1111 1111 1011 xo_en: set, ENABLE: set
	0x5B15, // R1: 0101 1011 0001 0101 stc_int_en: reset, deemp: set, mono: reset, smute: set, fmute: reset
	0xD0B9, // R2: 1101 0000 1011 1001 TUEN: reset, CHAN: 0 1011 1001
	0xA010, // R3: 1010 0000 0001 0000 SEEKUP: set, SEEK: reset, SPACE: set, BAND: 00, VOLUEM: 0000, SEEKTH: 001 0000(16)
	0x0780, // R4
	0x28AB, // R5
	0x6400, // R6
	0x1EE7, // R7
	0x7141, // R8
	0x007D, // R9
	0x82C6, // R10: 1000 0010 1100 0110 seek_wrap: reset
	0x4E55, // R11: 0100 1110 0101 0101 hilo_side: reset, hiloctrl_b1: set, hiloctrl_b2: set
	0x970C, // R12
	0xB845, // R13: 1011 1000 0100 0101 GPIO3: 00, GPIO2 01, GPIO1 01
	0xFC2D, // R14: 1111 1100 0010 1101 VOLUME2: 1111
	0x8097, // R15: 1000 0000 1001 0111 
	0x04A1, // R16
	0xDF6A  // R17
};
const unsigned short ar1010DefualtRegValEx[AR1010_W_REG_SIZE] = {
	0xFF7B, // R0: 1111 1111 0111 1011 xo_en: reset, ENABLE: set
	0x5B15, // R1: 0101 1011 0001 0101 stc_int_en: reset, deemp: set, mono: reset, smute: set, fmute: reset
	0xD0B9, // R2: 1101 0000 1011 1001 TUNE: reset, CHAN: 0 1011 1001
	0xA010, // R3: 1010 0000 0001 0000 SEEKUP: set, SEEK: reset, SPACE: set, BAND: 00, VOLUME: 0000, SEEKTH: 001 0000(16)
	0x0780, // R4
	0x28AB, // R5
	0x6400, // R6
	0x1EE7, // R7
	0x7141, // R8
	0x007D, // R9
	0x82C6, // R10: 1000 0010 1100 0110 seek_wrap: reset
	0x4F55, // R11: 0100 1111 0101 0101 hilo_side: reset, hiloctrl_b1: set, hiloctrl_b2: set
	0x970C, // R12 
	0xB845, // R13: 1011 1000 0100 0101 GPIO3 00, GPIO2 01, GPIO1 01
	0xFC2D, // R14: 1111 1100 0010 1101 VOLUME2: 1111
	0x8097, // R15: 1000 0000 1001 0111
	0x04A1, // R16
	0xDF6A  // R17
};

int xo_en = 0;

int I2cTransmit(unsigned char addrWR, unsigned char reg, unsigned char* data){}
int I2cWrite(unsigned char addr, unsigned char reg,  unsigned char* data, int dataLen){}
int I2cRead(unsigned char addr, unsigned char reg, unsigned char* buff, int buffLen){}

int Ar1010Transmit()
{

	return 0;
}

int Ar1010Write(unsigned char regAddr, unsigned short* data, int dataLen)
{
	/*
	 * AR1010 Write sequence (i2c protocol?)
	 * 1. Send Start Signal
	 * 2. Send Slave Address that include Write bit
	 * 3. Receive ACK from Slave
	 * 4. Send Register Address
	 * 5. Receive ACK from Slave
	 * 6. Send Data that 1Byte of 2Byte Data of MSB
	 * 7. Receive ACK from Slave
	 * 8. Send Data that 1Byte of 2Byte Data of LSB
	 * 9. Receive ACK froma Slave
	 * 10. Send Stop Signal
	 *
	 * It's Write of I2C Protocol
	 */
	if(data == NULL)
	{
		printf("data is NULL!\r\n");
		return -1;
	}

	I2cWrite(AR1010_ADDR, regAddr, data, dataLen);

	return 0;
}

int Ar1010Read(unsigned char regAddr, unsigned short* buff, int buffLen)
{
	/*
	 * AR1010 Read sequence (i2c protocol?)
	 * 1. Send Start Signal
	 * 2. Send Slave Address that include Write bit
	 * 3. Receive ACK from Slave
	 * 4. Send Register Address
	 * 5. Receive ACK from Slave
	 * 6. Send Start Signal
	 * 7. Send Slave Address that include Read bit
	 * 8. Receive ACK from Slave
	 * 9. Send clock to Slave
	 * 10. Receive byte of Register Data that is MSB of 2Byte
	 * 11. Send ACK to Slave
	 * 12. Receive byte of Register Data that is LSB of 2Byte
	 * 13. Send ACK to Slave
	 * 14. Send Stop Signal
	 *
	 * It's Read of I2C Protocol Sequence
	 */
	if(buff == NULL)
	{
		printf("buff is NULL!\r\n");
		return -1;
	}

	unsigned char temp[buffLen] = { 0, };

	I2cRead(AR1010_ADDR, regAddr, temp, buffLen);

	memcpy(buff, temp, buffLen);

	return 0;
}

int Ar1010Tune()
{
	unsigned short reg2 = 0;

	Ar1010Read(R_2, &reg2, sizeof(reg2));

	reg2 &= ~(R2_TUNE);

	Ar1010Write(R_2, &reg2, sizeof(reg2));

	reg2 |= R2_TUNE;

	Ar1010Write(R_2, &reg2, sizeof(reg2));

	return 0;
}

int Ar1010Init()
{
	int res = 0;
	unsigned short status = 0;
	char stc = 0;

	// delay for stable
	delay(1);

	// check the ex_no bit
	if(xo_en == 1)
	{
		// set registers
		for(int reg = R_0; reg < AR1010_W_REG_SIZE; reg++)
		{
			res = Ar1010Write(reg, &ar1010DefualtRegValIn[reg], 2);
			if(res < 0)
			{
				printf("AR1010 Register Initialize Fail!\r\n");
				return -1;
			}
		}
	}
	else
	{
		// set registers
		for(int reg = R_0; reg < AR1010_W_REG_SIZE; reg++)
		{
			res = Ar1010Write(reg, &ar1010DefualtRegValEx[reg], 2);
			if(res < 0)
			{
				printf("AR1010 Register Initialize Fail!\r\n");
				return -1;
			}
		}	
	}

	// Check STC Flag
	do
	{
		Ar1010Read(R_STATUS, &status, 2);
		stc = (status & ~(RSTATUS_STC)) >> 5;
	}while(stc == 0)

	// TUNING
	Ar1010Tune();

	return 0;
}


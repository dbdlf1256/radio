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

typedef struct _ar1010Reg_t
{
	unsigned short r0;
	unsigned short r1;
	unsigned short r2;
	unsigned short r3;
	unsigned short r4;
	unsigned short r5;
	unsigned short r6;
	unsigned short r7;
	unsigned short r8;
	unsigned short r9;
	unsigned short r10;
	unsigned short r11;
	unsigned short r12;
	unsigned short r13;
	unsigned short r14;
	unsigned short r15;
	unsigned short r16;
	unsigned short r17;
	unsigned short rssi;
	unsigned short rstatus;
	unsigned short rbs;
	unsigned short rds1;
	unsigned short rds2;
	unsigned short rds3;
	unsigned short rds4;
	unsigned short rds5;
	unsigned short rds6;
	unsigned short rdevice;
	unsigned short rchipid;
} ar1010Reg_t;

typedef union _ar1010R_t
{
	ar1010Reg_t mem;
	unsigned short rbuff[sizeof(ar1010Reg_t)];
	unsigned short wbuff[AR1010_W_REG_SIZE];
} ar1010R_t;

ar1010R_t ar1010;

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

int Ar1010Read(unsigned char regAddr/*, unsigned short* buff, int buffLen*/)
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
	/*
	if(buff == NULL)
	{
		printf("buff is NULL!\r\n");
		return -1;
	}
	*/

	I2cRead(AR1010_ADDR, regAddr, &ar1010.rbuff[regAddr], 2);

	return 0;
}

// R_2 functions
int Ar1010Tune()
{
	char res = 0;
	unsigned short r2 = ar1010.mem.r2;

	// Ar1010Read(R_2/*, &reg2, sizeof(reg2)*/);

	r2 &= ~(R2_TUNE);

	res = Ar1010Write(R_2, &r2, 2);
	if(res < 0)
	{
		printf("Reset TUNE bit fail!\r\n");
		return res;
	}

	ar1010.mem.r2 = r2;
	r2 |= R2_TUNE;

	res = Ar1010Write(R_2, &r2, 2);
	if(res < 0)
	{
		printf("Set TUNE bit fail!\r\n");
		return res;
	}

	ar1010.mem.r2 = r2;

	return 0;
}

// AR1010 initialize sequence
int Ar1010Init()
{
	int res = 0;
	// unsigned short status = 0;
	char stc = 0;

	memset(&ar1010, 0, sizeof(ar1010R_t));

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
			ar1010.wbuff[reg] = ar1010DefualtRegValIn[reg];
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
			ar1010.wbuff[reg] = ar1010DefualtRegValEx[reg];
		}	
	}

	// Check STC Flag
	do
	{
		Ar1010Read(R_STATUS/*, &status, 2*/);
		stc = (ar1010.mem.rstatus & ~(RSTATUS_STC)) >> 5;
	}while(stc == 0);

	// TUNING
	Ar1010Tune();

	return 0;
}

// R_0 functions
// 세마포어 걸어야 하나?
int SetXoEnable(int enable)
{
	int res = 0;
	unsigned short r0 = ar1010.mem.r0;

	if (enable)
	{
		r0 |= R0_xo_en; // using external reference clock
	}
	else
	{
		r0 &= ~(R0_xo_en); // using internal crystal osillator
	}

	res = Ar1010Write(R_0, &r0, 2);
	if(res < 0)
	{
		printf("xo_en config fail!\r\n");
		return res;
	}
	
	ar1010.mem.r0 = r0;

	return res;
}

// 세마포어 걸어야 하나?
int SetSlip(int enable)
{
	char res = 0;
	unsigned short r0 = ar1010.mem.r0;

	if (enable)
	{
		r0 |= R0_ENABLE; // low power mode (should be init when AR1010 wake up)
	}
	else
	{
		r0 &= ~(R0_ENABLE); // normal mode (should be init in this logic)
	}

	res = Ar1010Write(R_0, &r0, 2);
	if(res < 0)
	{
		printf("Slip mode config fail!\r\n");
		return res;
	}
	
	ar1010.mem.r0 = r0;

	return res;
}

// R1
// 세마포어 걸어야 하나?
int setStcIntEn(int enable)
{
	char res = 0;
	unsigned short r1 = ar1010.mem.r1;

	if(enable)
	{
		r1 |= R1_stc_int_en; // STC interrupt enable (Seek Tune Complete)
	}
	else
	{
		r1 &= ~(R1_stc_int_en); // STC interrupt disable (Seek Tune Complete)
	}

	res = Ar1010Write(R_1, &r1, 2);
	if(res < 0)
	{
		printf("set stc_int_en fail!\r\n");
		return res;
	}

	ar1010.mem.r1 = r1;

	return res;
}

// 세마포어 걸어야 하나?
int SetDeemp(int select)
{
	char res = 0;
	unsigned short r1 = ar1010.mem.r1;

	switch(select)
	{
		case 0:
			r1 &= ~(R1_deemp); // 50us de-emphasis
			break;
		case 1:
			r1 |= R1_deemp; // 75us de-emphasis
			break;
		default:
			printf("Unknown select value!\r\n");
			res = -1;
			return res;
	}

	res = Ar1010Write(R_1, &r1, 2);
	if(res < 0)
	{
		printf("set deemp fail!\r\n");
		return res;
	}

	ar1010.mem.r1 = r1;
	
	return res;
}

// 세마포어 걸어야 하나?
int SetMono(int enable)
{
	char res = 0;
	unsigned short r1 = ar1010.mem.r1;

	if(enable)
	{
		r1 |= R1_mono; // mono enable
	}
	else
	{
		r1 &= ~(R1_mono); // mono or sterero (depends on input signal strength)
	}

	res = Ar1010Write(R_1, &r1, 2);
	if(res < 0)
	{
		printf("Set mono fail!\r\n");
		return res;
	}

	ar1010.mem.r1 = r1;

	return res;
}

// 세마포어 걸어야 하나?
int SetSmute(int enable)
{
	char res = 0;
	unsigned short r1 = ar1010.mem.r1;

	if(enable)
	{
		r1 |= R1_smute; // on soft mute
	}
	else
	{
		r1 &= ~(R1_smute); // of soft mute
	}

	res = Ar1010Write(R_1, &r1, 2);
	if(res < 0)
	{
		printf("Set mono fail!\r\n");
		return res;
	}

	ar1010.mem.r1 = r1;

	return res;
}

// 세마포어 걸어야 하나?
int SetHmute(int enable)
{
	char res = 0;
	unsigned short r1 = ar1010.mem.r1;

	if(enable)
	{
		r1 |= R1_hmute; // on hard mute
	}
	else
	{
		r1 &= ~(R1_hmute); // off hard mute
	}

	res = Ar1010Write(R_1, &r1, 2);
	if(res < 0)
	{
		printf("Set mono fail!\r\n");
		return res;
	}

	ar1010.mem.r1 = r1;

	return res;
}

// R_2 functions
// 세마포어 걸어야 하나?
// int Ar1010Tuning(double freq)
int SetChannel(double freq)
{
	char res = 0;
	// BAND 값에 따라 허용 범위가 달라져야 함 (수정 필요)
	if(freq < 87.5)
	{
		freq = 87.5;
	}
	else if(freq > 108.0)
	{
		freq = 108.0;
	}
	else
	{}

	unsigned short chan = (freq - 69) * 10;

	res = Ar1010Write(R_2, &chan, sizeof(chan));
	if(res < 0)
	{
		printf("Set Channel fail!\r\n");
		return res;
	}

	ar1010.mem.r2 &= ~(R2_CHAN);
	ar1010.mem.r2 |= chan;

	Ar1010Tune();

	return 0;
}

//R_3 functions
// 세마포어 걸어야 하나?
int SetSeekUpDown(int upDown)
{
	char res = 0;
	unsigned short r3 = ar1010.mem.r3;

	if(upDown)
	{
		r3 |= R3_SEEKUP; // Seek Up
	}
	else
	{
		r3 &= !(R3_SEEKUP); // Seek Down
	}

	res = Ar1010Write(R_3, &r3, 2);
	if(res < 0)
	{
		printf("Set Seek Up/Down fail!\r\n");
		return res;
	}

	return res;
}

// 세마포어 걸어야 하나?
int Ar1010Seek()
{
	char res = 0;
	unsigned short r3 = ar1010.mem.r3;

	r3 &= ~(R3_SEEK);
	res = Ar1010Write(R_3, &r3, 2);
	if(res < 0)
	{
		printf("fail to Reset SEEK bit!\r\n");
		return res;
	}
	
	printf("Reset SEEK!\r\n");

	r3 |= R3_SEEK;
	res = Ar1010Write(R_3, &r3, 2);
	if(res < 0)
	{
		printf("fail to Set SEEK bit!\r\n");
		return res;
	}
	
	printf("Set SEEK!\r\n");

	ar1010.mem.r3 = r3;

	return 0;
}

// 세마포어 걸어야 하나?
int SetSpace(int select)
{
	char res = 0;
	unsigned short r3 = ar1010.mem.r3;

	switch(select)
	{
		case 0:
			r3 &= ~(R3_SPACE); // Space value = 200kHz
			break;
		case 1:
			r3 |= R3_SPACE; // Space value = 100kHz
			break;
		default:
			printf("Unknown Space Value!\r\n");
			res = -1;
			return res;
	}

	res = Ar1010Write(R_3, &r3, 2);
	if(res < 0)
	{
		printf("Set Space value fail!\r\n");
		return res;
	}

	ar1010.mem.r3 = r3;

	return res;
}

#define BAND_US_EU	0x0000
#define BAND_JP		0x1000
#define BAND_JP_EX	0x1800
// 세마포어 걸어야 하나?
int SetBand(int select)
{
	char res = 0;
	unsigned short r3 = ar1010.mem.r3;

	r3 &= ~(R3_BAND);

	switch(select)
	{
		case 0:
		case 1:
			r3 |= BAND_US_EU; // Band -> 87.5 ~ 108MHz
			break;
		case 2:
			r3 |= BAND_JP; // Band -> 76 ~ 90MHz
			break;
		case 3:
			r3 |= BAND_JP_EX; // Band -> 76 ~ 108MHz
			break;
		default:
			printf("Unknown select value!\r\n");
			res = -1;
			return res;
	}

	res = Ar1010Write(R_3, &r3, 2);
	if(res < 0)
	{
		printf("Set Band fail!\r\n");
		return res;
	}

	ar1010.mem.r3 = r3;

	return res;
}

// VOLUME은 2개의 레지스터의 각 BIT를 설정해줘야 하기 때문에 나중에 해당 함수 구현 예정(수정 필요)

#define SEEKTH_MAX	127

// 세마포어가 필요 한가?
// 입력 신호의 세기가 여기서 설정한 값 이상이어야 SEEK 동작을 수행한다.
int SetSeekThreshold(unsigned short threshold)
{
	char res = 0;
	unsigned short r3 = ar1010.mem.r3;

	r3 &= ~(R3_SEEKTH);

	if(threshold > SEEKTH_MAX)
	{
		threshold = SEEKTH_MAX;
	}

	r3 |= threshold;

	res = Ar1010Write(R_3, &r3, 2);
	if(res < 0)
	{
		printf("Set Seek Threshold fail!\r\n");
		return res;
	}

	ar1010.mem.r3 = r3;

	return res;
}

// R_10 function
int SetSeekWrap(int enable)
{
	char res = 0;
	unsigned short r10 = ar1010.mem.r10;

	if(enable)
	{
		r10 |= R10_seek_wrap; // Enable Seek Wrap
	}
	else
	{
		r10 &= ~(R10_seek_wrap); // Disable Seek Wrap
	}

	res = Ar1010Write(R_10, &r10, 2);
	if(res < 0)
	{
		printf("Set Seek Wrap fail!\r\n");
		return res;
	}

	ar1010.mem.r10 = r10;

	return res;
}

// R_11 functions
int SetHiloSide(int side)
{
	char res = 0;
	unsigned short r11 = ar1010.mem.r11;

	if(side)
	{
		r11 |= R11_hilo_side; // High side injection
	}
	else
	{
		r11 &= ~(R11_hilo_side); // Low side injection
	}

	res = Ar1010Write(R_11, &r11, 2);
	if(res < 0)
	{
		printf("Set High/Low Side fail!\r\n");
		return res;
	}

	ar1010.mem.r11 = r11;

	return res;
}

// hiloctrl_b1,2를 설정해야하는 데 아직 이 기능에 대해 이해하지 못 했으며 해당 비트가 어떤 값을 설정하는 건지 파악하지 못 함(수정 필요)

#define AR1010_GPIO3	2
#define AR1010_GPIO2	1
#define AR1010_GPIO1	0

#define AR1010_GPIO_MASK(port)	(0x0003 << (2 * (port)))

#define GPIO_DISABLE		0X0000
#define GPIO_FUNC_1(port)	(0x0001 << (2 * (port)))
#define GPIO_LOW(port)		(0x0002 << (2 * (port)))
#define GPIO_HIGH(port)		(0x0003 << (2 * (port)))

// #define GPIO3_STEREO_IND	0x0010
// #define GPIO2_INTERRUPT		0x0040

// R_13 function
int SetGpio(unsigned char port, int func)
{
	char res = 0;

	if(port > AR1010_GPIO3)
	{
		printf("Unknown GPIO Port!\r\n");
		res = -1;
		return res;
	}

	unsigned short r13 = ar1010.mem.r13;

	r13 &= ~(AR1010_GPIO_MASK(port));

	switch(func)
	{
		case 0:
			r13 |= GPIO_DISABLE;
			break;
		case 1:
			r13 |= GPIO_FUNC_1(port);
			break;
		case 2:
			r13 |= GPIO_LOW(port);
			break;
		case 3:
			r13 |= GPIO_HIGH(port);
			break;
		default:
			printf("Unknown select value!\r\n");
			res = -1;
			return res;
	}

	res = Ar1010Write(R_13, &r13, 2);
	if(res < 0)
	{
		printf("Select GPIO3 function fail!\r\n");
		return res;
	}

	ar1010.mem.r13 = r13;

	return res;
}

#define AR1010_SET_VOLUME1(vol)	((vol) & 0x0F)
#define AR1010_SET_VOLUME2(vol)	(((vol) & 0xF0) << 4)

// R_14 function
// 여기다가 VOLUME 설정 함수를 구현하자.(수정 필요)
int SetVolume(unsigned short volume)
{
	char res = 0;
	unsigned short r3 = ar1010.mem.r3;
	unsigned short r14 = ar1010.mem.r14;

	r3 &= ~(R3_VOLUME);
	r14 &= ~(R14_VOLUME2);

	r3 |= AR1010_SET_VOLUME1(volume);
	r14 |= AR1010_SET_VOLUME2(volume);

	res = Ar1010Write(R_3, &r3, 2);
	if(res < 0)
	{
		printf("Set Volume1 fail\r\n");
		return res;
	}
	
	ar1010.mem.r3 = r3;
	
	res = Ar1010Write(R_14, &r14, 2);
	if(res < 0)
	{
		printf("Set Volume2 fail\r\n");
		return res;
	}

	ar1010.mem.r14 = r14;

	return res;
}

// Set AR1010 specific register
int SetAr1010Reg(unsigned char reg, unsigned short val)
{
	char res = 0;
	// unsigned short data = val;
	
	res = Ar1010Write(reg, &val, 2);
	if(res < 0)
	{
		printf("Set 0x%02X Register fail!\r\n", reg);
		return res;
	}

	ar1010.wbuff[reg] = val;

	return res;
}

// Set AR1010 specific bit of specific register
int SetAr1010Bit(unsigned char reg, unsigned char bit, unsigned short val)
{
	char res = 0;
	unsigned short rx = ar1010.wbuff[reg];

	rx &= ~(bit);
	rx |= val;

	res = Ar1010Write(reg, &rx, 2);
	if(res < 0)
	{
		printf("Set 0x%04X Bit of 0x%02X Register fail!\r\n");
		return res;
	}

	ar1010.wbuff[reg] = rx;

	return res;
}
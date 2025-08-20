#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h> // usleep
#include <stdlib.h>

// AR1010 Address
#define AR1010_ADDR	0x10

// AR1010 Register
#define AR1010_REG0		0x00	// xo_en, ENABLE
#define AR1010_REG1		0x01	// stc_int_en, deemp, mono, smute, hmute
#define AR1010_REG2		0x02	// TUNE, CHAN
#define AR1010_REG3		0x03	// SEEKUP, SEEK, SPACE, BAND, VOLUME
#define AR1010_REG10		0x0A	// seek_wrap
#define AR1010_REG11		0x0B	// hilo_side, hiloctrl_b1, hiloctrl_b2
#define AR1010_REG13		0x0C	// AR_GPIO3, AR_GPIO2, AR_GPIO1
#define AR1010_REG14		0x0D	// VOLUEM2
#define AR1010_REG15		0x0E	// RDSs
#define AR1010_REG_SSI		0x12	// RSSI, IF_CNT
#define AR1010_REG_STATUS	0x13 	// READCHAN, STC, SF, ST
#define AR1010_REG_DEVID	0x1B	// VERSION, MFID
#define AR1010_REG_CHIPID	0x1C	// CHIPID

// AR1010 R_0 BIT
#define AR1010_R0_XO_EN_MASK	0x0080
#define AR1010_R0_ENABLE_MASK	0x0001

// AR1010 R_1 BIT
#define AR1010_R1_STC_INT_EN_MASK	0x0020
#define AR1010_R1_DEEMP_MASK		0x0010
#define AR1010_R1_MONO_MASK		0x0008
#define AR1010_R1_SMUTE_MASK		0x0004
#define AR1010_R1_HMUTE_MASK		0x0002

// AR1010 R_2 BIT
#define AR1010_R2_TUNE_MASK		0x0200
#define AR1010_R2_CHAN_MASK		0x01FF
#define AR1010_R2_CHAN_SHIFT		0

// AR1010 R_3 BIT
#define AR1010_R3_SEEKUP_MASK	0x8000
#define AR1010_R3_SEEK_MASK	0x4000
#define AR1010_R3_SPACE_MASK	0x2000
#define AR1010_R3_BAND_MASK	0x1800
#define AR1010_R3_BAND_SHIFT	11
#define AR1010_R3_VOLUME_MASK	0x0780
#define AR1010_R3_VOLUME_SHIFT	7
#define AR1010_R3_SEEKTH_MASK	0x007F
#define AR1010_R3_SEEKTH_SHIFT	0

// AR1010 R_10 BIT
#define AR1010_R10_SEEK_WRAP_MASK	0x0008	
#define AR1010_R10_SEEK_WRAP_SHIFT	3	

// AR1010 R_11 BIT
#define AR1010_R11_HILO_SIDE_MASK	0x8000
#define AR1010_R11_HILO_SIDE_SHIFT	15
#define AR1010_R11_HILOCTRL_B1_MASK	0x0004
#define AR1010_R11_HILOCTRL_B1_SHIFT	3
#define AR1010_R11_HILOCTRL_B2_MASK	0x0001
#define AR1010_R11_HILOCTRL_B2_SHIFT	0

// AR1010 R_13 BIT
#define AR1010_R13_GPIO3_MASK	0x0030
#define AR1010_R13_GPIO3_SHIFT	4
#define AR1010_R13_GPIO2_MASK	0x000C
#define AR1010_R13_GPIO2_SHIFT	2
#define AR1010_R13_GPIO1_MASK	0x0003
#define AR1010_R13_GPIO1_SHIFT	0

// AR1010 R_14 BIT
#define AR1010_R14_VOLUME2_MASK		0xF000
#define AR1010_R14_VOLUME2_SHIFT	12


// AR1010 R_SSI BIT
#define AR1010_RSSI_RSSI_MASK		0xFE00
#define AR1010_RSSI_RSSI_SHIFT		9
#define AR1010_RSSI_IF_CNT_MASK		0x01FF
#define AR1010_RSSI_IF_CNT_SHIFT	0

// AR1010 R_STATUS BIT
#define AR1010_RSTATUS_READCHAN_MASK	0xFF80
#define AR1010_RSTATUS_READCHAN_SHIFT	7
#define AR1010_RSTATUS_STC_MASK		0x0020
#define AR1010_RSTATUS_STC_SHIFT	5
#define AR1010_RSTATUS_SF_MASK		0x0010
#define AR1010_RSTATUS_SF_SHIFT		4
#define AR1010_RSTATUS_ST_MASK		0x0008
#define AR1010_RSTATUS_ST_SHIFT		3

// AR1010 R_DEVICE BIT
#define AR1010_RDEVICE_VERSION_MASK	0xF000
#define AR1010_RDEVICE_VERSION_SHIFT	12
#define AR1010_RDEVICE_MFID_MASK	0x0FFF
#define AR1010_RDEVICE_MFID_SHIFT	0

// AR1010 R_CHIPID BIT
#define AR1010_RCHIPID_CHIPNO	0xFFFF

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

typedef enum
{
	AR1010_OK = 0,
	AR1010_EIO = -1,
	AR1010_EINVAL = -2,
	AR1010_ETIMEOUT = -3,
} ar1010_err_t;


/**
 * @brief AR1010 쓰기 함수
 * @param reg AR1010의 레지스터 주소 값
 * @param value reg에 저장할 값
 * @param vlaueLength value의 길이 (uint8_t 기준)
 * @return 0(성공) / 음수(실패)
 */
int Ar1010Write(const uint8_t reg, uint16_t* value, uint32_t valueLength)
{
	uint8_t* writeBuf = NULL;
	int ret = AR1010_OK;

	// 버퍼 생성
	writeBuf = (uint8_t*)malloc(sizeof(uint8_t) * valueLength);
	if(writeBuf == NULL)
	{
		printf("malloc fail in Ar1010Write function!\r\n");
		ret = AR1010_EIO;
		return ret;
	}

	writeBuf[0] = reg;
	memcpy(&writeBuf[1], (uint8_t*)value, valueLength);

	// I2C Transmit to AR1010
	ret = iic_write(AR1010_ADDR, writeBuf, valueLength);
	if(ret < 0)
	{
		printf("AR1010 I2C Write fail!\r\n");
		ret = AR1010_EIO;
	}

	free(writeBuf);

	return ret;
}


/**
 * @brief AR1010 읽기 함수
 * @param reg AR1010의 레지스터 주소 값
 * @param readBuff reg의 값을 저장할 버퍼
 * @param readbBuffLength reg에서 부터 읽어올 데이터의 길이 (uint8_t 기준)
 * @return 0(성공) / 음수(실패)
 */
int Ar1010Read(const uint8_t reg, uint16_t* readBuff, uint32_t readbBuffLength)
{
	int ret = AR1010_OK;

	if(readBuff == NULL)
	{
		printf("Buffer is NULL!\r\n");
		ret = AR1010_EINVAL;
		return ret;
	}

	/*
	// Set AR1010 Register for Read	
	ret = iic_write(AR1010_ADDR, &reg, sizeof(reg));
	if(ret < 0)
	{
		printf("AR1010 I2C Write for Read fail!\r\n");
		ret = AR1010_EIO;
		return ret;
	}

	// Read reg Register of AR1010
	ret = iic_read(AR1010_ADDR, (uint8_t*)buff, 2);
	if(ret < 0)
	{
		printf("AR1010 I2C Read fail!\r\n");
		ret = AR1010_EIO;
	}
	*/

	ret = iic_read_reg(AR1010_ADDR, reg, (uint8_t*)readBuff, readbBuffLength);
	if(ret < 0)
	{
		printf("AR1010 I2C Read fail!\r\n");
		ret = AR1010_EIO;
	}

	return ret;
}

// AR1010 Default Register Value
#define AR1010_WR_REG_SIZE	18

const unsigned short ar1010DefaultRegValIn[AR1010_WR_REG_SIZE] = {
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
const unsigned short ar1010DefaultRegValEx[AR1010_WR_REG_SIZE] = {
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


#define AR1010_STC_TIMEOUT_MS	50
/**
 * @brief STC 플래그 확인 함수
 * @param timeout STC 플래그 확인 시간의 최대값
 * @return 0(성공) / 음수(실패)
 */
int Ar1010WaitStc(int timeout)
{
	const uint32_t step = 1000;
	uint32_t ret = AR1010_ETIMEOUT;
	uint16_t stc = 0;

	while(timeout--)
	{
		// Read Status Register
		Ar1010Read(AR1010_REG_STATUS, &stc, 2);

		// for debug
		// printf("AR1010 STATUS REG: 0x%04D\n\r");
		
		// Masking STC
		stc &= AR1010_RSTATUS_STC_MASK;

		// Check STC flag
		if(stc)
		{
			ret = AR1010_OK;
			break;
		}

		usleep(step);
	}

	return ret;
}

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
	unsigned short wbuff[AR1010_WR_REG_SIZE];
} ar1010R_t;

ar1010R_t ar1010;

int Ar1010Update()
{
	int ret = AR1010_OK;
	uint16_t rx = 0;

	// Get RSSI Bits for signal strength
	ret = Ar1010Read(AR1010_REG_STATUS, &ar1010.rbuff[AR1010_REG_STATUS], 4);
	if(ret < 0)
		printf("AR1010 RSSI/RSTATUS Update fail!\r\n");

	return ret;
}


#define BAND_US_EU	0x0000
#define BAND_JP		0x1000
#define BAND_JP_EX	0x1800

#define US_EU_MIN_FREQ	87.5
#define US_EU_MAX_FREQ	108.0
#define JP_MIN_FREQ		76.0
#define JP_MAX_FREQ		90.0
#define JP_EX_MIN_FREQ	76.0
#define JP_EX_MAX_FREQ	108.0

int Ar1010Tune(/*uint16_t band, uint16_t space, */double freq)
{
	int ret = AR1010_OK;
	double maxFreq = US_EU_MAX_FREQ;
	double minFreq = US_EU_MIN_FREQ;

	// 현재 band의 주파수 범위에 포함되는 주파수를 설정했는지 확인
	uint16_t rx = ar1010.mem.r3 & AR1010_R3_BAND_MASK;
	switch (rx)
	{
	case BAND_JP:
		maxFreq = JP_MAX_FREQ;
		minFreq = JP_MIN_FREQ;
		break;
	case BAND_JP_EX:
		maxFreq = JP_EX_MAX_FREQ;
		minFreq = JP_EX_MIN_FREQ;
		break;
	case BAND_US_EU:
		break;
	default:
		printf("Unknown Band!\r\n");
		break;
	}

	if(freq < minFreq || freq > maxFreq)
	{
		printf("Invalid argument!\r\n");
		ret = AR1010_EINVAL;
		return ret;
	}

	// 주파수 설정을 위한 CHAN 값 계산
	uint16_t chan = (uint16_t)((freq - 69) * 10);

	// Set HMUTE Bit
	rx = ar1010.mem.r1 | AR1010_R1_HMUTE_MASK;
	ret = Ar1010Write(AR1010_REG1, &rx, 2);
	if(ret < 0)
	{
		printf("AR1010 TUNE fail!\r\n");
		return ret;
	}

	ar1010.mem.r1 = rx;

	// Clear TUNE Bit
	rx = ar1010.mem.r2 & (~AR1010_R2_TUNE_MASK);
	ret = Ar1010Write(AR1010_REG2, &rx, 2);
	if(ret < 0)
	{
		printf("AR1010 TUNE fail!\r\n");
		return ret;
	}

	ar1010.mem.r2 = rx;

	// Clear SEEK Bit
	rx = ar1010.mem.r3 & (~AR1010_R3_SEEK_MASK);
	ret = Ar1010Write(AR1010_REG3, &rx, 2);
	if(ret < 0)
	{
		printf("AR1010 TUNE fail!\r\n");
		return ret;
	}

	ar1010.mem.r3 = rx;

	// Set BAND/SPACE/CHAN Bits
	/*
	rx = ar1010.mem.r3 & ~(AR1010_R3_BAND_MASK | AR1010_R3_SPACE_MASK);
	rx |= band | space;
	ret = Ar1010Write(AR1010_REG3, &rx, 2);
	if(ret < 0)
	{
		printf("AR1010 TUNE fail!\r\n");
		return ret;
	}

	ar1010.mem.r3 = rx;
	*/
	rx = ar1010.mem.r2 & (~AR1010_R2_CHAN_MASK);
	rx |= chan;
	ret = Ar1010Write(AR1010_REG2, &rx, 2);
	if(ret < 0)
	{
		printf("AR1010 TUNE fail!\r\n");
		return ret;
	}

	ar1010.mem.r2 = rx;

	// Enable TUNE Bit
	rx |= AR1010_R2_TUNE_MASK;
	ret = Ar1010Write(AR1010_REG2, &rx, 2);
	if(ret < 0)
	{
		printf("AR1010 TUNE fail!\r\n");
		return ret;
	}

	ar1010.mem.r2 = rx;

	// Wait STC flag
	ret = Ar1010WaitStc(AR1010_STC_TIMEOUT_MS);
	if(ret < 0)
	{
		printf("AR1010 TUNE Timeout!\r\n");
		return ret;
	}

	// Clear HMUTE Bit
	rx = ar1010.mem.r1 & (~AR1010_R1_HMUTE_MASK);
	ret = Ar1010Write(AR1010_REG1, &rx, 2);
	if(ret < 0)
	{
		printf("AR1010 TUNE fail!\r\n");
		return ret;
	}
	
	// Update Functions (optional)
	Ar1010Update();

	return ret;
}

int Ar1010HiloTune(double freq)
{
	int ret = AR1010_OK;
	double maxFreq = US_EU_MAX_FREQ;
	double minFreq = US_EU_MIN_FREQ;
	uint16_t loRssi = 0;
	uint16_t hiRssi = 0;

	// 현재 band의 주파수 범위에 포함되는 주파수를 설정했는지 확인
	uint16_t rx = ar1010.mem.r3 & AR1010_R3_BAND_MASK;
	switch (rx)
	{
	case BAND_JP:
		maxFreq = JP_MAX_FREQ;
		minFreq = JP_MIN_FREQ;
		break;
	case BAND_JP_EX:
		maxFreq = JP_EX_MAX_FREQ;
		minFreq = JP_EX_MIN_FREQ;
		break;
	case BAND_US_EU:
		break;
	default:
		printf("Unknown Band!\r\n");
		break;
	}

	if(freq < minFreq || freq > maxFreq)
	{
		printf("Invalid argument!\r\n");
		ret = AR1010_EINVAL;
		return ret;
	}

	uint16_t chan = (uint16_t)((freq - 69) * 10);

	// Set HMUTE Bit
	rx = ar1010.mem.r1 | AR1010_R1_HMUTE_MASK;
	ret = Ar1010Write(AR1010_REG1, &rx, 2);
	if(ret < 0)
	{
		printf("AR1010 TUNE fail!\r\n");
		return ret;
	}

	// Clear TUNE Bit
	rx = ar1010.mem.r2 & (~AR1010_R2_TUNE_MASK);
	ret = Ar1010Write(AR1010_REG2, &rx, 2);
	if(ret < 0)
	{
		printf("AR1010 TUNE fail!\r\n");
		return ret;
	}

	ar1010.mem.r2 = rx;

	// Clear SEEK Bit
	rx = ar1010.mem.r3 & (~AR1010_R3_SEEK_MASK);
	ret = Ar1010Write(AR1010_REG3, &rx, 2);
	if(ret < 0)
	{
		printf("AR1010 TUNE fail!\r\n");
		return ret;
	}

	ar1010.mem.r3 = rx;

	// Set BAND/SPACE/CHAN Bits
	rx = ar1010.mem.r2 & (~AR1010_R2_CHAN_MASK);

	rx |= chan;
	ret = Ar1010Write(AR1010_REG2, &rx, 2);
	if(ret < 0)
	{
		printf("AR1010 TUNE fail!\r\n");
		return ret;
	}

	ar1010.mem.r2 = rx;

	// Set R11 (clear hiloside, clear hiloctrl_b1/2)
	rx = ar1010.mem.r11 & ~(AR1010_R11_HILO_SIDE_MASK | AR1010_R11_HILOCTRL_B1_MASK | AR1010_R11_HILOCTRL_B2_MASK);
	ret = Ar1010Write(AR1010_REG2, &rx, 2);
	if(ret < 0)
	{
		printf("AR1010 TUNE fail!\r\n");
		return ret;
	}

	ar1010.mem.r11 = rx;

	// Enable TUNE Bit
	rx = ar1010.mem.r2 | AR1010_R2_TUNE_MASK;
	ret = Ar1010Write(AR1010_REG2, &rx, 2);
	if(ret < 0)
	{
		printf("AR1010 TUNE fail!\r\n");
		return ret;
	}

	ar1010.mem.r2 = rx;

	// Wait STC flag
	ret = Ar1010WaitStc(AR1010_STC_TIMEOUT_MS);
	if(ret < 0)
	{
		printf("AR1010 TUNE Timeout!\r\n");
		return ret;
	}

	// Get RSSI (RSSI1)
	ret = Ar1010Read(AR1010_REG_SSI, &loRssi, 2);
	if(ret < 0)
	{
		printf("AR1010 TUNE fail!\r\n");
		return ret;
	}
	loRssi &= AR1010_RSSI_RSSI_MASK;

	// Clear TUNE Bit
	rx = ar1010.mem.r2 & (~AR1010_R2_TUNE_MASK);
	ret = Ar1010Write(AR1010_REG2, &rx, 2);
	if(ret < 0)
	{
		printf("AR1010 TUNE fail!\r\n");
		return ret;
	}

	ar1010.mem.r2 = rx;

	// Set R11 (set hiloside, set hiloctrl_b1/2)
	rx = ar1010.mem.r11 | (AR1010_R11_HILO_SIDE_MASK | AR1010_R11_HILOCTRL_B1_MASK | AR1010_R11_HILOCTRL_B2_MASK);
	ret = Ar1010Write(AR1010_REG2, &rx, 2);
	if(ret < 0)
	{
		printf("AR1010 TUNE fail!\r\n");
		return ret;
	}

	ar1010.mem.r11 = rx;

	// Enable TUNE Bit
	rx = ar1010.mem.r2 | AR1010_R2_TUNE_MASK;
	ret = Ar1010Write(AR1010_REG2, &rx, 2);
	if(ret < 0)
	{
		printf("AR1010 TUNE fail!\r\n");
		return ret;
	}

	ar1010.mem.r2 = rx;

	// Wait STC flag
	ret = Ar1010WaitStc(AR1010_STC_TIMEOUT_MS);
	if(ret < 0)
	{
		printf("AR1010 TUNE Timeout!\r\n");
		return ret;
	}

	// Get RSSI (RSSI2)
	ret = Ar1010Read(AR1010_REG_SSI, &hiRssi, 2);
	if(ret < 0)
	{
		printf("AR1010 TUNE fail!\r\n");
		return ret;
	}
	hiRssi &= AR1010_RSSI_RSSI_MASK;

	// Clear TUNE Bit
	rx = ar1010.mem.r2 & (~AR1010_R2_TUNE_MASK);
	ret = Ar1010Write(AR1010_REG2, &rx, 2);
	if(ret < 0)
	{
		printf("AR1010 TUNE fail!\r\n");
		return ret;
	}

	ar1010.mem.r2 = rx;

	// Compare Hi/Lo Side Signal Strength
	if(loRssi > hiRssi)
	{
		// Set R11 (clear hiloside, clear hiloctrl_b1/2)
		rx = ar1010.mem.r11 & ~(AR1010_R11_HILO_SIDE_MASK | AR1010_R11_HILOCTRL_B1_MASK | AR1010_R11_HILOCTRL_B2_MASK);
		ret = Ar1010Write(AR1010_REG2, &rx, 2);
		if(ret < 0)
		{
			printf("AR1010 TUNE fail!\r\n");
			return ret;
		}
	}
	else
	{

	}

	// Enable TUNE Bit
	rx = ar1010.mem.r2 | AR1010_R2_TUNE_MASK;
	ret = Ar1010Write(AR1010_REG2, &rx, 2);
	if(ret < 0)
	{
		printf("AR1010 TUNE fail!\r\n");
		return ret;
	}

	ar1010.mem.r2 = rx;

	// Wait STC flag
	ret = Ar1010WaitStc(AR1010_STC_TIMEOUT_MS);
	if(ret < 0)
	{
		printf("AR1010 TUNE Timeout!\r\n");
		return ret;
	}

	// Clear TUNE Bit
	rx = ar1010.mem.r2 & (~AR1010_R2_TUNE_MASK);
	ret = Ar1010Write(AR1010_REG2, &rx, 2);
	if(ret < 0)
	{
		printf("AR1010 TUNE fail!\r\n");
		return ret;
	}

	// Update Functions (optional)
	Ar1010Update();

	return ret;
}

int Ar1010Init()
{
	uint32_t ret = AR1010_OK;

	// for stable
	usleep(100);

	// Set R1 to R17 Registers to default value
	/*
	for(int reg = AR1010_REG1; reg < AR1010_WR_REG_SIZE; reg++)
	{
		ret = Ar1010Write(reg, ar1010DefaultRegValEx[reg], 2);
		if(ret < 0)
		{
			printf("AR1010 Initialize fail!\r\n");
			return ret;
		}
	}
	*/

	// Set R0 Register to default value
	/*
	ret = Ar1010Write(AR1010_REG0, ar1010DefaultRegValEx[AR1010_REG0], 2);
	if(ret < 0)
	{
		printf("AR1010 Initialize fail!\r\n");
		return ret;
	}
	*/

	ret = Ar1010Write(AR1010_REG1, ar1010DefaultRegValEx[AR1010_REG1], AR1010_WR_REG_SIZE);
	if(ret < 0)
	{
		printf("AR1010 Initialize fail!\r\n");
		return ret;
	}


	// Wait STC flag
	ret = Ar1010WaitStc(AR1010_STC_TIMEOUT_MS);
	if(ret < 0)
	{
		printf("AR1010 Initialize STC Timeout!\r\n");
		return ret;
	}

	// TUNING
	ret = Ar1010HiloTune(87.5);
	ret = Ar1010Tune(87.5);
	if(ret < 0)
	{
		printf("TUNE fail while AR1010 Initializing!\r\n");
		return ret;
	}

	// Wait STC flag
	/*
	ret = Ar1010WaitStc(AR1010_STC_TIMEOUT_MS);
	if(ret < 0)
	{
		printf("AR1010 Initialize TUNE STC Timeout!\r\n");
		return ret;
	}
	*/

	return ret;
}

int Ar1010Seek(/*uint16_t band, uint16_t space, */)
{
	int ret = AR1010_OK;
	uint16_t chan = 0;

	// Set HMUTE Bit
	uint16_t rx = ar1010.mem.r1 | AR1010_R1_HMUTE_MASK;
	ret = Ar1010Write(AR1010_REG1, &rx, 2);
	if(ret < 0)
	{
		printf("AR1010 SEEK fail!\r\n");
		return ret;
	}

	// Clear TUNE Bit
	rx = ar1010.mem.r2 & (~AR1010_R2_TUNE_MASK);
	ret = Ar1010Write(AR1010_REG2, &rx, 2);
	if(ret < 0)
	{
		printf("AR1010 SEEK fail!\r\n");
		return ret;
	}

	ar1010.mem.r2 = rx;

	//Set CHAN Bit

	// Clear SEEK Bit
	rx = ar1010.mem.r3 & (~AR1010_R3_SEEK_MASK);
	ret = Ar1010Write(AR1010_REG3, &rx, 2);
	if(ret < 0)
	{
		printf("AR1010 SEEK fail!\r\n");
		return ret;
	}

	ar1010.mem.r3 = rx;

	// Set SEEKUP/SPACE/BAND/SEEKTH Bits
	/*
	rx = ar1010.mem.r3 & ~(AR1010_R3_BAND_MASK | AR1010_R3_SPACE_MASK);
	rx |= band | space;
	ret = Ar1010Write(AR1010_REG3, &rx, 2);
	if(ret < 0)
	{
		printf("AR1010 TUNE fail!\r\n");
		return ret;
	}

	ar1010.mem.r3 = rx;
	*/

	// Enable SEEK Bit
	rx |= AR1010_R3_SEEK_MASK;
	ret = Ar1010Write(AR1010_REG3, &rx, 2);
	if(ret < 0)
	{
		printf("AR1010 SEEK fail!\r\n");
		return ret;
	}

	ar1010.mem.r2 = rx;

	// Wait STC flag
	ret = Ar1010WaitStc(AR1010_STC_TIMEOUT_MS);
	if(ret < 0)
	{
		printf("AR1010 SEEK Timeout!\r\n");
		return ret;
	}

	// Clear HMUTE Bit
	rx = ar1010.mem.r1 & (~AR1010_R1_HMUTE_MASK);
	ret = Ar1010Write(AR1010_REG1, &rx, 2);
	if(ret < 0)
	{
		printf("AR1010 SEEK fail!\r\n");
		return ret;
	}

	ar1010.mem.r1 = rx;
	
	// Update Functions (optional, but remember to update CHAN with the seek result in READCHAN before next seek)
	Ar1010Update();

	chan = (ar1010.mem.rstatus & AR1010_RSTATUS_READCHAN_MASK) >> AR1010_RSTATUS_READCHAN_SHIFT;

	if(chan != (ar1010.mem.r2 & AR1010_R2_CHAN_MASK))
	{
		ar1010.mem.r2 &= AR1010_R2_CHAN_MASK;
		ar1010.mem.r2 |= chan;
	}

	return ret;
}
































#define MAX_TIMEOUT	15000
int xo_en = 0;


sem_t ar1010_sem;

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
	sem_wait(&ar1010_sem);

	int res = 0;
	unsigned short r2 = ar1010.mem.r2;
	char stc = 0;
	int timeout = 0;

	// Ar1010Read(R_2/*, &reg2, sizeof(reg2)*/);

	r2 &= ~(R2_TUNE);

	res = Ar1010Write(R_2, &r2, 2);
	if(res < 0)
	{
		printf("Reset TUNE bit fail!\r\n");
		sem_post(&ar1010_sem);
		return res;
	}

	ar1010.mem.r2 = r2;
	r2 |= R2_TUNE;

	res = Ar1010Write(R_2, &r2, 2);
	if(res < 0)
	{
		printf("Set TUNE bit fail!\r\n");
		sem_post(&ar1010_sem);
		return res;
	}

	ar1010.mem.r2 = r2;

	do
	{
		if(timeout > MAX_TIMEOUT)
		{
			printf("Tune STC Timeout!\r\n");
			res = -1;
			break;
		}

		Ar1010Read(R_STATUS);
		stc = ar1010.mem.rstatus & RSTATUS_STC ? 1 : 0;

		timeout++;
	}while(stc == 0);

	sem_post(&ar1010_sem);
	return res;
}

// AR1010 initialize sequence
int Ar1010Init()
{
	int res = 0;
	// unsigned short status = 0;
	char stc = 0;
	int timeout = 0;

	sem_init(&ar1010_sem, 0, 1);
	sem_wait(&ar1010_sem);
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
				sem_post(&ar1010_sem);
				res = -1;
				return res;
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
				sem_post(&ar1010_sem);
				res = -1;
				return res;
			}
			ar1010.wbuff[reg] = ar1010DefualtRegValEx[reg];
		}	
	}

	// Check STC Flag
	do
	{
		if(timeout > MAX_TIMEOUT)
		{
			printf("AR1010 Initailize STC Timeout!\r\n");
			res = -1;
			break;
		}

		Ar1010Read(R_STATUS/*, &status, 2*/);
		stc = (ar1010.mem.rstatus & RSTATUS_STC) ? 1 : 0;

		break;
	}while(stc == 0);

	// TUNING
	// semaphore problem!!
	if(res >= 0)
	{
		sem_post(&ar1010_sem);
		Ar1010Tune();
		sem_wait(&ar1010_sem);
	}

	sem_post(&ar1010_sem);

	return 0;
}

// R_0 functions
int SetXoEnable(int enable)
{
	sem_wait(&ar1010_sem);

	int res = 0;
	unsigned short r0 = ar1010.mem.r0;

	if (enable)
	{
		r0 |= R0_xo_en; // using internal crystal oscillator
	}
	else
	{
		r0 &= ~(R0_xo_en); // using external reference clock
	}

	res = Ar1010Write(R_0, &r0, 2);
	if(res < 0)
	{
		printf("xo_en config fail!\r\n");
		sem_post(&ar1010_sem);
		return res;
	}
	
	ar1010.mem.r0 = r0;

	sem_post(&ar1010_sem);
	return res;
}

int SetSleep(int enable)
{
	sem_wait(&ar1010_sem);

	int res = 0;
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
		sem_post(&ar1010_sem);
		return res;
	}
	
	ar1010.mem.r0 = r0;

	sem_post(&ar1010_sem);

	return res;
}

// R1
int setStcIntEn(int enable)
{
	sem_wait(&ar1010_sem);

	int res = 0;
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
		sem_post(&ar1010_sem);
		return res;
	}

	ar1010.mem.r1 = r1;

	sem_post(&ar1010_sem);

	return res;
}

int SetDeemp(int select)
{
	sem_wait(&ar1010_sem);

	int res = 0;
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
			sem_post(&ar1010_sem);
			return res;
	}

	res = Ar1010Write(R_1, &r1, 2);
	if(res < 0)
	{
		printf("set deemp fail!\r\n");
		sem_post(&ar1010_sem);
		return res;
	}

	ar1010.mem.r1 = r1;

	sem_post(&ar1010_sem);

	return res;
}

int SetMono(int enable)
{
	sem_wait(&ar1010_sem);

	int res = 0;
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
		sem_post(&ar1010_sem);
		return res;
	}

	ar1010.mem.r1 = r1;

	sem_post(&ar1010_sem);

	return res;
}

int SetSmute(int enable)
{
	sem_wait(&ar1010_sem);

	int res = 0;
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
		sem_post(&ar1010_sem);
		return res;
	}

	ar1010.mem.r1 = r1;

	sem_post(&ar1010_sem);

	return res;
}

int SetHmute(int enable)
{
	sem_wait(&ar1010_sem);

	int res = 0;
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
		sem_post(&ar1010_sem);
		return res;
	}

	ar1010.mem.r1 = r1;

	sem_post(&ar1010_sem);

	return res;
}

// R_2 functions
// int Ar1010Tuning(double freq)
#define BAND_US_EU	0x0000
#define BAND_JP		0x1000
#define BAND_JP_EX	0x1800
int SetChannel(double freq)
{
	sem_wait(&ar1010_sem);

	int res = 0;
	// BAND 값에 따라 허용 범위가 달라져야 함 (수정 필요)
	unsigned short band = ar1010.mem.r3 & R3_BAND;
	switch(band)
	{
		case BAND_US_EU:
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
			break;
		case BAND_JP:
			if(freq < 76.0)
			{
				freq = 76.0;
			}
			else if(freq > 90.0)
			{
				freq = 90.0;
			}
			else
			{}
			break;
		case BAND_JP_EX:
			if(freq < 76.0)
			{
				freq = 76.0;
			}
			else if(freq > 108.0)
			{
				freq = 108.0;
			}
			else
			{}
			break;
		default:
			printf("Band Error in SetChannel Function!\r\n");
			sem_post(&ar1010_sem);
			res = -1;
			return res;
	}

	unsigned short chan = (freq - 69) * 10;

	res = Ar1010Write(R_2, &chan, 2);
	if(res < 0)
	{
		printf("Set Channel fail!\r\n");
		sem_post(&ar1010_sem);
		return res;
	}

	ar1010.mem.r2 &= ~(R2_CHAN);
	ar1010.mem.r2 |= chan;

	sem_post(&ar1010_sem);

	Ar1010Tune();

	return 0;
}

//R_3 functions
int SetSeekUpDown(int upDown)
{
	sem_wait(&ar1010_sem);

	int res = 0;
	unsigned short r3 = ar1010.mem.r3;

	if(upDown)
	{
		r3 |= R3_SEEKUP; // Seek Up
	}
	else
	{
		r3 &= ~(R3_SEEKUP); // Seek Down
	}

	res = Ar1010Write(R_3, &r3, 2);
	if(res < 0)
	{
		printf("Set Seek Up/Down fail!\r\n");
		sem_post(&ar1010_sem);
		return res;
	}
	
	sem_post(&ar1010_sem);

	return res;
}

int Ar1010Seek()
{
	sem_wait(&ar1010_sem);

	int res = 0;
	unsigned short r3 = ar1010.mem.r3;
	char stc = 0;
	int timeout = 0;

	r3 &= ~(R3_SEEK);
	res = Ar1010Write(R_3, &r3, 2);
	if(res < 0)
	{
		printf("fail to Reset SEEK bit!\r\n");
		sem_post(&ar1010_sem);
		return res;
	}
	
	printf("Reset SEEK!\r\n");

	r3 |= R3_SEEK;
	res = Ar1010Write(R_3, &r3, 2);
	if(res < 0)
	{
		printf("fail to Set SEEK bit!\r\n");
		sem_post(&ar1010_sem);
		return res;
	}
	
	printf("Set SEEK!\r\n");

	do
	{
		if(timeout > MAX_TIMEOUT)
		{
			printf("Seek STC Timeout!\r\n");
			res = -1;
			break;
		}
		
		Ar1010Read(R_STATUS);
		stc = ar1010.mem.rstatus & RSTATUS_STC ? 1 : 0;
		
		timeout++;
	} while(stc == 0);

	ar1010.mem.r3 = r3;
	
	sem_post(&ar1010_sem);

	return res;
}

int SetSpace(int select)
{
	sem_wait(&ar1010_sem);

	int res = 0;
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
			sem_post(&ar1010_sem);
			return res;
	}

	res = Ar1010Write(R_3, &r3, 2);
	if(res < 0)
	{
		printf("Set Space value fail!\r\n");
		sem_post(&ar1010_sem);
		return res;
	}

	ar1010.mem.r3 = r3;

	sem_post(&ar1010_sem);

	return res;
}

int SetBand(int select)
{
	sem_wait(&ar1010_sem);

	int res = 0;
	unsigned short r3 = ar1010.mem.r3;
	double nowFreq = 69.0 + 0.1 * (double)(ar1010.mem.r2 & R2_CHAN);
	double change = 0;

	r3 &= ~(R3_BAND);

	switch(select)
	{
		case 0:
		case 1:
			r3 |= BAND_US_EU; // Band -> 87.5 ~ 108MHz
			if(nowFreq < 87.5 || nowFreq > 108.0)
				change = 87.5;
			break;
		case 2:
			r3 |= BAND_JP; // Band -> 76 ~ 90MHz
			if(nowFreq < 76.0 || nowFreq > 90)
				change = 76.0;
			break;
		case 3:
			r3 |= BAND_JP_EX; // Band -> 76 ~ 108MHz
			if(nowFreq < 76.0 || nowFreq > 108.0)
				change = 87.5;
			break;
		default:
			printf("Unknown select value!\r\n");
			res = -1;
			sem_post(&ar1010_sem);
			return res;
	}

	res = Ar1010Write(R_3, &r3, 2);
	if(res < 0)
	{
		printf("Set Band fail!\r\n");
		sem_post(&ar1010_sem);
		return res;
	}

	ar1010.mem.r3 = r3;

	if(change != 0)
	{
		res = SetChannel(change);
		if(res < 0)
		{
			printf("Set Channel fail in SetBand Function!\r\n");
			sem_post(&ar1010_sem);
			return res;
		}
	}

	sem_post(&ar1010_sem);

	return res;
}

#define SEEKTH_MAX	127

// 입력 신호의 세기가 여기서 설정한 값 이상이어야 SEEK 동작을 수행한다.
int SetSeekThreshold(unsigned short threshold)
{
	sem_wait(&ar1010_sem);

	int res = 0;
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
		sem_post(&ar1010_sem);
		return res;
	}

	ar1010.mem.r3 = r3;

	sem_post(&ar1010_sem);

	return res;
}

// R_10 function
int SetSeekWrap(int enable)
{
	sem_wait(&ar1010_sem);

	int res = 0;
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
		sem_post(&ar1010_sem);
		return res;
	}

	ar1010.mem.r10 = r10;

	sem_post(&ar1010_sem);

	return res;
}

// R_11 functions
int SetHiloSide(int side)
{
	sem_wait(&ar1010_sem);

	int res = 0;
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
		sem_post(&ar1010_sem);
		return res;
	}

	ar1010.mem.r11 = r11;
	
	sem_post(&ar1010_sem);

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
	sem_wait(&ar1010_sem);

	int res = 0;

	if(port > AR1010_GPIO3)
	{
		printf("Unknown GPIO Port!\r\n");
		res = -1;
		sem_post(&ar1010_sem);
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
			sem_post(&ar1010_sem);
			return res;
	}

	res = Ar1010Write(R_13, &r13, 2);
	if(res < 0)
	{
		printf("Select GPIO3 function fail!\r\n");
		sem_post(&ar1010_sem);
		return res;
	}

	ar1010.mem.r13 = r13;

	sem_post(&ar1010_sem);

	return res;
}

#define AR1010_SET_VOLUME1(vol)	(((unsigned short)((vol) & 0x0F)) << 7)
#define AR1010_SET_VOLUME2(vol)	(((unsigned short)((vol) & 0xF0)) << 8)

// R_14 function
int SetVolume(unsigned short volume)
{
	sem_wait(&ar1010_sem);

	int res = 0;
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
		sem_post(&ar1010_sem);
		return res;
	}
	
	ar1010.mem.r3 = r3;
	
	res = Ar1010Write(R_14, &r14, 2);
	if(res < 0)
	{
		printf("Set Volume2 fail\r\n");
		sem_post(&ar1010_sem);
		return res;
	}

	ar1010.mem.r14 = r14;

	sem_post(&ar1010_sem);

	return res;
}

// Set AR1010 specific register
int SetAr1010Reg(unsigned char reg, unsigned short val)
{
	sem_wait(&ar1010_sem);

	int res = 0;
	// unsigned short data = val;
	
	res = Ar1010Write(reg, &val, 2);
	if(res < 0)
	{
		printf("Set 0x%02X Register fail!\r\n", reg);
		sem_post(&ar1010_sem);
		return res;
	}

	ar1010.wbuff[reg] = val;

	sem_post(&ar1010_sem);

	return res;
}

// Set AR1010 specific bit of specific register
int SetAr1010Bit(unsigned char reg, unsigned char bit, unsigned short val)
{
	sem_wait(&ar1010_sem);

	int res = 0;
	unsigned short rx = ar1010.wbuff[reg];

	rx &= ~(bit);
	rx |= val;

	res = Ar1010Write(reg, &rx, 2);
	if(res < 0)
	{
		printf("Set 0x%04X Bit of 0x%02X Register fail!\r\n", bit, reg);
		sem_post(&ar1010_sem);
		return res;
	}

	ar1010.wbuff[reg] = rx;

	sem_post(&ar1010_sem);

	return res;
}

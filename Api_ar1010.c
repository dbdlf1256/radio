#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h> // usleep
#include <stdlib.h>
#include <semaphore.h>

int iic_read_reg(unsigned char slaveAddress,unsigned char regAddr,unsigned char *readData,unsigned int readDataLength);
int iic_write(unsigned char slaveAddress, unsigned char *writeData,unsigned int writeDataLength);

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

// 조금 더 세분화할 필요가 있음
typedef enum
{
	AR1010_OK = 0,
	AR1010_EIO = -1,
	AR1010_EINVAL = -2,
	AR1010_ETIMEOUT = -3,
} ar1010_err_t;

// AR1010 Default Register Value
#define AR1010_WR_REG_SIZE	18
#define AR1010_RD_REG_SIZE	29

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
	unsigned short rbuff[AR1010_RD_REG_SIZE];
	unsigned short wbuff[AR1010_WR_REG_SIZE];
} ar1010R_t;

ar1010R_t ar1010;
sem_t ar1010_sem;

/**
 * @brief ar1010R_t 자료형의 값을 0으로 초기화
 * 
 * @param ar1010State 초기화할 ar1010R_t 자료형 변수의 포인터
 * @return int 0 반환
 */
int StructAr1010Init(ar1010R_t* ar1010State)
{
	sem_wait(&ar1010_sem);

	memset(ar1010State, 0, sizeof(ar1010R_t));
	
	sem_post(&ar1010_sem);

	return AR1010_OK;
}

/**
 * @brief 전역 변수 ar1010에 저장되어 있는 AR1010의 레지스터 값을 반환
 * 
 * @param reg 반환 받을 레지스터의 위치
 * @return uint16_t 전역 변수 ar1010에 저장되어 있는 AR1010의 레지스터 값을 반환
 */
uint16_t GetAr1010Reg(uint8_t reg)
{
	uint16_t regVal = 0;

	sem_wait(&ar1010_sem);

	regVal = ar1010.rbuff[reg];

	sem_post(&ar1010_sem);

	return regVal;
}

/**
 * @brief 전역 변수 ar1010에 값을 대입
 * 
 * @param reg 값을 저장할 레지스터의 위치
 * @param value 전역 변수 ar1010에 저장할 값
 * @return int 0 반환
 */
int SetAr1010Reg(uint8_t reg, uint16_t value)
{	
	sem_wait(&ar1010_sem);

	ar1010.wbuff[reg] = value;

	sem_post(&ar1010_sem);

	return AR1010_OK;
}

#define AR1010_WR_LENGTH(L)	(((L) * 2) + 1)

 /**
  * @brief 주어진 값을 주어진 길이 만큼 AR1010의 레지스터에 쓰고 전역 변수 ar1010의 값을 업데이트
  * 
  * @param reg 저장을 시작할 AR1010의 레지스터 주소 값
  * @param value 레지스터에 쓸 값의 포인터
  * @param valueLength 레지스터에 쓸 값의 길이 (uint16_t 기준)
  * @return int 0(성공) / 음수(실패)
  */
int Ar1010Write(const uint8_t reg, uint16_t* value, uint32_t valueLength)
{
	int ret = AR1010_OK;
	uint8_t* writeData = NULL;
	uint32_t writeLength = AR1010_WR_LENGTH(valueLength);
	int valIndex = 0;

	// 버퍼 생성
	writeData = (uint8_t*)malloc(writeLength);
	if(writeData == NULL)
	{
		printf("malloc fail in Ar1010Write function!\r\n");
		ret = AR1010_EIO;
		return ret;
	}

	writeData[0] = reg;
	// memcpy(&writeData[1], (uint8_t*)value, valueLength);
	for(int i = 1; i < writeLength; i++)
	{
		writeData[i] = ((uint8_t*)value)[i - 1];
	}

	// I2C Transmit to AR1010
	ret = iic_write(AR1010_ADDR, writeData, valueLength);
	if(ret < 0)
	{
		printf("AR1010 I2C Write fail!\r\n");
		// ret = AR1010_EIO;
	}

	sem_wait(&ar1010_sem);

	for(int r = reg; r < valueLength; r++)
		SetAr1010Reg(r, value[valIndex++]);

	sem_post(&ar1010_sem);

	free(writeData);

	return ret;
}

/**
 * @brief AR1010의 레지스터 값을 읽어 전역 변수 ar1010에 값을 저장
 * 
 * @param reg 읽기 시작할 AR1010 레지스터의 주소 값
 * @param readLength 읽을 레지스터의 길이 (uint16_t 기준)
 * @return int 0(성공) / 음수(실패)
 */
int Ar1010Read(const uint8_t reg, /*uint16_t* readBuff,*/ uint32_t readLength)
{
	int ret = AR1010_OK;
	uint32_t readLength = readLength * 2;

	/*
	if(readBuff == NULL)
	{
		printf("Buffer is NULL!\r\n");
		ret = AR1010_EINVAL;
		return ret;
	}

	
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

	sem_wait(&ar1010_sem);

	ret = iic_read_reg(AR1010_ADDR, reg, (uint8_t*)&ar1010.rbuff[reg], readLength);
	if(ret < 0)
	{
		printf("AR1010 I2C Read fail!\r\n");
		ret = AR1010_EIO;
	}

	sem_post(&ar1010_sem);

	return ret;
}


#define AR1010_STC_TIMEOUT_MS	50
/**
 * @brief AR1010의 STC 플래그 확인 함수
 * 
 * @param timeout STC 플래그 확인 시간의 최대값
 * @return int 0(성공) / 음수(실패)
 */
int Ar1010WaitStc(int timeout)
{
	const uint32_t step = 1000;
	uint32_t ret = AR1010_ETIMEOUT;
	uint16_t stc = 0;

	while(timeout--)
	{
		// Read Status Register
		Ar1010Read(AR1010_REG_STATUS, 1);

		// for debug
		// printf("AR1010 STATUS REG: 0x%04D\n\r");
		
		// Get STC and Masking
		stc = GetAr1010Reg(AR1010_REG_STATUS);
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

/**
 * @brief AR1010의 RSSI와 STATUS 레지스터를 읽어 전역 변수 ar1010에 저장 (데이터시트 기준 동작)
 * 
 * @return int 0(성공) / 음수(실패)
 */
int Ar1010Update()
{
	int ret = AR1010_OK;
	uint16_t rx = 0;

	// Get RSSI Bits for signal strength
	ret = Ar1010Read(AR1010_REG_SSI, 2);
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

/**
 * @brief 주어진 주파수로 AR1010의 수신 주파수를 설정: TUNE (데이터시트의 Pseudo code 기준에서 약간 변경)
 * 
 * @param freq AR1010의 수신 주파수를 설정할 목표 주파수
 * @return int 0(성공) / 음수(실패)
 */
int Ar1010Tune(/*uint16_t band, uint16_t space, */double freq)
{
	int ret = AR1010_OK;
	double maxFreq = US_EU_MAX_FREQ;
	double minFreq = US_EU_MIN_FREQ;

	// 현재 band의 주파수 범위에 포함되는 주파수를 설정했는지 확인
	// uint16_t rx = ar1010.mem.r3 & AR1010_R3_BAND_MASK;
	uint16_t rx = GetAr1010Reg(AR1010_REG3);
	rx &= AR1010_R3_BAND_MASK;
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
	// rx = ar1010.mem.r1 | AR1010_R1_HMUTE_MASK;
	rx = GetAr1010Reg(AR1010_REG1);
	rx |= AR1010_R1_HMUTE_MASK;
	ret = Ar1010Write(AR1010_REG1, &rx, 2);
	if(ret < 0)
	{
		printf("AR1010 TUNE fail!\r\n");
		return ret;
	}

	// ar1010.mem.r1 = rx;

	// Clear TUNE Bit
	// rx = ar1010.mem.r2 & (~AR1010_R2_TUNE_MASK);
	rx = GetAr1010Reg(AR1010_REG2);
	rx &= ~AR1010_R2_TUNE_MASK;
	ret = Ar1010Write(AR1010_REG2, &rx, 2);
	if(ret < 0)
	{
		printf("AR1010 TUNE fail!\r\n");
		return ret;
	}

	// ar1010.mem.r2 = rx;

	// Clear SEEK Bit
	// rx = ar1010.mem.r3 & (~AR1010_R3_SEEK_MASK);
	rx = GetAr1010Reg(AR1010_REG3);
	rx &= ~AR1010_R3_SEEK_MASK;
	ret = Ar1010Write(AR1010_REG3, &rx, 2);
	if(ret < 0)
	{
		printf("AR1010 TUNE fail!\r\n");
		return ret;
	}

	// ar1010.mem.r3 = rx;

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
	// rx = ar1010.mem.r2 & (~AR1010_R2_CHAN_MASK);
	rx = GetAr1010Reg(AR1010_REG2);
	rx &= ~AR1010_R2_CHAN_MASK;
	rx |= chan;
	ret = Ar1010Write(AR1010_REG2, &rx, 2);
	if(ret < 0)
	{
		printf("AR1010 TUNE fail!\r\n");
		return ret;
	}

	// ar1010.mem.r2 = rx;

	// Enable TUNE Bit
	rx |= AR1010_R2_TUNE_MASK;
	ret = Ar1010Write(AR1010_REG2, &rx, 2);
	if(ret < 0)
	{
		printf("AR1010 TUNE fail!\r\n");
		return ret;
	}

	// ar1010.mem.r2 = rx;

	// Wait STC flag
	ret = Ar1010WaitStc(AR1010_STC_TIMEOUT_MS);
	if(ret < 0)
	{
		printf("AR1010 TUNE Timeout!\r\n");
		return ret;
	}

	// Clear HMUTE Bit
	// rx = ar1010.mem.r1 & (~AR1010_R1_HMUTE_MASK);
	rx = GetAr1010Reg(AR1010_REG1);
	rx &= ~AR1010_R1_HMUTE_MASK;
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

/**
 * @brief AR1010의 수신 주파수를 HIGH/LOW SIDE INJECTION을 사용해 주어진 주파수로 설정 (데이터시트의 Pseudo code 기준에서 약간 변경)
 * 
 * @param freq 목표 주파수
 * @return int 0(성공) / 음수(실패)
 */
int Ar1010HiloTune(double freq)
{
	int ret = AR1010_OK;
	double maxFreq = US_EU_MAX_FREQ;
	double minFreq = US_EU_MIN_FREQ;
	uint16_t loRssi = 0;
	uint16_t hiRssi = 0;

	// 현재 band의 주파수 범위에 포함되는 주파수를 설정했는지 확인
	// uint16_t rx = ar1010.mem.r3 & AR1010_R3_BAND_MASK;
	uint16_t rx = GetAr1010Reg(AR1010_REG3);
	rx &= AR1010_R3_BAND_MASK;
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
	// rx = ar1010.mem.r1 | AR1010_R1_HMUTE_MASK;
	rx = GetAr1010Reg(AR1010_REG1);
	rx |= AR1010_R1_HMUTE_MASK;
	ret = Ar1010Write(AR1010_REG1, &rx, 1);
	if(ret < 0)
	{
		printf("AR1010 TUNE fail!\r\n");
		return ret;
	}

	// Clear TUNE Bit
	// rx = ar1010.mem.r2 & (~AR1010_R2_TUNE_MASK);
	rx = GetAr1010Reg(AR1010_REG2);
	rx &= ~AR1010_R2_TUNE_MASK;
	ret = Ar1010Write(AR1010_REG2, &rx, 1);
	if(ret < 0)
	{
		printf("AR1010 TUNE fail!\r\n");
		return ret;
	}

	// ar1010.mem.r2 = rx;

	// Clear SEEK Bit
	// rx = ar1010.mem.r3 & (~AR1010_R3_SEEK_MASK);
	rx = GetAr1010Reg(AR1010_REG3);
	rx &= ~AR1010_R3_SEEK_MASK;
	ret = Ar1010Write(AR1010_REG3, &rx, 1);
	if(ret < 0)
	{
		printf("AR1010 TUNE fail!\r\n");
		return ret;
	}

	// ar1010.mem.r3 = rx;

	// Set BAND/SPACE/CHAN Bits
	// rx = ar1010.mem.r2 & (~AR1010_R2_CHAN_MASK);
	rx = GetAr1010Reg(AR1010_REG2);
	rx &= ~AR1010_R2_CHAN_MASK;
	rx |= chan;
	ret = Ar1010Write(AR1010_REG2, &rx, 1);
	if(ret < 0)
	{
		printf("AR1010 TUNE fail!\r\n");
		return ret;
	}

	// ar1010.mem.r2 = rx;

	// Set R11 (clear hiloside, clear hiloctrl_b1/2)
	// rx = ar1010.mem.r11 & ~(AR1010_R11_HILO_SIDE_MASK | AR1010_R11_HILOCTRL_B1_MASK | AR1010_R11_HILOCTRL_B2_MASK);
	rx = GetAr1010Reg(AR1010_REG11);
	rx &= ~(AR1010_R11_HILO_SIDE_MASK | AR1010_R11_HILOCTRL_B1_MASK | AR1010_R11_HILOCTRL_B2_MASK);
	ret = Ar1010Write(AR1010_REG11, &rx, 1);
	if(ret < 0)
	{
		printf("AR1010 TUNE fail!\r\n");
		return ret;
	}

	// ar1010.mem.r11 = rx;

	// Enable TUNE Bit
	// rx = ar1010.mem.r2 | AR1010_R2_TUNE_MASK;
	rx = GetAr1010Reg(AR1010_REG2);
	rx |= AR1010_R2_TUNE_MASK;
	ret = Ar1010Write(AR1010_REG2, &rx, 1);
	if(ret < 0)
	{
		printf("AR1010 TUNE fail!\r\n");
		return ret;
	}

	// ar1010.mem.r2 = rx;

	// Wait STC flag
	ret = Ar1010WaitStc(AR1010_STC_TIMEOUT_MS);
	if(ret < 0)
	{
		printf("AR1010 TUNE Timeout!\r\n");
		return ret;
	}

	// Get RSSI (RSSI1)
	ret = Ar1010Read(AR1010_REG_SSI, 1);
	if(ret < 0)
	{
		printf("AR1010 TUNE fail!\r\n");
		return ret;
	}
	loRssi = GetAr1010Reg(AR1010_REG_SSI);
	loRssi &= AR1010_RSSI_RSSI_MASK;

	// Clear TUNE Bit
	// rx = ar1010.mem.r2 & (~AR1010_R2_TUNE_MASK);
	rx = GetAr1010Reg(AR1010_REG2);
	rx &= ~AR1010_R2_TUNE_MASK;
	ret = Ar1010Write(AR1010_REG2, &rx, 1);
	if(ret < 0)
	{
		printf("AR1010 TUNE fail!\r\n");
		return ret;
	}

	// ar1010.mem.r2 = rx;

	// Set R11 (set hiloside, set hiloctrl_b1/2)
	// rx = ar1010.mem.r11 | (AR1010_R11_HILO_SIDE_MASK | AR1010_R11_HILOCTRL_B1_MASK | AR1010_R11_HILOCTRL_B2_MASK);
	rx = GetAr1010Reg(AR1010_REG11);
	rx |= AR1010_R11_HILO_SIDE_MASK | AR1010_R11_HILOCTRL_B1_MASK | AR1010_R11_HILOCTRL_B2_MASK;
	ret = Ar1010Write(AR1010_REG2, &rx, 1);
	if(ret < 0)
	{
		printf("AR1010 TUNE fail!\r\n");
		return ret;
	}

	// ar1010.mem.r11 = rx;

	// Enable TUNE Bit
	// rx = ar1010.mem.r2 | AR1010_R2_TUNE_MASK;
	rx = GetAr1010Reg(AR1010_REG2);
	rx |= AR1010_R2_TUNE_MASK;
	ret = Ar1010Write(AR1010_REG2, &rx, 1);
	if(ret < 0)
	{
		printf("AR1010 TUNE fail!\r\n");
		return ret;
	}

	// ar1010.mem.r2 = rx;

	// Wait STC flag
	ret = Ar1010WaitStc(AR1010_STC_TIMEOUT_MS);
	if(ret < 0)
	{
		printf("AR1010 TUNE Timeout!\r\n");
		return ret;
	}

	// Get RSSI (RSSI2)
	ret = Ar1010Read(AR1010_REG_SSI, 1);
	if(ret < 0)
	{
		printf("AR1010 TUNE fail!\r\n");
		return ret;
	}
	hiRssi = GetAr1010Reg(AR1010_REG_SSI);
	hiRssi &= AR1010_RSSI_RSSI_MASK;

	// Clear TUNE Bit
	// rx = ar1010.mem.r2 & (~AR1010_R2_TUNE_MASK);
	rx = GetAr1010Reg(AR1010_REG2);
	rx &= ~AR1010_R2_TUNE_MASK;
	ret = Ar1010Write(AR1010_REG2, &rx, 2);
	if(ret < 0)
	{
		printf("AR1010 TUNE fail!\r\n");
		return ret;
	}

	// ar1010.mem.r2 = rx;

	// Compare Hi/Lo Side Signal Strength
	if(loRssi > hiRssi)
	{
		// Set R11 (clear hiloside, clear hiloctrl_b1/2)
		// rx = ar1010.mem.r11 & ~(AR1010_R11_HILO_SIDE_MASK | AR1010_R11_HILOCTRL_B1_MASK | AR1010_R11_HILOCTRL_B2_MASK);
		rx = GetAr1010Reg(AR1010_REG11);
		rx &= ~(AR1010_R11_HILO_SIDE_MASK | AR1010_R11_HILOCTRL_B1_MASK | AR1010_R11_HILOCTRL_B2_MASK);
		ret = Ar1010Write(AR1010_REG11, &rx, 2);
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
	// rx = ar1010.mem.r2 | AR1010_R2_TUNE_MASK;
	rx = GetAr1010Reg(AR1010_REG2);
	rx |= AR1010_R2_TUNE_MASK;
	ret = Ar1010Write(AR1010_REG2, &rx, 2);
	if(ret < 0)
	{
		printf("AR1010 TUNE fail!\r\n");
		return ret;
	}

	// ar1010.mem.r2 = rx;

	// Wait STC flag
	ret = Ar1010WaitStc(AR1010_STC_TIMEOUT_MS);
	if(ret < 0)
	{
		printf("AR1010 TUNE Timeout!\r\n");
		return ret;
	}

	// Clear HMUTE Bit
	// rx = ar1010.mem.r1 & (~AR1010_R1_HMUTE_MASK);
	rx = GetAr1010Reg(AR1010_REG1);
	rx &= ~AR1010_R1_HMUTE_MASK;
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

/**
 * @brief AR1010을 초기화
 * 
 * @param xo_en 초기화할 때 레지스터의 값을 결정하기 위한 플래그
 * @return int 0(성공) / 음수(실패)
 */
int Ar1010Init(uint8_t xo_en)
{
	uint32_t ret = AR1010_OK;
	uint16_t* defaultValue = NULL;

	sem_init(&ar1010_sem, 0, 1);
	StructAr1010Init(&ar1010);

	// for stable
	usleep(1000);

	if(xo_en)
	{
		defaultValue = ar1010DefaultRegValEx;
	}
	else
	{
		defaultValue = ar1010DefaultRegValIn;
	}

	// Set R1 to R17 Registers to default value
	ret = Ar1010Write(AR1010_REG1, defaultValue[AR1010_REG1], AR1010_WR_REG_SIZE - 1);
	if(ret < 0)
	{
		printf("AR1010 Initialize fail!\r\n");
		return ret;
	}

	// Set R0 Register to default value
	ret = Ar1010Write(AR1010_REG1, defaultValue[AR1010_REG0], 1);
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
	// ret = Ar1010Tune(87.5);
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

/**
 * @brief AR1010의 SEEK 동작을 수행 (데이터시트의 Pseudo code 기준에서 약간 변경)
 * 
 * @return int 0(성공) / 음수(실패)
 */
int Ar1010Seek(/*uint16_t band, uint16_t space, */)
{
	int ret = AR1010_OK;
	uint16_t chan = 0;

	// Set HMUTE Bit
	// uint16_t rx = ar1010.mem.r1 | AR1010_R1_HMUTE_MASK;
	uint16_t rx = GetAr1010Reg(AR1010_REG1);
	rx |= AR1010_R1_HMUTE_MASK;
	ret = Ar1010Write(AR1010_REG1, &rx, 2);
	if(ret < 0)
	{
		printf("AR1010 SEEK fail!\r\n");
		return ret;
	}

	// Clear TUNE Bit
	// rx = ar1010.mem.r2 & (~AR1010_R2_TUNE_MASK);
	rx = GetAr1010Reg(AR1010_REG2);
	rx &= ~AR1010_R2_TUNE_MASK;
	ret = Ar1010Write(AR1010_REG2, &rx, 2);
	if(ret < 0)
	{
		printf("AR1010 SEEK fail!\r\n");
		return ret;
	}

	// ar1010.mem.r2 = rx;

	//Set CHAN Bit

	// Clear SEEK Bit
	// rx = ar1010.mem.r3 & (~AR1010_R3_SEEK_MASK);
	rx = GetAr1010Reg(AR1010_REG3);
	rx &= ~AR1010_R3_SEEK_MASK;
	ret = Ar1010Write(AR1010_REG3, &rx, 2);
	if(ret < 0)
	{
		printf("AR1010 SEEK fail!\r\n");
		return ret;
	}

	// ar1010.mem.r3 = rx;

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

	// ar1010.mem.r2 = rx;

	// Wait STC flag
	ret = Ar1010WaitStc(AR1010_STC_TIMEOUT_MS);
	if(ret < 0)
	{
		printf("AR1010 SEEK Timeout!\r\n");
		return ret;
	}

	// Clear HMUTE Bit
	// rx = ar1010.mem.r1 & (~AR1010_R1_HMUTE_MASK);
	rx = GetAr1010Reg(AR1010_REG1);
	rx &= ~AR1010_R1_HMUTE_MASK;
	ret = Ar1010Write(AR1010_REG1, &rx, 2);
	if(ret < 0)
	{
		printf("AR1010 SEEK fail!\r\n");
		return ret;
	}

	// ar1010.mem.r1 = rx;
	
	// Update Functions (optional, but remember to update CHAN with the seek result in READCHAN before next seek)
	Ar1010Update();

	chan = (ar1010.mem.rstatus & AR1010_RSTATUS_READCHAN_MASK) >> AR1010_RSTATUS_READCHAN_SHIFT;

	if(chan != (ar1010.mem.r2 & AR1010_R2_CHAN_MASK))
	{
		rx = GetAr1010Reg(AR1010_REG2);
		rx &= AR1010_R2_CHAN_MASK;
		rx |= chan;
		SetAr1010Reg(AR1010_REG2, rx);
	}

	return ret;
}

/**
 * @brief AR1010의 SEEK 동작을 HIGH/LOW SIDE INJECTION을 사용하여 수행 (데이터시트의 Pseudo code 기준에서 약간 변경)
 * 
 * @return int 0(성공) / 음수(실패)
 */
int Ar1010HiloSeek(/*double freq*/)
{
	int ret = AR1010_OK;
	// uint16_t chan = (uint16_t)((freq - 69) * 10);
	uint16_t chan = 0;
	
	// Set HMUTE Bit
	uint16_t rx = GetAr1010Reg(AR1010_REG1);
	rx |= AR1010_R1_HMUTE_MASK;
	ret = Ar1010Write(AR1010_REG1, &rx, 1);
	if(ret < 0)
	{
		printf("Hilo SEEK fail!\r\n");
		return ret;
	}

	// Clear TUNE Bit
	rx = GetAr1010Reg(AR1010_REG2);
	rx &= ~AR1010_R2_TUNE_MASK;
	ret = Ar1010Write(AR1010_REG2, &rx, 1);
	if(ret < 0)
	{
		printf("Hilo SEEK fail!\r\n");
		return ret;
	}

	// Set CHAN Bits
	/*
	rx = GetAr1010Reg(AR1010_REG2);
	rx &= ~AR1010_R2_CHAN_MASK;
	rx |= chan;
	ret = Ar1010Write(AR1010_REG2, &rx, 1);
	if(ret < 0)
	{
		printf("Hilo SEEK fail!\r\n");
		return ret;
	}
	*/

	// Clear SEEK Bit
	rx = GetAr1010Reg(AR1010_REG3);
	rx &= ~AR1010_R3_SEEK_MASK;
	ret = Ar1010Write(AR1010_REG3, &rx, 1);
	if(ret < 0)
	{
		printf("Hilo SEEK fail!\r\n");
		return ret;
	}

	// Set SEEKUP/SPACE/BAND/SEEKTH Bits

	// Enable SEEK Bit
	// rx = GetAr1010Reg(AR1010_REG3);
	rx |= AR1010_R3_SEEK_MASK;
	ret = Ar1010Write(AR1010_REG3, &rx, 1);
	if(ret < 0)
	{
		printf("Hilo SEEK fail!\r\n");
		return ret;
	}

	// Wait STC flag
	ret = Ar1010WaitStc(AR1010_STC_TIMEOUT_MS);
	if(ret < 0)
	{
		printf("Hilo SEEK Timeout!\r\n");
		return ret;
	}

	// If SF is not set, TUNE with auto Hi/Lo (using the seek result in READCHAN as CHAN)
	// ret = Ar1010Read(AR1010_REG_STATUS, 1);
	Ar1010Update();

	rx = GetAr1010Reg(AR1010_REG_STATUS);
	rx &= AR1010_RSTATUS_SF_MASK;
	if(!rx)
	{
		// Ar1010Update();
		double freq = 0;

		rx = GetAr1010Reg(AR1010_REG_STATUS);
		chan = (rx & AR1010_RSTATUS_READCHAN_MASK) >> AR1010_RSTATUS_READCHAN_SHIFT;
		freq = 69 + 0.1 * chan;
		/*
		rx = GetAr1010Reg(AR1010_REG2);
		rx &= AR1010_R2_CHAN_MASK;
		
		if(chan != rx)
		{
			rx = GetAr1010Reg(AR1010_REG2);
			rx &= AR1010_R2_CHAN_MASK;
			rx |= chan;

			SetAr1010Reg(AR1010_REG2, rx);
		}
		*/

		ret = Ar1010HiloTune(freq);
		if(ret < 0)
		{
			printf("AR1010 Hilo SEEK fail while Hilo TUNE!\r\n");

			rx = GetAr1010Reg(AR1010_REG1);
			rx &= ~AR1010_R1_HMUTE_MASK;
			ret = Ar1010Write(AR1010_REG1, &rx, 1);
			if (ret < 0)
			{
				printf("AR1010 HMUTE fail in Ar1010HiloSeek function!\r\n");
			}

			ret = AR1010_EIO;
		}
		else
		{
			printf("AR1010 Hilo SEEK Success!\r\n");
		}
	}
	else
	{
		rx = GetAr1010Reg(AR1010_REG1);
		rx &= ~AR1010_R1_HMUTE_MASK;
		ret = Ar1010Write(AR1010_REG1, &rx, 1);
		if (ret < 0)
		{
			printf("AR1010 HMUTE fail in Ar1010HiloSeek function!\r\n");
		}

		ret = AR1010_EIO;
		printf("AR1010 HILO SEEK fail!\r\n");
	}

	// Clear HMUTE Bit
	if(ret >= 0)
	{
		rx = GetAr1010Reg(AR1010_REG1);
		rx &= ~AR1010_R1_HMUTE_MASK;
		ret = Ar1010Write(AR1010_REG1, &rx, 1);
		if (ret < 0)
		{
			printf("AR1010 HMUTE fail in Ar1010HiloSeek function!\r\n");
		}
	}

	// Update Functions (optional)

	return ret;
}
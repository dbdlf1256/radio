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
#define AR1010_REG0			0x00	// xo_en, ENABLE
#define AR1010_REG1			0x01	// stc_int_en, deemp, mono, smute, hmute
#define AR1010_REG2			0x02	// TUNE, CHAN
#define AR1010_REG3			0x03	// SEEKUP, SEEK, SPACE, BAND, VOLUME
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
#define AR1010_R0_XO_EN_SHIFT	7
#define AR1010_R0_ENABLE_MASK	0x0001
#define AR1010_R0_ENABLE_SHIFT	0

// AR1010 R_1 BIT
#define AR1010_R1_STC_INT_EN_MASK	0x0020
#define AR1010_R1_STC_INT_EN_SHIFT	5
#define AR1010_R1_DEEMP_MASK		0x0010
#define AR1010_R1_DEEMP_SHIFT		4
#define AR1010_R1_MONO_MASK			0x0008
#define AR1010_R1_MONO_SHIFT		3
#define AR1010_R1_SMUTE_MASK		0x0004
#define AR1010_R1_SMUTE_SHIFT		2
#define AR1010_R1_HMUTE_MASK		0x0002
#define AR1010_R1_HMUTE_SHIFT		1

// AR1010 R_2 BIT
#define AR1010_R2_TUNE_MASK		0x0200
#define AR1010_R2_TUNE_SHIFT	9
#define AR1010_R2_CHAN_MASK		0x01FF
#define AR1010_R2_CHAN_SHIFT	0

// AR1010 R_3 BIT
#define AR1010_R3_SEEKUP_MASK	0x8000
#define AR1010_R3_SEEKUP_SHIFT	15
#define AR1010_R3_SEEK_MASK		0x4000
#define AR1010_R3_SEEK_SHIFT	14
#define AR1010_R3_SPACE_MASK	0x2000
#define AR1010_R3_SPACE_SHIFT	13
#define AR1010_R3_BAND_MASK		0x1800
#define AR1010_R3_BAND_SHIFT	11
#define AR1010_R3_VOLUME_MASK	0x0780
#define AR1010_R3_VOLUME_SHIFT	7
#define AR1010_R3_SEEKTH_MASK	0x007F
#define AR1010_R3_SEEKTH_SHIFT	0

// AR1010 R_10 BIT
#define AR1010_R10_SEEK_WRAP_MASK	0x0008	
#define AR1010_R10_SEEK_WRAP_SHIFT	3	

// AR1010 R_11 BIT
#define AR1010_R11_HILO_SIDE_MASK		0x8000
#define AR1010_R11_HILO_SIDE_SHIFT		15
#define AR1010_R11_HILOCTRL_B1_MASK		0x0004
#define AR1010_R11_HILOCTRL_B1_SHIFT	3
#define AR1010_R11_HILOCTRL_B2_MASK		0x0001
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
#define AR1010_R14_VOLUME2_SHIFT	8


// AR1010 R_SSI BIT
#define AR1010_RSSI_RSSI_MASK		0xFE00
#define AR1010_RSSI_RSSI_SHIFT		9
#define AR1010_RSSI_IF_CNT_MASK		0x01FF
#define AR1010_RSSI_IF_CNT_SHIFT	0

// AR1010 R_STATUS BIT
#define AR1010_RSTATUS_READCHAN_MASK	0xFF80
#define AR1010_RSTATUS_READCHAN_SHIFT	7
#define AR1010_RSTATUS_STC_MASK			0x0020
#define AR1010_RSTATUS_STC_SHIFT		5
#define AR1010_RSTATUS_SF_MASK			0x0010
#define AR1010_RSTATUS_SF_SHIFT			4
#define AR1010_RSTATUS_ST_MASK			0x0008
#define AR1010_RSTATUS_ST_SHIFT			3

// AR1010 R_DEVICE BIT
#define AR1010_RDEVICE_VERSION_MASK		0xF000
#define AR1010_RDEVICE_VERSION_SHIFT	12
#define AR1010_RDEVICE_MFID_MASK		0x0FFF
#define AR1010_RDEVICE_MFID_SHIFT		0

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
	AR1010_EFAIL = -4
} ar1010_err_t;

// AR1010 Default Register Value
#define AR1010_WR_REG_SIZE	18
#define AR1010_RD_REG_SIZE	29

// AR1010 band별 기본 주파수 설정 값
#define AR1010_DEFAULT_FREQ_US_EU	87.5
#define AR1010_DEFAULT_FREQ_JP		76.0
#define AR1010_DEFAULT_FREQ_JP_EX	76.0

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

// Ar1010Wrtie 함수의 인자 valueLength는 16bit 기준이지만 내부에서는 8bit 기준으로 변경해야 하기 때문에 이를 계산하기 위한 매크로
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
int Ar1010Read(const uint8_t reg, uint32_t readLength)
{
	int ret = AR1010_OK;
	uint32_t readLength = readLength * 2;

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

// STC 플래그 확인 등에 사용할 timeout 값
#define AR1010_STC_TIMEOUT_MS	50

/**
 * 
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
	// uint16_t rx = 0;

	// Get RSSI Bits for signal strength
	ret = Ar1010Read(AR1010_REG_SSI, 2);
	if(ret < 0)
		printf("AR1010 RSSI/RSTATUS Update fail!\r\n");

	return ret;
}

/**
 * @brief AR1010의 모든 레지스터를 읽어 ar1010을 업데이트
 * 
 * @return int 0(성공) / 음수(실패)
 */
int Ar1010UpdateAll()
{
	int ret = AR1010_OK;

	ret = Ar1010Read(AR1010_REG0, AR1010_RD_REG_SIZE);
	if(ret < 0)
		printf("AR1010 Update All fail!\r\n");

	return ret;
}

// 유요한 BAND 비트의 값
#define BAND_US_EU	0x0000
#define BAND_JP		0x1000
#define BAND_JP_EX	0x1800

// BAND 별 최대/최소 주파수 값
#define US_EU_MIN_FREQ	87.5
#define US_EU_MAX_FREQ	108.0
#define JP_MIN_FREQ		76.0
#define JP_MAX_FREQ		90.0
#define JP_EX_MIN_FREQ	76.0
#define JP_EX_MAX_FREQ	108.0

/**
 * @brief AR1010의 internal crystal 사용 혹은 external reference clock 사용 여부 결정
 * 
 * @param enable 1(using internal crystal) / 0(using external reference clock)
 * @return int 0(성공) / 음수(실패)
 */
int Ar1010XoEnable(uint16_t enable)
{
	int ret = AR1010_OK;

	uint16_t r0 = GetAr1010Reg(AR1010_REG0);
	r0 &= ~AR1010_R0_XO_EN_MASK;
	r0 |= enable << AR1010_R0_XO_EN_SHIFT;

	ret = Ar1010Write(AR1010_REG0, &r0, 1);
	if(ret < 0)
		printf("Ar1010XoEnable function fail! value: %d\r\n", enable);

	return ret;
}

/**
 * @brief AR1010의 전력을 차단하여 Standby mode로 전환
 * 
 * @return int 0(성공) / 음수(실패)
 */
int Ar1010Off()
{
	int ret = AR1010_OK;

	uint16_t r0 = GetAr1010Reg(AR1010_REG0);
	r0 &= ~AR1010_R0_ENABLE_MASK;

	ret = Ar1010Write(AR1010_REG0, &r0, 1);
	if(ret < 0)
		printf("Ar1010Off function fail!\r\n");

	return ret;
}

/**
 * @brief AR1010의 STC 플래그 발생 시 인터럽트를 발생시킬지 여부에 대한 설정
 * 
 * @param enable 1(인터럽트 활성화) / 0(인터럽트 비활성화)
 * @return int 0(성공) / 음수(실패)
 */
int Ar1010StcInterruptEnable(uint16_t enable)
{
	int ret = AR1010_OK;

	uint16_t r1 = GetAr1010Reg(AR1010_REG1);
	r1 &= ~AR1010_R1_STC_INT_EN_MASK;
	r1 |= enable << AR1010_R1_STC_INT_EN_SHIFT;

	ret = Ar1010Write(AR1010_REG1, &r1, 1);
	if(ret < 0)
		printf("Ar1010StcInterruptEnable function fail! value: %d\r\n", enable);

	return ret;
}

/**
 * @brief AR1010의 De-emphasis 값을 설정
 * 
 * @param set 1(75us) / 0(50us)
 * @return int 0(성공) / 음수(실패)
 */
int Ar1010DeempSet(uint16_t set)
{
	int ret = AR1010_OK;

	uint16_t r1 = GetAr1010Reg(AR1010_REG1);
	r1 &= ~AR1010_R1_DEEMP_MASK;
	r1 |= set << AR1010_R1_DEEMP_SHIFT;

	ret = Ar1010Write(AR1010_REG1, &r1, 1);
	if(ret < 0)
		printf("Ar1010DeempSet function fail! value: %d\r\n", set);

	return ret;
}

/**
 * @brief AR1010의 출력을 MONO 또는 STEREO(수신 신호의 강도(RSSI값)에 따라 결정)로 설정
 * 
 * @param enable 1(MONO) / 0(STERERO or MONO)
 * @return int 0(성공) / 음수(실패)
 */
int Ar1010MonoEnable(uint16_t enable)
{
	int ret = AR1010_OK;

	uint16_t r1 = GetAr1010Reg(AR1010_REG1);
	r1 &= ~AR1010_R1_MONO_MASK;
	r1 |= enable << AR1010_R1_MONO_SHIFT;

	ret = Ar1010Write(AR1010_REG1, &r1, 1);
	if(ret < 0)
		printf("Ar1010MonoEnable function fail! value: %d\r\n", enable);

	return ret;
}

/**
 * @brief AR1010의 Soft Mute 활성화
 * 
 * @param enable 1(활성화) / 0(비활성화)
 * @return int 0(성공) / 음수(실패)
 */
int Ar1010SmuteEnable(uint16_t enable)
{
	int ret = AR1010_OK;

	uint16_t r1 = GetAr1010Reg(AR1010_REG1);
	r1 &= ~AR1010_R1_SMUTE_MASK;
	r1 |= enable << AR1010_R1_SMUTE_SHIFT;

	ret = Ar1010Write(AR1010_REG1, &r1, 1);
	if(ret < 0)
		printf("Ar1010SmuteEnable function fail! value: %d\r\n", enable);

	return ret;
}

/**
 * @brief AR1010의 Soft Mute 활성화
 * 
 * @param enable 1(활성화) / 0(비활성화)
 * @return int 0(성공) / 음수(실패)
 */
int Ar1010HmuteEnable(uint16_t enable)
{
	int ret = AR1010_OK;

	uint16_t r1 = GetAr1010Reg(AR1010_REG1);
	r1 &= ~AR1010_R1_HMUTE_MASK;
	r1 |= enable << AR1010_R1_HMUTE_SHIFT;

	ret = Ar1010Write(AR1010_REG1, &r1, 1);
	if(ret < 0)
		printf("Ar1010HmuteEnable function fail! value: %d\r\n", enable);

	return ret;
}

/**
 * @brief AR1010의 TUNE 트리거 비트 설정
 * 
 * @param enable 1(SET) / 0(RESET)
 * @return int 0(성공) / 음수(실패)
 */
int Ar1010TuneEnable(uint16_t enable)
{
	int ret = AR1010_OK;

	uint16_t r2 = GetAr1010Reg(AR1010_REG2);
	r2 &= ~AR1010_R2_TUNE_MASK;
	r2 |= enable << AR1010_R2_TUNE_SHIFT;

	ret = Ar1010Write(AR1010_REG2, &r2, 1);
	if(ret < 0)
		printf("Ar1010TuneEnable function fail! value: %d\r\n", enable);

	return ret;
}

// AR1010의 목표 주파수에 맞는 CHAN 비트의 값을 계산 혹은 그 반대 
#define AR1010_FREQ2CHAN(f)	(uint16_t)(((f) - 69) * 10)
#define AR1010_CHAN2FREQ(c)	(double)(69 + 0.1 * (c))

/**
 * @brief BAND 별로 목표 주파수가 유효 주파수 범위 내에 있는지 확인
 * 
 * @param freq 목표 주파수
 * @param band BAND 값
 * @return int 0(유효 범위 내) / 음수(유효 범위 외)
 */
int Ar1010CheckBandFreq(uint16_t freq, uint16_t band)
{
	int ret = AR1010_OK;
	uint16_t maxFreq = 0;
	uint16_t minFreq = 0;

	switch(band)
	{
		case BAND_US_EU:
			maxFreq = US_EU_MAX_FREQ;
			minFreq = US_EU_MIN_FREQ;
			break;
		case BAND_JP:
			maxFreq = JP_MAX_FREQ;
			minFreq = JP_MIN_FREQ;
			break;
		case BAND_JP_EX:
			maxFreq = JP_EX_MAX_FREQ;
			minFreq = JP_EX_MIN_FREQ;
			break;
		default:
			printf("AR1010 Unknown BAND value is set\r\n");
			ret = AR1010_EINVAL;
			return ret;
	}

	if(freq < minFreq || freq > maxFreq)
	{
		printf("Invalid Frequency!\r\n");
		ret = AR1010_EINVAL;
	}

	return ret;
}

/**
 * @brief AR1010의 CHAN 비트를 설정
 * 
 * @param chan 설정할 CHAN 비트 값
 * @return int 0(성공) / 음수(실패)
 */
int Ar1010ChannelSet(uint16_t chan)
{
	int ret = AR1010_OK;
	
	uint16_t r2 = GetAr1010Reg(AR1010_REG2);
	r2 &= ~AR1010_R2_CHAN_MASK;
	r2 |= chan;

	ret = Ar1010Write(AR1010_REG2, &r2, 1);
	if(ret < 0)
		printf("Ar1010ChannelSet function fail! value: 0x%03X\r\n", chan);

	return ret;
}

/**
 * @brief AR1010의 SEEK 동작을 수행할 때 기존 목표 주파수에서 증가할 것인지 감소할 것인지 결정
 * 
 * @param upDown 1(증가) / 0(감소)
 * @return int 0(성공) / 음수(실패)
 */
int Ar1010SeekDirection(uint16_t upDown)
{
	int ret = AR1010_OK;

	uint16_t r3 = GetAr1010Reg(AR1010_REG3);
	r3 &= ~AR1010_R3_SEEKUP_MASK;
	r3 |= upDown << AR1010_R3_SEEKUP_SHIFT;

	ret = Ar1010Write(AR1010_REG3, &r3, 1);
	if(ret < 0)
		printf("Ar1010SeekUpDown function fail! value: %d\r\n", upDown);

	return ret;
}

/**
 * @brief AR1010의 SEEK 비트를 설정
 * 
 * @param enable 1(SEEK-1) / 0(SEEK-0)
 * @return int 0(성공) / 음수(실패)
 */
int Ar1010SeekEnable(uint16_t enable)
{
	int ret = AR1010_OK;

	uint16_t r3 = GetAr1010Reg(AR1010_REG3);
	r3 &= ~AR1010_R3_SEEK_MASK;
	r3 |= enable << AR1010_R3_SEEK_SHIFT;

	ret = Ar1010Write(AR1010_REG3, &r3, 1);
	if(ret < 0)
		printf("Ar1010SeekEnable function fail! value: %d\r\n", enable);

	return ret;
}

/**
 * @brief AR1010의 SEEK 동작을 수행할 때 증감 값을 설정
 * 
 * @param set 1(100kHz) / 0(200kHz)
 * @return int 0(성공) / 음수(실패)
 */
int Ar1010SpaceSet(uint16_t set)
{
	int ret = AR1010_OK;

	uint16_t r3 = GetAr1010Reg(AR1010_REG3);
	r3 &= ~AR1010_R3_SEEKUP_MASK;
	r3 |= set << AR1010_R3_SEEKUP_SHIFT;

	ret = Ar1010Wrtie(AR1010_REG3, &r3, 1);
	if(ret < 0)
		printf("Ar1010SpaceSet function fail! value: %d\r\n");

	return ret;
}

/**
 * @brief AR1010의 BAND 비트를 설정
 * 
 * @param band 설정할 BAND 비트 값(BAND_US_EU/BAND_JP/BAND_JP_EX 중 선택)
 * @return int 0(성공) / 음수(실패)
 */
int Ar1010BandSelect(uint16_t band)
{
	int ret = AR1010_OK;

	uint16_t r3 = GetAr1010Reg(AR1010_REG3);
	r3 &= ~AR1010_R3_BAND_MASK;
	// r3 |= band << AR1010_R3_BAND_SHIFT;

	ret = Ar1010Write(AR1010_REG3, &r3, 1);
	if(ret < 0)
		printf("Ar1010BandSelect function fail! value: 0x%04X\r\n", band);

	return ret;
}

/**
 * @brief AR1010의 VOLUME 비트 설정
 * 
 * @param vol1 설정할 VOLUME 비트 값(개별로 사용 시 0-3 비트만 설정 (0x00-0x0F))
 * @return int 0(성공) / 음수(실패)
 */
int Ar1010Volume1Set(uint16_t vol1)
{
	int ret = AR1010_REG3;
	
	if(vol1 > 0x0F)
	{
		printf("vol1 is too big!\r\n");
		ret = AR1010_EINVAL;
		return ret;
	}

	uint16_t r3 = GetAr1010Reg(AR1010_REG3);
	r3 &= ~AR1010_R3_VOLUME_MASK;
	r3 |= vol1 << AR1010_R3_VOLUME_SHIFT;

	ret = Ar1010Write(AR1010_REG3, &r3, 1);
	if(ret < 0)
		printf("Ar1010VolumeSet function fail! value: 0x%03X\r\n", vol1);

	return ret;
}

/**
 * @brief AR1010의 SEEK 동작을 수행 여부를 결정하는 수신 강도의 임계값 설정(수신 강도가 여기에 설정하는 값 이상이어야 SEEK 동작을 수행함)
 * 
 * @param th SEEK 동작 수행 여부 결정 수신 강도 임계값
 * @return int 0(성공) / 음수(실패)
 */
int Ar1010SeekThSet(uint16_t th)
{
	int ret = AR1010_OK;

	uint16_t r3 = GetAr1010Reg(AR1010_REG3);
	r3 &= ~AR1010_R3_SEEKTH_MASK;
	r3 |= th;

	ret = Ar1010Write(AR1010_REG3, &r3, 1);
	if(ret < 0)
		printf("Ar1010SeekThSet function fail! value: 0x%02X\r\n", th);

	return ret;
}

/**
 * @brief AR1010이 SEEK 동작을 수행할 때 BAND 별 최대/최소값을 넘어가게된 경우 최소/최대값으로 돌아가 SEEK 동작을 수행할 것인지에 대한 여부 설정
 * 
 * @param enable 1(활성화) / 0(비활성화)
 * @return int 0(성공) / 음수(실패)
 */
int Ar1010SeekWrapEnable(uint16_t enable)
{
	int ret = AR1010_OK;

	uint16_t r10 = GetAr1010Reg(AR1010_REG10);
	r10 &= ~AR1010_R10_SEEK_WRAP_MASK;
	r10 |= enable << AR1010_R10_SEEK_WRAP_SHIFT;

	ret = Ar1010Write(AR1010_REG10, &r10, 1);
	if(ret < 0)
		printf("Ar1010SeekWrapEnable function fail! value: %d\r\n", enable);

	return ret;
}

/**
 * @brief AR1010의 High Side Injection 동작을 수행하기 위한 설정
 * 
 * @return int 0(성공) / 음수(실패)
 */
int Ar1010HighSideInjection()
{
	int ret = AR1010_OK;

	uint16_t r11 = GetAr1010Reg(AR1010_REG11);
	r11 |= AR1010_R11_HILO_SIDE_MASK | AR1010_R11_HILOCTRL_B1_MASK | AR1010_R11_HILOCTRL_B2_MASK;

	ret = Ar1010Write(AR1010_REG11, &r11, 1);
	if(ret < 0)
		printf("Ar1010HighSideInjection function fail!\r\n");

	return ret;
}

/**
 * @brief AR1010의 Low Side Injection 동작을 수행하기 위한 설정
 * 
 * @return int 0(성공) / 음수(실패)
 */
int Ar1010LowSideInjection()
{
	int ret = AR1010_OK;

	uint16_t r11 = GetAr1010Reg(AR1010_REG11);
	r11 &= ~(AR1010_R11_HILO_SIDE_MASK | AR1010_R11_HILOCTRL_B1_MASK | AR1010_R11_HILOCTRL_B2_MASK);

	ret = Ar1010Write(AR1010_REG11, &r11, 1);
	if(ret < 0)
		printf("Ar1010LowSideInjection function fail!\r\n");

	return ret;
}

// AR1010의 GPIO 번호
#define AR1010_GPIO3	2
#define AR1010_GPIO2	1
#define AR1010_GPIO1	0

// AR1010의 GPIO 제어 비트 마스크 계산 매크로
#define AR1010_GPIO_MASK(port)	(0x0003 << (2 * (port)))

// AR1010의 GPIO 기능 설정 값
#define AR1010_GPIO_DISABLE			0X0000
#define AR1010_GPIO_FUNC_1(port)	(0x0001 << (2 * (port)))
#define AR1010_GPIO_LOW(port)		(0x0002 << (2 * (port)))
#define AR1010_GPIO_HIGH(port)		(0x0003 << (2 * (port)))

/**
 * @brief AR1010의 GPIO 포트의 기능을 설정
 * 
 * @param port 기능을 설정할 GPIO 포트 번호(AR1010_GPIO1/AR1010_GPIO2/AR1010_GPIO3 중 선택)
 * @param func GPIO 기능: 0(Disable) / 1(port에 따른 특정 기능) / 2(Low Signal) / 3(High Signal)
 * @return int 0(성공) / 음수(실패)
 */
int Ar1010GpioSet(uint8_t port, uint32_t func)
{
	int ret = AR1010_OK;

	if(port > AR1010_GPIO3)
	{
		printf("Unknown GPIO Port!\r\n");
		ret = AR1010_EINVAL;
		return ret;
	}

	unsigned short r13 = GetAr1010Reg(AR1010_REG13);

	r13 &= ~(AR1010_GPIO_MASK(port));

	// GPIO 기능 선택
	switch(func)
	{
		case 0:
			r13 |= AR1010_GPIO_DISABLE;
			break;
		case 1:
			r13 |= AR1010_GPIO_FUNC_1(port);
			break;
		case 2:
			r13 |= AR1010_GPIO_LOW(port);
			break;
		case 3:
			r13 |= AR1010_GPIO_HIGH(port);
			break;
		default:
			printf("Unknown select value!\r\n");
			ret = AR1010_EINVAL;
			return ret;
	}

	ret = Ar1010Write(AR1010_REG13, &r13, 1);
	if(ret < 0)
	{
		printf("Select GPIO3 function fail!\r\n");
		return ret;
	}

	return ret;
}

/**
 * @brief AR1010의 VOLUME2 비트 값 설정
 * 
 * @param vol2 설정할 VOLUME2 비트 값 (개별로 사용 시 4-7 비트만 설정 (0x0F 초과, 0xF0 이하 (0x00은 사용 가능)))
 * @return int 0(성공) / 음수(실패)
 */
int Ar1010Volume2Set(uint16_t vol2)
{
	int ret = AR1010_OK;

	if((vol2 != 0) && (vol2 < 0x0F || vol2 > 0xF0))
	{
		printf("vol2 is invalid value!\r\n");
		ret = AR1010_EINVAL;
		return ret;
	}

	uint16_t r14 = GetAr1010Reg(AR1010_REG14);
	r14 &= ~AR1010_R14_VOLUME2_MASK;
	r14 |= vol2 << AR1010_R14_VOLUME2_SHIFT;

	ret = Ar1010Write(AR1010_REG14, &r14, 1);
	if(ret < 0)
		printf("Ar1010Volume2Set functio fail! value: 0x%04X\r\n");

	return ret;
}

/**
 * @brief 현재 AR1010의 volume 설정 step을 구함
 * 
 * @return int 0이상(volume step) / 음수(volume 설정이 잘 못 되어 있음)
 */
int GetAr1010VolumeStep()
{
	int ret = 0;

	uint16_t volume = 0;

	uint16_t rx = GetAr1010Reg(AR1010_REG3);
	rx &= AR1010_R3_VOLUME_MASK;
	rx >>= AR1010_R3_VOLUME_SHIFT;
	volume |= rx;

	rx = GetAr1010Reg(AR1010_REG14);
	rx &= AR1010_R14_VOLUME2_MASK;
	rx >>= AR1010_R14_VOLUME2_SHIFT;
	volume |= rx;

	switch(volume)
	{
		case AR1010_VOL_STEP0:
			ret = 0;
			break;
		case AR1010_VOL_STEP1:
			ret = 1;
			break;
		case AR1010_VOL_STEP2:
			ret = 2;
			break;
		case AR1010_VOL_STEP3:
			ret = 3;
			break;
		case AR1010_VOL_STEP4:
			ret = 4;
			break;
		case AR1010_VOL_STEP5:
			ret = 5;
			break;
		case AR1010_VOL_STEP6:
			ret = 6;
			break;
		case AR1010_VOL_STEP7:
			ret = 7;
			break;
		case AR1010_VOL_STEP8:
			ret = 8;
			break;
		case AR1010_VOL_STEP9:
			ret = 9;
			break;
		case AR1010_VOL_STEP10:
			ret = 10;
			break;
		case AR1010_VOL_STEP11:
			ret = 11;
			break;
		case AR1010_VOL_STEP12:
			ret = 12;
			break;
		case AR1010_VOL_STEP13:
			ret = 13;
			break;
		case AR1010_VOL_STEP14:
			ret = 14;
			break;
		case AR1010_VOL_STEP15:
			ret = 15;
			break;
		case AR1010_VOL_STEP16:
			ret = 16;
			break;
		case AR1010_VOL_STEP17:
			ret = 17;
			break;
		case AR1010_VOL_STEP18:
			ret = 18;
			break;
		default:
			ret = AR1010_EINVAL;
			break;
	}

	return ret;
}

/**
 * @brief AR1010의 VOLUME, VOLUME2 비트를 설정(즉, AR1010의 볼륨 설정)
 * 
 * @param stepVal 설정 할 볼륨 값(하위 4Bit-VOLUME Bits/상위 4Bit-VOLUME2-Bits)
 * @return int 0(성공) / 음수(실패)
 */
int SetAr1010Volume(uint8_t stepVal)
{
	int ret = AR1010_OK;

	uint16_t vol1 = stepVal & 0x0F;
	uint16_t vol2 = stepVal & 0xF0;
	
	ret = Ar1010Volume1Set(vol1);
	if(ret < 0)
	{
		printf("SetAr1010Volume function step1 fail!\r\n");
		return ret;
	}

	ret = Ar1010Volume2Set(vol2);
	if(ret < 0)
	{
		printf("SetAr1010Volume function step1 fail!\r\n");
		return ret;
	}
	
	int volStep = GetAr1010VolumeStep();
	printf("Now AR1010 Volume step: %d", volStep);

	return ret;
}

/**
 * @brief AR1010 데이터시트에 나온 VOLUME STEP에 따라 AR1010의 볼륨 설정
 * 
 * @param step 설정할 볼륨의 STEP(0-18 총 19 단계)
 * @return int 0(성공) / 음수(실패)
 */
int SetAr1010VolumeStep(int step)
{
	int ret = AR1010_OK;
	uint16_t StepVal = 0;

	switch(step)
	{
		case 0:
			StepVal = AR1010_VOL_STEP0;
			break;
		case 1:
			StepVal = AR1010_VOL_STEP1;
			break;
		case 2:
			StepVal = AR1010_VOL_STEP2;
			break;
		case 3:
			StepVal = AR1010_VOL_STEP3;
			break;
		case 4:
			StepVal = AR1010_VOL_STEP4;
			break;
		case 5:
			StepVal = AR1010_VOL_STEP5;
			break;
		case 6:
			StepVal = AR1010_VOL_STEP6;
			break;
		case 7:
			StepVal = AR1010_VOL_STEP7;
			break;
		case 8:
			StepVal = AR1010_VOL_STEP8;
			break;
		case 9:
			StepVal = AR1010_VOL_STEP9;
			break;
		case 10:
			StepVal = AR1010_VOL_STEP10;
			break;
		case 11:
			StepVal = AR1010_VOL_STEP11;
			break;
		case 12:
			StepVal = AR1010_VOL_STEP12;
			break;
		case 13:
			StepVal = AR1010_VOL_STEP13;
			break;
		case 14:
			StepVal = AR1010_VOL_STEP14;
			break;
		case 15:
			StepVal = AR1010_VOL_STEP15;
			break;
		case 16:
			StepVal = AR1010_VOL_STEP16;
			break;
		case 17:
			StepVal = AR1010_VOL_STEP17;
			break;
		case 18:
			StepVal = AR1010_VOL_STEP18;
			break;
		default:
			printf("SetAR1010VoluemStep function Invalid Argument!\r\n");
			ret = AR1010_EINVAL;
			return ret;
	}

	ret = SetAr1010Volume(StepVal);
	if(ret < 0)
		printf("SetAr1010VolumeStep function fail!\r\n");

	return ret;
}

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
	uint16_t rx = GetAr1010Reg(AR1010_REG3);
	rx &= AR1010_R3_BAND_MASK;
	ret = Ar1010CheckBandFreq(freq, rx);
	if(ret < 0)
	{
		printf("Invalid Frequency in this band!\r\n");
		return ret;
	}

	// 주파수 설정을 위한 CHAN 값 계산
	uint16_t chan = AR1010_FREQ2CHAN(freq);

	// Set HMUTE Bit
	ret = Ar1010HmuteEnable(1);
	if(ret < 0)
	{
		printf("AR1010 TUNE fail!\r\n");
		return ret;
	}

	// Clear TUNE Bit
	ret = Ar1010TuneEnable(0);
	if(ret < 0)
	{
		printf("AR1010 TUNE fail!\r\n");
		return ret;
	}
	
	// Clear SEEK Bit
	ret = Ar1010SeekEnable(0);
	if(ret < 0)
	{
		printf("AR1010 TUNE fail!\r\n");
		return ret;
	}

	// Set BAND/SPACE/CHAN Bits
	ret = Ar1010ChannelSet(chan);
	if(ret < 0)
	{
		printf("AR1010 TUNE fail!\r\n");
		return ret;
	}

	// Enable TUNE Bit
	ret = Ar1010TuneEnable(1);
	if(ret < 0)
	{
		printf("AR1010 TUNE fail!\r\n");
		return ret;
	}

	// Wait STC flag
	ret = Ar1010WaitStc(AR1010_STC_TIMEOUT_MS);
	if(ret < 0)
	{
		printf("AR1010 TUNE Timeout!\r\n");
		return ret;
	}

	// Clear HMUTE Bit
	ret = Ar1010HmuteEnable(0);
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
	uint16_t rx = GetAr1010Reg(AR1010_REG3);
	rx &= AR1010_R3_BAND_MASK;
	ret = Ar1010CheckBandFreq(freq, rx);
	if(ret < 0)
	{
		printf("Invalid Frequency in this band!\r\n");
		return ret;
	}

	uint16_t chan = AR1010_FREQ2CHAN(freq);

	// Set HMUTE Bit
	ret = Ar1010HmuteEnable(1);
	if(ret < 0)
	{
		printf("AR1010 TUNE fail!\r\n");
		return ret;
	}

	// Clear TUNE Bit
	ret = Ar1010TuneEnable(0);
	if(ret < 0)
	{
		printf("AR1010 TUNE fail!\r\n");
		return ret;
	}

	// Clear SEEK Bit
	ret = Ar1010SeekEnable(0);
	if(ret < 0)
	{
		printf("AR1010 TUNE fail!\r\n");
		return ret;
	}

	// Set BAND/SPACE/CHAN Bits
	ret = Ar1010ChannelSet(chan);
	if(ret < 0)
	{
		printf("AR1010 TUNE fail!\r\n");
		return ret;
	}
	// Set R11 (clear hiloside, clear hiloctrl_b1/2)
	ret = Ar1010LowSideInjection();
	if(ret < 0)
	{
		printf("AR1010 TUNE fail!\r\n");
		return ret;
	}

	// Enable TUNE Bit
	ret = Ar1010TuneEnable(1);
	if(ret < 0)
	{
		printf("AR1010 TUNE fail!\r\n");
		return ret;
	}

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
	ret = Ar1010TuneEnable(0);
	if(ret < 0)
	{
		printf("AR1010 TUNE fail!\r\n");
		return ret;
	}

	// Set R11 (set hiloside, set hiloctrl_b1/2)
	ret = Ar1010HighSideInjection();
	if(ret < 0)
	{
		printf("AR1010 TUNE fail!\r\n");
		return ret;
	}

	// Enable TUNE Bit
	ret = Ar1010TuneEnable(1);
	if(ret < 0)
	{
		printf("AR1010 TUNE fail!\r\n");
		return ret;
	}

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
	ret = Ar1010TuneEnable(0);
	if(ret < 0)
	{
		printf("AR1010 TUNE fail!\r\n");
		return ret;
	}

	// Compare Hi/Lo Side Signal Strength
	if(loRssi > hiRssi)
	{
		ret = Ar1010LowSideInjection();
		if(ret < 0)
		{
			printf("AR1010 TUNE fail!\r\n");
			return ret;
		}
	}
	else
	{
		ret = Ar1010HighSideInjection();
		if(ret < 0)
		{
			printf("AR1010 TUNE fail!\r\n");
			return ret;
		}
	}

	// Enable TUNE Bit
	ret = Ar1010TuneEnable(1);
	if(ret < 0)
	{
		printf("AR1010 TUNE fail!\r\n");
		return ret;
	}

	// Wait STC flag
	ret = Ar1010WaitStc(AR1010_STC_TIMEOUT_MS);
	if(ret < 0)
	{
		printf("AR1010 TUNE Timeout!\r\n");
		return ret;
	}

	// Clear HMUTE Bit
	ret = Ar1010HmuteEnable(0);
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
 * @brief AR1010을 초기화(전원 첫 인가 시 사용)
 * 
 * @param xo_en 1(Internal Crystal) / 0(External Reference Clock) (초기화할 때 레지스터의 값을 결정하기 위한 플래그)
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
	ret = Ar1010HiloTune(AR1010_DEFAULT_FREQ_US_EU);
	// ret = Ar1010Tune(AR1010_DEFAULT_FREQ_US_EU);
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
 * @brief AR1010을 초기화(STANDBY 모드에서 NORMAL 모드로 전환 시 사용)
 * 
 * @param xo_en 1(Internal Crystal) / 0(External Reference Clock) (초기화할 때 레지스터의 값을 결정하기 위한 플래그)
 * @return int 0(성공) / 음수(실패)
 */
int Ar1010On(uint8_t xo_en)
{
	uint32_t ret = AR1010_OK;
	uint16_t* defaultValue = NULL;

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
	ret = Ar1010HiloTune(AR1010_DEFAULT_FREQ_US_EU);
	// ret = Ar1010Tune(AR1010_DEFAULT_FREQ_US_EU);
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
 * @brief AR1010을 초기화(STANDBY MODE -> NORMAL MODE)
 * 
 * @param xo_en 1(Internal Crystal) / 0(External Reference Clock) (초기화할 때 레지스터의 값을 결정하기 위한 플래그)
 * @return int 0(성공) / 음수(실패)
 */
int Ar1010Reset(uint8_t xo_en)
{
	int ret = AR1010_OK;

	// for debug
	printf("AR1010 Reset Start!\r\n");

	ret = Ar1010Off();
	if(ret < 0)
	{
		printf("AR1010 Off fail in AR1010Reset function!\r\n");
	}
	else
	{
		// for stable
		sleep(1);

		ret = Ar1010On(xo_en);
		if(ret < 0)
		{
			printf("AR1010 Init fail in AR1010Reset function!\r\n");
		}
	}

	// for debug
	printf("AR1010 Reset Complete!\r\n");

	return ret;
}


/**
 * @brief AR1010의 SEEK 동작을 수행 (데이터시트의 Pseudo code 기준에서 약간 변경)
 * 
 * @return int 0(성공) / 음수(실패)
 */
int Ar1010Seek()
{
	int ret = AR1010_OK;
	uint16_t chan = 0;
	uint16_t rx = 0;

	// Set HMUTE Bit
	ret = Ar1010HmuteEnable(1);
	if(ret < 0)
	{
		printf("AR1010 SEEK fail!\r\n");
		return ret;
	}

	// Clear TUNE Bit
	ret = Ar1010TuneEnable(0);
	if(ret < 0)
	{
		printf("AR1010 SEEK fail!\r\n");
		return ret;
	}

	//Set CHAN Bit

	// Clear SEEK Bit
	ret = Ar1010SeekEnable(0);
	if(ret < 0)
	{
		printf("AR1010 SEEK fail!\r\n");
		return ret;
	}

	// Set SEEKUP/SPACE/BAND/SEEKTH Bits

	// Enable SEEK Bit
	ret = Ar1010SeekEnable(1);
	if(ret < 0)
	{
		printf("AR1010 SEEK fail!\r\n");
		return ret;
	}

	// Wait STC flag
	ret = Ar1010WaitStc(AR1010_STC_TIMEOUT_MS);
	if(ret < 0)
	{
		printf("AR1010 SEEK Timeout!\r\n");
		return ret;
	}

	// Clear HMUTE Bit
	ret = Ar1010HmuteEnable(0);
	if(ret < 0)
	{
		printf("AR1010 SEEK fail!\r\n");
		return ret;
	}

	// Update Functions (optional, but remember to update CHAN with the seek result in READCHAN before next seek)
	ret = Ar1010UpdateAll();
	if(ret < 0)
	{
		printf("AR1010 Update fail! Can't check SF flag!\r\n");
		ret = AR1010_EFAIL;
	}
	else
	{
		rx = GetAr1010Reg(AR1010_REG_STATUS);
	
		rx &= AR1010_RSTATUS_SF_MASK;
	
		if(rx)
		{
			printf("AR1010 SEEK fail!\r\n");
			ret = AR1010_EFAIL;
		}
	}

	return ret;
}

/**
 * @brief AR1010의 SEEK 동작을 HIGH/LOW SIDE INJECTION을 사용하여 수행 (데이터시트의 Pseudo code 기준에서 약간 변경)
 * 
 * @return int 0(성공) / 음수(실패)
 */
int Ar1010HiloSeek()
{
	int ret = AR1010_OK;
	uint16_t chan = 0;
	uint16_t rx = 0;

	// Set HMUTE Bit
	ret = Ar1010HmuteEnable(1);
	if(ret < 0)
	{
		printf("Hilo SEEK fail!\r\n");
		return ret;
	}

	// Clear TUNE Bit
	ret = Ar1010TuneEnable(0);
	if(ret < 0)
	{
		printf("Hilo SEEK fail!\r\n");
		return ret;
	}

	// Set CHAN Bits

	// Clear SEEK Bit
	ret = Ar1010SeekEnable(0);
	if(ret < 0)
	{
		printf("Hilo SEEK fail!\r\n");
		return ret;
	}

	// Set SEEKUP/SPACE/BAND/SEEKTH Bits

	// Enable SEEK Bit
	ret = Ar1010SeekEnable(1);
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
	ret = Ar1010UpdateAll();
	if(ret < 0)
	{
		printf("");
		ret = AR1010_EFAIL;
	}
	else
	{
		rx = GetAr1010Reg(AR1010_REG_STATUS);

		chan = (rx & AR1010_RSTATUS_READCHAN_MASK) >> AR1010_RSTATUS_READCHAN_SHIFT;

		rx &= AR1010_RSTATUS_SF_MASK;
		if(rx)
		{
			printf("AR1010 HILO SEEK fail!\r\n");
			ret = AR1010_EFAIL;
		}
		else
		{
			ret = Ar1010HiloTune(AR1010_CHAN2FREQ(chan));
			if(ret < 0)
			{
				printf("AR1010 HILO SEEK fail!\r\n");
				ret = AR1010_EFAIL;
			}
		}
	}

	// Clear HMUTE Bit
	if(ret >= 0)
	{
		ret = Ar1010HmuteEnable(0);
		if (ret < 0)
		{
			printf("AR1010 HMUTE fail in Ar1010HiloSeek function! Need to reset AR1010\r\n");
			ret = AR1010_EFAIL;
		}
	}

	// Update Functions (optional)

	return ret;
}

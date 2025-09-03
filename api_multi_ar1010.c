#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h> // usleep
#include <stdlib.h>
#include <semaphore.h>
#include "api_multi_ar1010.h"

int iic_read_reg(unsigned char slaveAddress,unsigned char regAddr,unsigned char *readData,unsigned int readDataLength);
int iic_write(unsigned char slaveAddress, unsigned char *writeData,unsigned int writeDataLength);
int I2CSwitch(unsigned char channel);

ar1010_dev_t ar1010Ch1;
ar1010_dev_t ar1010Ch2;
sem_t ar1010Ch1_sem;
sem_t ar1010Ch2_sem;

/*
// 레지스터 초기값
Reg0 = 0xFFFE  // R0: 1111 1111 1111 1110 xo_en: 1, ENABLE: 0
Reg1 = 0xC17F  // R1: 1100 0001 0111 1111 stc_int_en: 1, deemp: 1, mono: 1, smute: 1, hmute: 1
Reg2 = 0xCC00  // R2: 1100 1100 0000 0000 TUNE: 0, CHAN: 0 0000 0000
Reg3 = 0x8000  // R3: 1000 0000 0000 0000 SEEKUP: 1, SEEK: 0, SPACE: 0, BAND: 00, VOLUME: 0000, SEEKTH: 000 0000
Reg4 = 0xC400  // R4: 1100 0100 0000 0000 
Reg5 = 0x28AA  // R5: 0010 1000 1010 1010
Reg6 = 0x4000  // R6: 0100 0000 0000 0000
Reg7 = 0x1C00  // R7: 0001 1100 0000 0000
Reg8 = 0x0140  // R8: 0000 0001 0100 0000
Reg9 = 0x007D  // R9: 0000 0000 0111 1101
Reg10 = 0x8182 // R10: 1000 0001 1000 0010 seek_wrap: 0
Reg11 = 0x0048 // R11: 0000 0000 0100 1000 hilo_side: 0, hiloctrl_b1: 0, hiloctrl_b2: 0
Reg12 = 0xB500 // R12: 1011 0101 0000 0000
Reg13 = 0x8A40 // R13: 1000 1010 0100 0000 GPIO3: 00, GPIO2: 00, GPIO1: 00
Reg14 = 0x0418 // R14: 0000 0100 0010 1000 VOLUME2: 0000
Reg15 = 0x8088 // R15: 1000 0000 1000 1000
Reg16 = 0x0440 // R16: 0000 0100 0100 0000
Reg17 = 0xA003 // R17: 1010 0000 0000 0011
*/

// AR1010 Default Register Value
//const unsigned short ar1010DefaultRegValIn[AR1010_WR_REG_SIZE] = {
unsigned short ar1010DefaultRegValIn[AR1010_WR_REG_SIZE] = {
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
	0xB84A, // R13: 1011 1000 0100 0101 GPIO3: 00, GPIO2 01, GPIO1 01(기존) -> 1011 1000 0100 1010 GPIO2 10, GPIO1 10(변경)
	0xFC2D, // R14: 1111 1100 0010 1101 VOLUME2: 1111
	0x8097, // R15: 1000 0000 1001 0111 
	0x04A1, // R16
	0xDF6A  // R17
};
//const unsigned short ar1010DefaultRegValEx[AR1010_WR_REG_SIZE] = {
unsigned short ar1010DefaultRegValEx[AR1010_WR_REG_SIZE] = {
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
	0xB84A, // R13: 1011 1000 0100 0101 GPIO3 00, GPIO2 01, GPIO1 01(기존) -> 1011 1000 0100 1010 GPIO2 10, GPIO1 10(변경)
	0xFC2D, // R14: 1111 1100 0010 1101 VOLUME2: 1111
	0x8097, // R15: 1000 0000 1001 0111
	0x04A1, // R16
	0xDF6A  // R17
};

uint8_t ar1010VolStep[AR1010_VOL_STEP_SIZE] = {
	0x0F, 0xCF, 0xDF, 0xFF, 0xCB,
	0xDB, 0xFB, 0xFA, 0xF9, 0xF8,
	0xF7, 0xD6, 0xE6, 0xF6, 0xE3,
	0xF3, 0xF2, 0xF1, 0xF0
};

// ar과 세마포어 안 거는 함수들

/*
int InitAr1010Lock(ar1010_dev_t* ar)
{
	if(ar == NULL)
	{
		printf("No Information Of AR1010!\r\n");
		return AR1010_EINVAL;
	}

	sem_init(ar->lock, 0, 1);

	return AR1010_OK;
}

*/
int InitAr1010Reg(ar1010_dev_t* ar)
{
	/*
	if(ar == NULL)
	{
		printf("No Information Of AR1010!\r\n");
		return AR1010_EINVAL;
	}

	sem_wait(ar->lock);
	*/

	memset(ar->reg, 0, AR1010_RD_REG_SIZE * 2);
	// memset(ar->reg, 0, sizeof(ar->reg));

	// sem_post(ar->lock);

	return AR1010_OK;
}

int InitAr1010Dev(ar1010_dev_t* ar, sem_t* sem)
{
	/*
	if(ar == NULL)
	{
		printf("No Information Of AR1010!\r\n");
		return AR1010_EINVAL;
	}
	*/

	ar->lock = sem;
	sem_init(ar->lock, 0, 1);
	memset(ar->reg, 0, AR1010_RD_REG_SIZE * 2);
	ar->onoff = 1;

	return AR1010_OK;
}

uint16_t GetAr1010Reg(ar1010_dev_t* ar, uint8_t reg)
{
	/*
	if(ar == NULL)
	{
		printf("No Information Of AR1010!\r\n");
		return AR1010_EINVAL;
	}
	*/

	uint16_t regVal = 0;

	// sem_wait(ar->lock);

	regVal = ar->reg[reg];

	// sem_post(ar->lock);

	return regVal;
}

int GetAr1010Regs(ar1010_dev_t* ar, uint8_t reg, uint16_t* regBuff, int length)
{
	/*
	if(ar == NULL)
	{
		printf("No Information Of AR1010!\r\n");
		return AR1010_EINVAL;
	}
	*/

	if(regBuff == NULL || length == 0)
	{
		printf("Invalid Argument in GetAr1010Regs function!\r\n");
		return AR1010_EINVAL;
	}

	// sem_wait(ar->lock);

	int i;
	for(i = 0; i < length; i++)
		regBuff[i] = ar->reg[i + reg];

	// sem_post(ar->lock);

	return AR1010_OK;
}

int SetAr1010Reg(ar1010_dev_t* ar, uint8_t reg, uint16_t val)
{
	/*
	if(ar == NULL)
	{
		printf("No Information Of AR1010!\r\n");
		return AR1010_EINVAL;
	}
	*/

	// sem_wait(ar->lock);

	ar->reg[reg] = val;

	// sem_post(ar->lock);

	return AR1010_OK;
}

int SetAr1010Regs(ar1010_dev_t* ar, uint8_t reg, uint16_t* val, int length)
{
	/*
	if(ar == NULL)
	{
		printf("No Information Of AR1010!\r\n");
		return AR1010_EINVAL;
	}
	*/

	if(val == NULL || length == 0)
	{
		printf("Invalid Argument in SetAr1010Regs function!\r\n");
		return AR1010_EINVAL;
	}

	// sem_wait(ar->lock);
	int i;
	for(i = 0; i < length; i++)
		ar->reg[i + reg] = val[i];

	// sem_post(ar->lock);

	return AR1010_OK;
}

/*
int SwitchAr1010(ar1010_dev_t* ar)
{
	if(ar == NULL)
	{
		printf("No Information Of AR1010!\r\n");
		return AR1010_EINVAL;
	}

	return ar->select ? ar->select(ar->chan) : 1;
}
*/

inline void Pack16(uint8_t* buff, uint16_t value)
{
	if(buff == NULL)
	{
		printf("Pack16 error!\r\n");
		return;
	}

	buff[0] = (uint8_t)(value >> 8);
	buff[1] = (uint8_t)(value & 0xFF);
}

inline uint16_t Unpack16(uint8_t* buff)
{
	if(buff == NULL)
	{
		printf("Pack16 error!\r\n");
		return 0;
	}

	return (uint16_t)((buff[0] << 8) | buff[1]);
}

 /**
  * @brief 주어진 값을 주어진 길이 만큼 AR1010의 레지스터에 쓰고 ar->wcache의 값을 업데이트
  * 
  * @param ar 제어할 AR1010의 정보를 담고 있는 구조체 포인터
  * @param reg 저장을 시작할 AR1010의 레지스터 주소 값
  * @param value 레지스터에 쓸 값의 포인터
  * @param valueLength 레지스터에 쓸 값의 길이 (uint16_t 기준)
  * @return int 0(성공) / 음수(실패)
  */
int Ar1010Write(ar1010_dev_t* ar, const uint8_t reg, uint16_t* value, uint32_t valueLength)
{	
	/*
	if(ar == NULL)
	{
		printf("No Information Of AR1010!\r\n");
		return AR1010_EINVAL;
	}
	*/

	int ret = AR1010_OK;
	int valIndex = 0;
	uint32_t writeLength = AR1010_WR_LENGTH(valueLength);
	uint8_t* writeData = (uint8_t*)malloc(writeLength);
	if(writeData == NULL)
	{
		printf("malloc fail in Ar1010Write function!\r\n");
		ret = AR1010_EIO;
		return ret;
	}

	memset(writeData, 0, writeLength);

	writeData[0] = reg;
	int i;
	for(i = 1; i < writeLength; i += 2)
	{
		Pack16(&writeData[i], value[valIndex++]);
	}

	// I2C Transmit to AR1010
	ret = iic_write(AR1010_ADDR, writeData, writeLength);
	free(writeData);
	if(ret < 0)
	{
		printf("AR1010 I2C Write fail!\r\n");
		return ret;
	}
	
	/*
	// 여기서 SetAr1010Reg 함수를 사용하면 오버헤드가 심해지고 병목이 생길 수 있므
	// ar->wcache를 직접 제어하는 방향으로 바꿔야하나?
	for(int r = reg; r < reg + valueLength; r++)
		SetAr1010Reg(ar, r, value[valIndex++]);
	*/
	SetAr1010Regs(ar, reg, value, valueLength);

	return ret;
}

/**
 * @brief AR1010의 레지스터 값을 읽어 ar->rcache에 값을 저장
 * 
 * @param ar 제어할 AR1010의 정보를 담고 있는 구조체 포인터
 * @param reg 읽기 시작할 AR1010 레지스터의 주소 값
 * @param readLength 읽을 레지스터의 길이 (uint16_t 기준)
 * @return int 0(성공) / 음수(실패)
 */
int Ar1010Read(ar1010_dev_t* ar, const uint8_t reg, uint32_t readLength)
{
	/*
	if(ar == NULL)
	{
		printf("No Information Of AR1010!\r\n");
		return AR1010_EINVAL;
	}
	*/

	int ret = AR1010_EIO;
	int readIndex = 0;
	uint32_t readDataLength = readLength * 2;
	uint16_t* unpackData = (uint16_t*)malloc(readDataLength);
	uint8_t* readData = (uint8_t*)malloc(readDataLength);
	if(readData == NULL || unpackData == NULL)
	{
		printf("malloc fail in Ar1010Read function!\r\n");

		if(readData == NULL)
			free(readData);
		
		if(unpackData == NULL)
			free(unpackData);

		return ret;
	}

	memset(readData, 0, readDataLength);
	memset(unpackData, 0, readDataLength);

	ret = iic_read_reg(AR1010_ADDR, reg, readData, readDataLength);
	if(ret < 0)
	{
		printf("AR1010 I2C Read fail!\r\n");

		if(readData == NULL)
			free(readData);
		
		if(unpackData == NULL)
			free(unpackData);

		return ret;
	}
	
	/*
	// 여기서 SetAr1010Reg 함수를 사용하면 오버헤드가 심해지고 병목이 생길 수 있므
	// ar->rcache를 직접 제어하는 방향으로 바꿔야하나?
	for(int r = reg; r < reg + readLength; r++)
	{
		unpackData = Unpack16(&readData[readIndex]);
		SetAr1010Reg(ar, r, unpackData);
		readIndex += 2;
	}
	*/
	int i;
	for(i = 0; i < readLength; i++)
	{
		unpackData[i] = Unpack16(&readData[readIndex]);
		readIndex += 2;
	}
	SetAr1010Regs(ar, reg, unpackData, readLength);

	if(readData == NULL)
		free(readData);
	
	if(unpackData == NULL)
		free(unpackData);

	return ret;
}

/*
ret = SwitchAr1010(ar);
if(ret < 0)
{
	printf("AR1010 Switching fail!\r\n");
	free(writeData);
	return ret;
}
*/

/**
 * 
 * @brief AR1010의 STC 플래그 확인 함수
 * 
 * @param ar 제어할 AR1010의 정보를 담고 있는 구조체 포인터
 * @param timeout STC 플래그 확인 시간의 최대값
 * @return int 0(성공) / 음수(실패)
 */
int Ar1010WaitStc(ar1010_dev_t* ar, int timeout)
{
	/*
	if(ar == NULL)
	{
		printf("No Information Of AR1010!\r\n");
		return AR1010_EINVAL;
	}
	*/

	const uint32_t step = 1000;
	int ret = AR1010_ETIMEOUT;
	uint16_t stc = 0;

	while(timeout--)
	{
		// Read Status Register
		Ar1010Read(ar, AR1010_REG_STATUS, 1);

		// for debug
		// printf("AR1010 STATUS REG: 0x%04D\n\r");
		
		// Get STC and Masking
		stc = GetAr1010Reg(ar, AR1010_REG_STATUS);
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
 * @brief AR1010의 RSSI와 STATUS 레지스터를 읽어 ar->rcache에 저장 (데이터시트 기준 동작)
 * 
 * @param ar 제어할 AR1010의 정보를 담고 있는 구조체 포인터
 * @return int 0(성공) / 음수(실패)
 */
int Ar1010Update(ar1010_dev_t* ar)
{
	/*
	if(ar == NULL)
	{
		printf("No Information Of AR1010!\r\n");
		return AR1010_EINVAL;
	}
	*/

	int ret = AR1010_OK;
	// uint16_t rx = 0;

	// Get RSSI Bits for signal strength
	ret = Ar1010Read(ar, AR1010_REG_SSI, 2);
	if(ret < 0)
		printf("AR1010 RSSI/RSTATUS Update fail!\r\n");

	return ret;
}

/**
 * @brief AR1010의 모든 레지스터를 읽어 ar->rcache를 업데이트
 * 
 * @param ar 제어할 AR1010의 정보를 담고 있는 구조체 포인터
 * @return int 0(성공) / 음수(실패)
 */
int Ar1010UpdateAll(ar1010_dev_t* ar)
{
	/*
	if(ar == NULL)
	{
		printf("No Information Of AR1010!\r\n");
		return AR1010_EINVAL;
	}
	*/

	int ret = AR1010_OK;

	ret = Ar1010Read(ar, AR1010_REG0, AR1010_RD_REG_SIZE);
	if(ret < 0)
		printf("AR1010 Update All fail!\r\n");

	return ret;
}

/*
//고민 필요
#define AR1010_BIT_SET(var, m)	((r) |= (m))
#define AR1010_BIT_CLEAR(var, m)	((r) &= (~(m)))
#define AR1010_BIT_MASK(r, m)	((r) &= (m))
#define AR1010_FEILD_GET(r, m, s)	(AR1010_BIT_MASK((r), (m)) >>= s)
#define AR1010_FEILD_SET(r, v, m, s)	(AR1010_BIT_CLEAR((r), (m)) |= ((v) << (s)))
*/


/**
 * @brief AR1010의 internal crystal 사용 혹은 external reference clock 사용 여부 결정
 * 
 * @param ar 제어할 AR1010의 정보를 담고 있는 구조체 포인터
 * @param enable 1(using internal crystal) / 0(using external reference clock)
 * @return int 0(성공) / 음수(실패)
 */
int Ar1010XoEnable(ar1010_dev_t* ar, uint16_t enable)
{
	/*
	if(ar == NULL)
	{
		printf("No Information Of AR1010!\r\n");
		return AR1010_EINVAL;
	}
	*/

	int ret = AR1010_OK;

	uint16_t r0 = GetAr1010Reg(ar, AR1010_REG0);
	r0 &= ~AR1010_R0_XO_EN_MASK;
	r0 |= enable << AR1010_R0_XO_EN_SHIFT;

	ret = Ar1010Write(ar, AR1010_REG0, &r0, 1);
	if(ret < 0)
		printf("Ar1010XoEnable function fail! value: %d\r\n", enable);

	return ret;
}

/**
 * @brief AR1010의 전력을 차단하여 Standby mode로 전환
 * 
 * @param ar 제어할 AR1010의 정보를 담고 있는 구조체 포인터
 * @return int 0(성공) / 음수(실패)
 */
int Ar1010Off(ar1010_dev_t* ar)
{
	/*
	if(ar == NULL)
	{
		printf("No Information Of AR1010!\r\n");
		return AR1010_EINVAL;
	}
	*/

	int ret = AR1010_OK;

	uint16_t r0 = GetAr1010Reg(ar, AR1010_REG0);
	r0 &= ~AR1010_R0_ENABLE_MASK;

	ret = Ar1010Write(ar, AR1010_REG0, &r0, 1);
	if(ret < 0)
		printf("Ar1010Off function fail!\r\n");

	ar->onoff = 0;

	return ret;
}

/**
 * @brief AR1010의 전력을 연결하여 Normal mode로 전환
 * 
 * @param ar 제어할 AR1010의 정보를 담고 있는 구조체 포인터
 * @return int 0(성공) / 음수(실패)
 */
int Ar1010On(ar1010_dev_t* ar)
{
	/*
	if(ar == NULL)
	{
		printf("No Information Of AR1010!\r\n");
		return AR1010_EINVAL;
	}
	*/

	int ret = AR1010_OK;

	uint16_t r0 = GetAr1010Reg(ar, AR1010_REG0);
	r0 |= AR1010_R0_ENABLE_MASK;

	ret = Ar1010Write(ar, AR1010_REG0, &r0, 1);
	if(ret < 0)
		printf("Ar1010Off function fail!\r\n");

	// ar->onoff = 1;

	return ret;
}

/**
 * @brief AR1010의 STC 플래그 발생 시 인터럽트를 발생시킬지 여부에 대한 설정
 * 
 * @param ar 제어할 AR1010의 정보를 담고 있는 구조체 포인터
 * @param enable 1(인터럽트 활성화) / 0(인터럽트 비활성화)
 * @return int 0(성공) / 음수(실패)
 */
int Ar1010StcInterruptEnable(ar1010_dev_t* ar, uint16_t enable)
{
	/*
	if(ar == NULL)
	{
		printf("No Information Of AR1010!\r\n");
		return AR1010_EINVAL;
	}
	*/

	int ret = AR1010_OK;

	uint16_t r1 = GetAr1010Reg(ar, AR1010_REG1);
	r1 &= ~AR1010_R1_STC_INT_EN_MASK;
	r1 |= enable << AR1010_R1_STC_INT_EN_SHIFT;

	ret = Ar1010Write(ar, AR1010_REG1, &r1, 1);
	if(ret < 0)
		printf("Ar1010StcInterruptEnable function fail! value: %d\r\n", enable);

	return ret;
}

/**
 * @brief AR1010의 De-emphasis 값을 설정
 * 
 * @param ar 제어할 AR1010의 정보를 담고 있는 구조체 포인터
 * @param set 1(75us) / 0(50us)
 * @return int 0(성공) / 음수(실패)
 */
int Ar1010DeempSet(ar1010_dev_t* ar, uint16_t set)
{
	/*
	if(ar == NULL)
	{
		printf("No Information Of AR1010!\r\n");
		return AR1010_EINVAL;
	}
	*/

	int ret = AR1010_OK;

	uint16_t r1 = GetAr1010Reg(ar, AR1010_REG1);
	r1 &= ~AR1010_R1_DEEMP_MASK;
	r1 |= set << AR1010_R1_DEEMP_SHIFT;

	ret = Ar1010Write(ar, AR1010_REG1, &r1, 1);
	if(ret < 0)
		printf("Ar1010DeempSet function fail! value: %d\r\n", set);

	return ret;
}

/**
 * @brief AR1010의 출력을 MONO 또는 STEREO(수신 신호의 강도(RSSI값)에 따라 결정)로 설정
 * 
 * @param ar 제어할 AR1010의 정보를 담고 있는 구조체 포인터
 * @param enable 1(MONO) / 0(STERERO or MONO)
 * @return int 0(성공) / 음수(실패)
 */
int Ar1010MonoEnable(ar1010_dev_t* ar, uint16_t enable)
{
	/*
	if(ar == NULL)
	{
		printf("No Information Of AR1010!\r\n");
		return AR1010_EINVAL;
	}
	*/

	int ret = AR1010_OK;

	uint16_t r1 = GetAr1010Reg(ar, AR1010_REG1);
	r1 &= ~AR1010_R1_MONO_MASK;
	r1 |= enable << AR1010_R1_MONO_SHIFT;

	ret = Ar1010Write(ar, AR1010_REG1, &r1, 1);
	if(ret < 0)
		printf("Ar1010MonoEnable function fail! value: %d\r\n", enable);

	return ret;
}

/**
 * @brief AR1010의 Soft Mute 활성화
 * 
 * @param ar 제어할 AR1010의 정보를 담고 있는 구조체 포인터
 * @param enable 1(활성화) / 0(비활성화)
 * @return int 0(성공) / 음수(실패)
 */
int Ar1010SmuteEnable(ar1010_dev_t* ar, uint16_t enable)
{
	/*
	if(ar == NULL)
	{
		printf("No Information Of AR1010!\r\n");
		return AR1010_EINVAL;
	}
	*/

	int ret = AR1010_OK;

	uint16_t r1 = GetAr1010Reg(ar, AR1010_REG1);
	r1 &= ~AR1010_R1_SMUTE_MASK;
	r1 |= enable << AR1010_R1_SMUTE_SHIFT;

	ret = Ar1010Write(ar, AR1010_REG1, &r1, 1);
	if(ret < 0)
		printf("Ar1010SmuteEnable function fail! value: %d\r\n", enable);

	return ret;
}

/**
 * @brief AR1010의 Hard Mute 활성화
 * 
 * @param ar 제어할 AR1010의 정보를 담고 있는 구조체 포인터
 * @param enable 1(활성화) / 0(비활성화)
 * @return int 0(성공) / 음수(실패)
 */
int Ar1010HmuteEnable(ar1010_dev_t* ar, uint16_t enable)
{
	/*
	if(ar == NULL)
	{
		printf("No Information Of AR1010!\r\n");
		return AR1010_EINVAL;
	}
	*/

	int ret = AR1010_OK;

	uint16_t r1 = GetAr1010Reg(ar, AR1010_REG1);
	r1 &= ~AR1010_R1_HMUTE_MASK;
	r1 |= enable << AR1010_R1_HMUTE_SHIFT;

	ret = Ar1010Write(ar, AR1010_REG1, &r1, 1);
	if(ret < 0)
		printf("Ar1010HmuteEnable function fail! value: %d\r\n", enable);

	return ret;
}

/**
 * @brief AR1010의 TUNE 트리거 비트 설정
 * 
 * @param ar 제어할 AR1010의 정보를 담고 있는 구조체 포인터
 * @param enable 1(SET) / 0(RESET)
 * @return int 0(성공) / 음수(실패)
 */
int Ar1010TuneEnable(ar1010_dev_t* ar, uint16_t enable)
{
	/*
	if(ar == NULL)
	{
		printf("No Information Of AR1010!\r\n");
		return AR1010_EINVAL;
	}
	*/

	int ret = AR1010_OK;

	uint16_t r2 = GetAr1010Reg(ar, AR1010_REG2);
	r2 &= ~AR1010_R2_TUNE_MASK;
	r2 |= enable << AR1010_R2_TUNE_SHIFT;

	ret = Ar1010Write(ar, AR1010_REG2, &r2, 1);
	if(ret < 0)
		printf("Ar1010TuneEnable function fail! value: %d\r\n", enable);

	return ret;
}

/**
 * @brief BAND 별로 목표 주파수가 유효 주파수 범위 내에 있는지 확인
 * 
 * @param freq 목표 주파수
 * @param band BAND 값
 * @return int 0(유효 범위 내) / 음수(유효 범위 외)
 */
int Ar1010CheckBandFreq(double freq, uint16_t band)
{
	int ret = AR1010_EINVAL;
	double maxFreq = 0;
	double minFreq = 0;

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
			return ret;
	}

	if(freq >= minFreq && freq <= maxFreq)
	{
		printf("Valid Frequency! %.1f\r\n", freq);
		ret = AR1010_OK;
	}

	return ret;
}

/**
 * @brief AR1010의 CHAN 비트를 설정
 * 
 * @param ar 제어할 AR1010의 정보를 담고 있는 구조체 포인터
 * @param chan 설정할 CHAN 비트 값
 * @return int 0(성공) / 음수(실패)
 */
int Ar1010ChannelSet(ar1010_dev_t* ar, uint16_t chan)
{
	/*
	if(ar == NULL)
	{
		printf("No Information Of AR1010!\r\n");
		return AR1010_EINVAL;
	}
	*/

	printf("Ar1010ChannelSet Function receive a %04X CHAN value\r\n", chan);

	int ret = AR1010_OK;
	
	uint16_t r2 = GetAr1010Reg(ar, AR1010_REG2);
	r2 &= ~AR1010_R2_CHAN_MASK;
	r2 |= chan;

	ret = Ar1010Write(ar, AR1010_REG2, &r2, 1);
	if(ret < 0)
		printf("Ar1010ChannelSet function fail! value: 0x%03X\r\n", chan);

	return ret;
}

/**
 * @brief AR1010의 SEEK 동작을 수행할 때 기존 목표 주파수에서 증가할 것인지 감소할 것인지 결정
 * 
 * @param ar 제어할 AR1010의 정보를 담고 있는 구조체 포인터
 * @param upDown 1(증가) / 0(감소)
 * @return int 0(성공) / 음수(실패)
 */
int Ar1010SeekDirection(ar1010_dev_t* ar, uint16_t upDown)
{
	/*
	if(ar == NULL)
	{
		printf("No Information Of AR1010!\r\n");
		return AR1010_EINVAL;
	}
	*/

	int ret = AR1010_OK;

	uint16_t r3 = GetAr1010Reg(ar, AR1010_REG3);
	r3 &= ~AR1010_R3_SEEKUP_MASK;
	r3 |= upDown << AR1010_R3_SEEKUP_SHIFT;

	ret = Ar1010Write(ar, AR1010_REG3, &r3, 1);
	if(ret < 0)
		printf("Ar1010SeekUpDown function fail! value: %d\r\n", upDown);

	return ret;
}

/**
 * @brief AR1010의 SEEK 비트를 설정
 * 
 * @param ar 제어할 AR1010의 정보를 담고 있는 구조체 포인터
 * @param enable 1(SEEK-1) / 0(SEEK-0)
 * @return int 0(성공) / 음수(실패)
 */
int Ar1010SeekEnable(ar1010_dev_t* ar, uint16_t enable)
{
	/*
	if(ar == NULL)
	{
		printf("No Information Of AR1010!\r\n");
		return AR1010_EINVAL;
	}
	*/

	int ret = AR1010_OK;

	uint16_t r3 = GetAr1010Reg(ar, AR1010_REG3);
	r3 &= ~AR1010_R3_SEEK_MASK;
	r3 |= enable << AR1010_R3_SEEK_SHIFT;

	ret = Ar1010Write(ar, AR1010_REG3, &r3, 1);
	if(ret < 0)
		printf("Ar1010SeekEnable function fail! value: %d\r\n", enable);

	return ret;
}

/**
 * @brief AR1010의 SEEK 동작을 수행할 때 증감 값을 설정
 * 
 * @param ar 제어할 AR1010의 정보를 담고 있는 구조체 포인터
 * @param set 1(100kHz) / 0(200kHz)
 * @return int 0(성공) / 음수(실패)
 */
int Ar1010SpaceSet(ar1010_dev_t* ar, uint16_t set)
{
	/*
	if(ar == NULL)
	{
		printf("No Information Of AR1010!\r\n");
		return AR1010_EINVAL;
	}
	*/

	int ret = AR1010_OK;

	uint16_t r3 = GetAr1010Reg(ar, AR1010_REG3);
	r3 &= ~AR1010_R3_SPACE_MASK;
	r3 |= set << AR1010_R3_SPACE_SHIFT;

	ret = Ar1010Write(ar, AR1010_REG3, &r3, 1);
	if(ret < 0)
		printf("Ar1010SpaceSet function fail! value: %d\r\n", set);

	return ret;
}

/**
 * @brief AR1010의 BAND 비트를 설정
 * 
 * @param ar 제어할 AR1010의 정보를 담고 있는 구조체 포인터
 * @param band 설정할 BAND 비트 값(BAND_US_EU/BAND_JP/BAND_JP_EX 중 선택)
 * @return int 0(성공) / 음수(실패)
 */
int Ar1010BandSelect(ar1010_dev_t* ar, uint16_t band)
{
	/*
	if(ar == NULL)
	{
		printf("No Information Of AR1010!\r\n");
		return AR1010_EINVAL;
	}
	*/

	int ret = AR1010_OK;

	uint16_t r3 = GetAr1010Reg(ar, AR1010_REG3);
	r3 &= ~AR1010_R3_BAND_MASK;
	r3 |= band;

	ret = Ar1010Write(ar, AR1010_REG3, &r3, 1);
	if(ret < 0)
		printf("Ar1010BandSelect function fail! value: 0x%04X\r\n", band);

	return ret;
}

/**
 * @brief AR1010의 VOLUME 비트 설정
 * 
 * @param ar 제어할 AR1010의 정보를 담고 있는 구조체 포인터
 * @param vol1 설정할 VOLUME 비트 값(개별로 사용 시 0-3 비트만 설정 (0x00-0x0F))
 * @return int 0(성공) / 음수(실패)
 */
int Ar1010Volume1Set(ar1010_dev_t* ar, uint16_t vol1)
{
	/*
	if(ar == NULL)
	{
		printf("No Information Of AR1010!\r\n");
		return AR1010_EINVAL;
	}
	*/

	int ret = AR1010_EINVAL;
	
	if(vol1 > 0x0F)
	{
		printf("vol1 is too big!\r\n");
		return ret;
	}

	uint16_t r3 = GetAr1010Reg(ar, AR1010_REG3);
	r3 &= ~AR1010_R3_VOLUME_MASK;
	r3 |= vol1 << AR1010_R3_VOLUME_SHIFT;

	ret = Ar1010Write(ar, AR1010_REG3, &r3, 1);
	if(ret < 0)
		printf("Ar1010VolumeSet function fail! value: 0x%03X\r\n", vol1);

	return ret;
}

/**
 * @brief AR1010의 SEEK 동작을 수행 여부를 결정하는 수신 강도의 임계값 설정(수신 강도가 여기에 설정하는 값 이상이어야 SEEK 동작을 수행함)
 * 
 * @param ar 제어할 AR1010의 정보를 담고 있는 구조체 포인터
 * @param th SEEK 동작 수행 여부 결정 수신 강도 임계값
 * @return int 0(성공) / 음수(실패)
 */
int Ar1010SeekThSet(ar1010_dev_t* ar, uint16_t th)
{
	/*
	if(ar == NULL)
	{
		printf("No Information Of AR1010!\r\n");
		return AR1010_EINVAL;
	}
	*/

	int ret = AR1010_OK;

	uint16_t r3 = GetAr1010Reg(ar, AR1010_REG3);
	r3 &= ~AR1010_R3_SEEKTH_MASK;
	r3 |= th;

	ret = Ar1010Write(ar, AR1010_REG3, &r3, 1);
	if(ret < 0)
		printf("Ar1010SeekThSet function fail! value: 0x%02X\r\n", th);

	return ret;
}

/**
 * @brief AR1010이 SEEK 동작을 수행할 때 BAND 별 최대/최소값을 넘어가게된 경우 최소/최대값으로 돌아가 SEEK 동작을 수행할 것인지에 대한 여부 설정
 * 
 * @param ar 제어할 AR1010의 정보를 담고 있는 구조체 포인터
 * @param enable 1(활성화) / 0(비활성화)
 * @return int 0(성공) / 음수(실패)
 */
int Ar1010SeekWrapEnable(ar1010_dev_t* ar, uint16_t enable)
{
	/*
	if(ar == NULL)
	{
		printf("No Information Of AR1010!\r\n");
		return AR1010_EINVAL;
	}
	*/

	int ret = AR1010_OK;

	uint16_t r10 = GetAr1010Reg(ar, AR1010_REG10);
	r10 &= ~AR1010_R10_SEEK_WRAP_MASK;
	r10 |= enable << AR1010_R10_SEEK_WRAP_SHIFT;

	ret = Ar1010Write(ar, AR1010_REG10, &r10, 1);
	if(ret < 0)
		printf("Ar1010SeekWrapEnable function fail! value: %d\r\n", enable);

	return ret;
}

/**
 * @brief AR1010의 High Side Injection 동작을 수행하기 위한 설정
 * 
 * @param ar 제어할 AR1010의 정보를 담고 있는 구조체 포인터
 * @return int 0(성공) / 음수(실패)
 */
int Ar1010HighSideInjection(ar1010_dev_t* ar)
{
	/*
	if(ar == NULL)
	{
		printf("No Information Of AR1010!\r\n");
		return AR1010_EINVAL;
	}
	*/

	int ret = AR1010_OK;

	uint16_t r11 = GetAr1010Reg(ar, AR1010_REG11);
	r11 |= AR1010_R11_HILO_SIDE_MASK | AR1010_R11_HILOCTRL_B1_MASK | AR1010_R11_HILOCTRL_B2_MASK;

	ret = Ar1010Write(ar, AR1010_REG11, &r11, 1);
	if(ret < 0)
		printf("Ar1010HighSideInjection function fail!\r\n");

	return ret;
}

/**
 * @brief AR1010의 Low Side Injection 동작을 수행하기 위한 설정
 * 
 * @param ar 제어할 AR1010의 정보를 담고 있는 구조체 포인터
 * @return int 0(성공) / 음수(실패)
 */
int Ar1010LowSideInjection(ar1010_dev_t* ar)
{
	/*
	if(ar == NULL)
	{
		printf("No Information Of AR1010!\r\n");
		return AR1010_EINVAL;
	}
	*/

	int ret = AR1010_OK;

	uint16_t r11 = GetAr1010Reg(ar, AR1010_REG11);
	r11 &= ~(AR1010_R11_HILO_SIDE_MASK | AR1010_R11_HILOCTRL_B1_MASK | AR1010_R11_HILOCTRL_B2_MASK);

	ret = Ar1010Write(ar, AR1010_REG11, &r11, 1);
	if(ret < 0)
		printf("Ar1010LowSideInjection function fail!\r\n");

	return ret;
}

/**
 * @brief AR1010의 GPIO 포트의 기능을 설정
 * 
 * @param ar 제어할 AR1010의 정보를 담고 있는 구조체 포인터
 * @param port 기능을 설정할 GPIO 포트 번호(AR1010_GPIO1/AR1010_GPIO2/AR1010_GPIO3 중 선택)
 * @param func GPIO 기능: 0(Disable) / 1(port에 따른 특정 기능) / 2(Low Signal) / 3(High Signal)
 * @return int 0(성공) / 음수(실패)
 */
int Ar1010GpioSet(ar1010_dev_t* ar, uint8_t port, uint32_t func)
{
	/*
	if(ar == NULL)
	{
		printf("No Information Of AR1010!\r\n");
		return AR1010_EINVAL;
	}
	*/

	int ret = AR1010_EINVAL;

	if(port > AR1010_GPIO3)
	{
		printf("Unknown GPIO Port!\r\n");
		return ret;
	}

	unsigned short r13 = GetAr1010Reg(ar, AR1010_REG13);

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
			return ret;
	}

	ret = Ar1010Write(ar, AR1010_REG13, &r13, 1);
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
 * @param ar 제어할 AR1010의 정보를 담고 있는 구조체 포인터
 * @param vol2 설정할 VOLUME2 비트 값 (개별로 사용 시 4-7 비트만 설정 (0x0F 초과, 0xF0 이하 (0x00은 사용 가능)))
 * @return int 0(성공) / 음수(실패)
 */
int Ar1010Volume2Set(ar1010_dev_t* ar, uint16_t vol2)
{
	/*
	if(ar == NULL)
	{
		printf("No Information Of AR1010!\r\n");
		return AR1010_EINVAL;
	}
	*/

	int ret = AR1010_EINVAL;

	if((vol2 != 0) && (vol2 < 0x0F || vol2 > 0xF0))
	{
		printf("vol2 is invalid value!\r\n");
		return ret;
	}

	uint16_t r14 = GetAr1010Reg(ar, AR1010_REG14);
	r14 &= ~AR1010_R14_VOLUME2_MASK;
	r14 |= vol2 << AR1010_R14_VOLUME2_SHIFT;

	ret = Ar1010Write(ar, AR1010_REG14, &r14, 1);
	if(ret < 0)
		printf("Ar1010Volume2Set function fail! value: 0x%04X\r\n", r14);

	return ret;
}

/**
 * @brief 현재 AR1010의 volume 설정 step을 구함
 * 
 * @param ar 제어할 AR1010의 정보를 담고 있는 구조체 포인터
 * @return int 0이상(volume step) / 음수(volume 설정이 잘 못 되어 있음)
 */
int GetAr1010VolumeStep(ar1010_dev_t* ar)
{
	if(ar == NULL)
	{
		printf("No Information Of AR1010!\r\n");
		return AR1010_EINVAL;
	}

	int ret = 0;

	uint8_t volume = 0;

	uint16_t rx = GetAr1010Reg(ar, AR1010_REG3);
	rx &= AR1010_R3_VOLUME_MASK;
	rx >>= AR1010_R3_VOLUME_SHIFT;
	volume |= rx;

	rx = GetAr1010Reg(ar, AR1010_REG14);
	rx &= AR1010_R14_VOLUME2_MASK;
	rx >>= AR1010_R14_VOLUME2_SHIFT;
	volume |= rx;

	for(; ret < AR1010_VOL_STEP_SIZE; ret++)
		if(volume == ar1010VolStep[ret])
			break;

	if(ret >= AR1010_VOL_STEP_SIZE)
		ret = AR1010_EINVAL;

	return ret;
}

/**
 * @brief AR1010의 VOLUME, VOLUME2 비트를 설정(즉, AR1010의 볼륨 설정)
 * 
 * @param ar 제어할 AR1010의 정보를 담고 있는 구조체 포인터
 * @param stepVal 설정 할 볼륨 값(하위 4Bit-VOLUME Bits/상위 4Bit-VOLUME2-Bits)
 * @return int 0(성공) / 음수(실패)
 */
int SetAr1010Volume(ar1010_dev_t* ar, uint8_t stepVal)
{
	/*
	if(ar == NULL)
	{
		printf("No Information Of AR1010!\r\n");
		return AR1010_EINVAL;
	}
	*/

	int ret = AR1010_OK;

	uint16_t vol1 = stepVal & 0x0F;
	uint16_t vol2 = stepVal & 0xF0;
	
	ret = Ar1010Volume1Set(ar, vol1);
	if(ret < 0)
	{
		printf("SetAr1010Volume function step1 fail!\r\n");
		return ret;
	}

	ret = Ar1010Volume2Set(ar, vol2);
	if(ret < 0)
	{
		printf("SetAr1010Volume function step1 fail!\r\n");
		return ret;
	}
	
	// for debug
	int volStep = GetAr1010VolumeStep(ar);
	printf("Now AR1010 Volume step: %d\r\n", volStep);

	return ret;
}

/**
 * @brief AR1010 데이터시트에 나온 VOLUME STEP에 따라 AR1010의 볼륨 설정
 * 
 * @param ar 제어할 AR1010의 정보를 담고 있는 구조체 포인터
 * @param step 설정할 볼륨의 STEP(0-18 총 19 단계)
 * @return int 0(성공) / 음수(실패)
 */
int SetAr1010VolumeStep(ar1010_dev_t* ar, int step)
{
	if(ar == NULL)
	{
		printf("No Information Of AR1010!\r\n");
		return AR1010_EINVAL;
	}

	// sem_wait(ar->lock);

	int ret = AR1010_OK;

	if(step >= AR1010_VOL_STEP_SIZE)
	{
		printf("Invalid Argument in SetAr1010VolumeStep function!\r\n");
		return ret;
	}

	ret = SetAr1010Volume(ar, ar1010VolStep[step]);
	if(ret < 0)
		printf("SetAr1010VolumeStep function fail!\r\n");

	// sem_post(ar->lock);

	return ret;
}

/**
 * @brief 주어진 주파수로 AR1010의 수신 주파수를 설정: TUNE (데이터시트의 Pseudo code 기준에서 약간 변경)
 * 
 * @param ar 제어할 AR1010의 정보를 담고 있는 구조체 포인터
 * @param freq AR1010의 수신 주파수를 설정할 목표 주파수
 * @return int 0(성공) / 음수(실패)
 */
int Ar1010Tune(ar1010_dev_t* ar, /*uint16_t band, uint16_t space, */double freq)
{
	if(ar == NULL)
	{
		printf("No Information Of AR1010!\r\n");
		return AR1010_EINVAL;
	}

	if(ar->onoff == 0)
	{
		printf("AR1010 is Off Now!\r\n");
		return AR1010_EFAIL;
	}

	int ret = AR1010_OK;
	// double maxFreq = US_EU_MAX_FREQ;
	// double minFreq = US_EU_MIN_FREQ;

	// sem_wait(ar->lock);

	// 현재 band의 주파수 범위에 포함되는 주파수를 설정했는지 확인
	uint16_t rx = GetAr1010Reg(ar, AR1010_REG3);
	rx &= AR1010_R3_BAND_MASK;
	ret = Ar1010CheckBandFreq(freq, rx);
	if(ret < 0)
	{
		printf("Invalid Frequency in this band!\r\n");
		// sem_post(ar->lock);
		return ret;
	}

	// 주파수 설정을 위한 CHAN 값 계산
	uint16_t chan = AR1010_FREQ2CHAN(freq);

	// Set HMUTE Bit
	ret = Ar1010HmuteEnable(ar, 1);
	if(ret < 0)
	{
		printf("AR1010 TUNE fail!\r\n");
		// sem_post(ar->lock);
		return ret;
	}

	// Clear TUNE Bit
	ret = Ar1010TuneEnable(ar, 0);
	if(ret < 0)
	{
		printf("AR1010 TUNE fail!\r\n");
		// sem_post(ar->lock);
		return ret;
	}
	
	// Clear SEEK Bit
	ret = Ar1010SeekEnable(ar, 0);
	if(ret < 0)
	{
		printf("AR1010 TUNE fail!\r\n");
		// sem_post(ar->lock);
		return ret;
	}

	// Set BAND/SPACE/CHAN Bits
	ret = Ar1010ChannelSet(ar, chan);
	if(ret < 0)
	{
		printf("AR1010 TUNE fail!\r\n");
		// sem_post(ar->lock);
		return ret;
	}

	// Enable TUNE Bit
	ret = Ar1010TuneEnable(ar, 1);
	if(ret < 0)
	{
		printf("AR1010 TUNE fail!\r\n");
		// sem_post(ar->lock);
		return ret;
	}

	// Wait STC flag
	ret = Ar1010WaitStc(ar, AR1010_STC_TIMEOUT_MS);
	if(ret < 0)
	{
		printf("AR1010 TUNE Timeout!\r\n");
		// sem_post(ar->lock);
		return ret;
	}

	// Clear HMUTE Bit
	ret = Ar1010HmuteEnable(ar, 0);
	if(ret < 0)
	{
		printf("AR1010 TUNE fail!\r\n");
		// sem_post(ar->lock);
		return ret;
	}
	
	// Update Functions (optional)
	Ar1010Update(ar);

	// sem_post(ar->lock);

	return ret;
}

/**
 * @brief AR1010의 수신 주파수를 HIGH/LOW SIDE INJECTION을 사용해 주어진 주파수로 설정 (데이터시트의 Pseudo code 기준에서 약간 변경)
 * 
 * @param ar 제어할 AR1010의 정보를 담고 있는 구조체 포인터
 * @param freq 목표 주파수
 * @return int 0(성공) / 음수(실패)
 */
int Ar1010HiloTune(ar1010_dev_t* ar, double freq)
{
	if(ar == NULL)
	{
		printf("No Information Of AR1010!\r\n");
		return AR1010_EINVAL;
	}

	if(ar->onoff == 0)
	{
		printf("AR1010 is Off Now!\r\n");
		return AR1010_EFAIL;
	}

	// sem_wait(ar->lock);

	int ret = AR1010_OK;
	// double maxFreq = US_EU_MAX_FREQ;
	// double minFreq = US_EU_MIN_FREQ;
	uint16_t loRssi = 0;
	uint16_t hiRssi = 0;

	// 현재 band의 주파수 범위에 포함되는 주파수를 설정했는지 확인
	uint16_t rx = GetAr1010Reg(ar, AR1010_REG3);
	rx &= AR1010_R3_BAND_MASK;
	ret = Ar1010CheckBandFreq(freq, rx);
	if(ret < 0)
	{
		printf("Invalid Frequency in this band!\r\n");
		// sem_post(ar->lock);
		return ret;
	}

	uint16_t chan = AR1010_FREQ2CHAN(freq);
	printf("AR1010 Make Channel value: %04X\r\n", chan);

	// Set HMUTE Bit
	ret = Ar1010HmuteEnable(ar, 1);
	if(ret < 0)
	{
		printf("AR1010 TUNE fail!\r\n");
		// sem_post(ar->lock);
		return ret;
	}

	// Clear TUNE Bit
	ret = Ar1010TuneEnable(ar, 0);
	if(ret < 0)
	{
		printf("AR1010 TUNE fail!\r\n");
		// sem_post(ar->lock);
		return ret;
	}

	// Clear SEEK Bit
	ret = Ar1010SeekEnable(ar, 0);
	if(ret < 0)
	{
		printf("AR1010 TUNE fail!\r\n");
		// sem_post(ar->lock);
		return ret;
	}

	// Set BAND/SPACE/CHAN Bits
	ret = Ar1010ChannelSet(ar, chan);
	if(ret < 0)
	{
		printf("AR1010 TUNE fail!\r\n");
		// sem_post(ar->lock);
		return ret;
	}
	// Set R11 (clear hiloside, clear hiloctrl_b1/2)
	ret = Ar1010LowSideInjection(ar);
	if(ret < 0)
	{
		printf("AR1010 TUNE fail!\r\n");
		// sem_post(ar->lock);
		return ret;
	}

	// Enable TUNE Bit
	ret = Ar1010TuneEnable(ar, 1);
	if(ret < 0)
	{
		printf("AR1010 TUNE fail!\r\n");
		// sem_post(ar->lock);
		return ret;
	}

	// Wait STC flag
	ret = Ar1010WaitStc(ar, AR1010_STC_TIMEOUT_MS);
	if(ret < 0)
	{
		printf("AR1010 TUNE Timeout!\r\n");
		// sem_post(ar->lock);
		return ret;
	}

	// Get RSSI (RSSI1)
	ret = Ar1010Read(ar, AR1010_REG_SSI, 1);
	if(ret < 0)
	{
		printf("AR1010 TUNE fail!\r\n");
		// sem_post(ar->lock);
		return ret;
	}
	loRssi = GetAr1010Reg(ar, AR1010_REG_SSI);
	loRssi &= AR1010_RSSI_RSSI_MASK;

	// Clear TUNE Bit
	ret = Ar1010TuneEnable(ar, 0);
	if(ret < 0)
	{
		printf("AR1010 TUNE fail!\r\n");
		// sem_post(ar->lock);
		return ret;
	}

	// Set R11 (set hiloside, set hiloctrl_b1/2)
	ret = Ar1010HighSideInjection(ar);
	if(ret < 0)
	{
		printf("AR1010 TUNE fail!\r\n");
		// sem_post(ar->lock);
		return ret;
	}

	// Enable TUNE Bit
	ret = Ar1010TuneEnable(ar, 1);
	if(ret < 0)
	{
		printf("AR1010 TUNE fail!\r\n");
		// sem_post(ar->lock);
		return ret;
	}

	// Wait STC flag
	ret = Ar1010WaitStc(ar, AR1010_STC_TIMEOUT_MS);
	if(ret < 0)
	{
		printf("AR1010 TUNE Timeout!\r\n");
		// sem_post(ar->lock);
		return ret;
	}

	// Get RSSI (RSSI2)
	ret = Ar1010Read(ar, AR1010_REG_SSI, 1);
	if(ret < 0)
	{
		printf("AR1010 TUNE fail!\r\n");
		// sem_post(ar->lock);
		return ret;
	}
	hiRssi = GetAr1010Reg(ar, AR1010_REG_SSI);
	hiRssi &= AR1010_RSSI_RSSI_MASK;

	// Clear TUNE Bit
	ret = Ar1010TuneEnable(ar, 0);
	if(ret < 0)
	{
		printf("AR1010 TUNE fail!\r\n");
		// sem_post(ar->lock);
		return ret;
	}

	// Compare Hi/Lo Side Signal Strength
	if(loRssi > hiRssi)
		ret = Ar1010LowSideInjection(ar);
	else
		ret = Ar1010HighSideInjection(ar);

	if(ret < 0)
	{
		printf("AR1010 TUNE fail!\r\n");
		// sem_post(ar->lock);
		return ret;
	}


	// Enable TUNE Bit
	ret = Ar1010TuneEnable(ar, 1);
	if(ret < 0)
	{
		printf("AR1010 TUNE fail!\r\n");
		// sem_post(ar->lock);
		return ret;
	}

	// Wait STC flag
	ret = Ar1010WaitStc(ar, AR1010_STC_TIMEOUT_MS);
	if(ret < 0)
	{
		printf("AR1010 TUNE Timeout!\r\n");
		// sem_post(ar->lock);
		return ret;
	}

	// Clear HMUTE Bit
	ret = Ar1010HmuteEnable(ar, 0);
	if(ret < 0)
	{
		printf("AR1010 TUNE fail!\r\n");
		// sem_post(ar->lock);
		return ret;
	}

	// Update Functions (optional)
	Ar1010Update(ar);

	// sem_post(ar->lock);

	return ret;
}

/**
 * @brief AR1010의 SEEK 동작을 수행 (데이터시트의 Pseudo code 기준에서 약간 변경)
 * 
 * @param ar 제어할 AR1010의 정보를 담고 있는 구조체 포인터
 * @return int 0(성공) / 음수(실패)
 */
int Ar1010Seek(ar1010_dev_t* ar)
{
	if(ar == NULL)
	{
		printf("No Information Of AR1010!\r\n");
		return AR1010_EINVAL;
	}

	if(ar->onoff == 0)
	{
		printf("AR1010 is Off Now!\r\n");
		return AR1010_EFAIL;
	}

	// sem_wait(ar->lock);

	int ret = AR1010_OK;
	// uint16_t chan = 0;
	uint16_t rx = 0;

	// Set HMUTE Bit
	ret = Ar1010HmuteEnable(ar, 1);
	if(ret < 0)
	{
		printf("AR1010 SEEK fail!\r\n");
		// sem_post(ar->lock);
		return ret;
	}

	// Clear TUNE Bit
	ret = Ar1010TuneEnable(ar, 0);
	if(ret < 0)
	{
		printf("AR1010 SEEK fail!\r\n");
		// sem_post(ar->lock);
		return ret;
	}

	//Set CHAN Bit

	// Clear SEEK Bit
	ret = Ar1010SeekEnable(ar, 0);
	if(ret < 0)
	{
		printf("AR1010 SEEK fail!\r\n");
		// sem_post(ar->lock);
		return ret;
	}

	// Set SEEKUP/SPACE/BAND/SEEKTH Bits

	// Enable SEEK Bit
	ret = Ar1010SeekEnable(ar, 1);
	if(ret < 0)
	{
		printf("AR1010 SEEK fail!\r\n");
		// sem_post(ar->lock);
		return ret;
	}

	// Wait STC flag
	ret = Ar1010WaitStc(ar, AR1010_STC_TIMEOUT_MS);
	if(ret < 0)
	{
		printf("AR1010 SEEK Timeout!\r\n");
		// sem_post(ar->lock);
		return ret;
	}

	// Clear HMUTE Bit
	ret = Ar1010HmuteEnable(ar, 0);
	if(ret < 0)
	{
		printf("AR1010 SEEK fail!\r\n");
		// sem_post(ar->lock);
		return ret;
	}

	// Update Functions (optional, but remember to update CHAN with the seek result in READCHAN before next seek)
	ret = Ar1010UpdateAll(ar);
	if(ret < 0)
	{
		printf("AR1010 Update fail! Can't check SF flag!\r\n");
		// sem_post(ar->lock);
		ret = AR1010_EFAIL;
	}
	else
	{
		rx = GetAr1010Reg(ar, AR1010_REG_STATUS);
	
		rx &= AR1010_RSTATUS_SF_MASK;
	
		if(rx)
		{
			printf("AR1010 SEEK fail!\r\n");
			ret = AR1010_EFAIL;
		}
	}

	// sem_post(ar->lock);

	return ret;
}

/**
 * @brief AR1010의 SEEK 동작을 HIGH/LOW SIDE INJECTION을 사용하여 수행 (데이터시트의 Pseudo code 기준에서 약간 변경)
 * 
 * @param ar 제어할 AR1010의 정보를 담고 있는 구조체 포인터
 * @return int 0(성공) / 음수(실패)
 */
int Ar1010HiloSeek(ar1010_dev_t* ar)
{
	if(ar == NULL)
	{
		printf("No Information Of AR1010!\r\n");
		return AR1010_EINVAL;
	}

	if(ar->onoff == 0)
	{
		printf("AR1010 is Off Now!\r\n");
		return AR1010_EFAIL;
	}

	int ret = AR1010_OK;
	uint16_t chan = 0;
	uint16_t rx = 0;

	// sem_wait(ar->lock);

	// Set HMUTE Bit
	ret = Ar1010HmuteEnable(ar, 1);
	if(ret < 0)
	{
		printf("Hilo SEEK fail!\r\n");
		// sem_post(ar->lock);
		return ret;
	}

	// Clear TUNE Bit
	ret = Ar1010TuneEnable(ar, 0);
	if(ret < 0)
	{
		printf("Hilo SEEK fail!\r\n");
		// sem_post(ar->lock);
		return ret;
	}

	// Set CHAN Bits

	// Clear SEEK Bit
	ret = Ar1010SeekEnable(ar, 0);
	if(ret < 0)
	{
		printf("Hilo SEEK fail!\r\n");
		// sem_post(ar->lock);
		return ret;
	}

	// Set SEEKUP/SPACE/BAND/SEEKTH Bits

	// Enable SEEK Bit
	ret = Ar1010SeekEnable(ar, 1);
	if(ret < 0)
	{
		printf("Hilo SEEK fail!\r\n");
		// sem_post(ar->lock);
		return ret;
	}

	// Wait STC flag
	ret = Ar1010WaitStc(ar, AR1010_STC_TIMEOUT_MS);
	if(ret < 0)
	{
		printf("Hilo SEEK Timeout!\r\n");
		// sem_post(ar->lock);
		return ret;
	}

	// If SF is not set, TUNE with auto Hi/Lo (using the seek result in READCHAN as CHAN)
	ret = Ar1010UpdateAll(ar);
	if(ret < 0)
	{
		printf("Ar1010HiloSeek function fail\r\n");
		ret = AR1010_EFAIL;
	}
	else
	{
		rx = GetAr1010Reg(ar, AR1010_REG_STATUS);

		chan = (rx & AR1010_RSTATUS_READCHAN_MASK) >> AR1010_RSTATUS_READCHAN_SHIFT;

		rx &= AR1010_RSTATUS_SF_MASK;
		if(rx)
		{
			printf("AR1010 HILO SEEK fail!\r\n");
			ret = AR1010_EFAIL;
		}
		else
		{
			ret = Ar1010HiloTune(ar, AR1010_CHAN2FREQ(chan));
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
		ret = Ar1010HmuteEnable(ar, 0);
		if (ret < 0)
		{
			printf("AR1010 HMUTE fail in Ar1010HiloSeek function! Need to reset AR1010\r\n");
			ret = AR1010_EFAIL;
		}
	}

	// Update Functions (optional)
	
	// sem_post(ar->lock);
	
	return ret;
}



// 이 아랫부분은 AR1010의 초기화 관련 함수들인데 정리가 좀 필요하다.

/**
 * @brief AR1010을 초기화
 * 
 * @param ar 제어할 AR1010의 정보를 담고 있는 구조체 포인터
 * @param xo_en 1(Internal Crystal) / 0(External Reference Clock) (초기화할 때 레지스터의 값을 결정하기 위한 플래그)
 * @return int 0(성공) / 음수(실패)
 */
int Ar1010InitSequence(ar1010_dev_t* ar, uint16_t* custom)
{
	/*
	if(ar == NULL)
	{
		printf("No Information Of AR1010!\r\n");
		return AR1010_EINVAL;
	}
	*/

	uint32_t ret = AR1010_OK;
	// uint16_t* defaultValue = xo_en ? ar1010DefaultRegValIn : ar1010DefaultRegValEx;

	// StructAr1010Init(&ar1010);
	// InitAr1010Reg(ar);

	// Ar1010On(ar);

	// for stable
	usleep(1000);

	// sem_wait(ar->lock);

	// Set R1 to R17 Registers to default value
	printf("FMR - Ar1010InitSequence Function Start!\r\n");
	ret = Ar1010Write(ar, AR1010_REG1, &custom[AR1010_REG1], AR1010_WR_REG_SIZE - 1);
	if(ret < 0)
	{
		printf("AR1010 Initialize fail!\r\n");
		// sem_post(ar->lock);
		return ret;
	}

	// Set R0 Register to default value
	ret = Ar1010Write(ar, AR1010_REG0, custom, 1);
	if(ret < 0)
	{
		printf("AR1010 Initialize fail!\r\n");
		// sem_post(ar->lock);
		return ret;
	}

	// Wait STC flag
	ret = Ar1010WaitStc(ar, AR1010_STC_TIMEOUT_MS);
	if(ret < 0)
	{
		printf("AR1010 Initialize STC Timeout!\r\n");
		// sem_post(ar->lock);
		return ret;
	}

	ar->onoff = 1;

	// TUNING
	ret = Ar1010HiloTune(ar, AR1010_DEFAULT_FREQ_US_EU);
	// ret = Ar1010Tune(AR1010_DEFAULT_FREQ_US_EU);
	if(ret < 0)
	{
		printf("TUNE fail while AR1010 Initializing!\r\n");
		// sem_post(ar->lock);
		return ret;
	}

	// Wait STC flag
	/*
	ret = Ar1010WaitStc(ar, AR1010_STC_TIMEOUT_MS);
	if(ret < 0)
	{
		printf("AR1010 Initialize TUNE STC Timeout!\r\n");
		sem_post(ar->lock);
		return ret;
	}
	*/

	printf("FMR - Ar1010InitSequence Function End!\r\n");
	return ret;
}

/**
 * @brief AR1010을 초기화(전원 첫 인가 시 사용)
 * 
 * @param ar 제어할 AR1010의 정보를 담고 있는 구조체 포인터
 * @param sem 세마포어
 * @param f 2개 이상의 AR1010을 사용할 때 각 AR1010을 구분할 함수
 * @param chan f의 인자로 들어갈 AR1010 구분을 위한 채널 값
 * @param xo_en 1(Internal Crystal) / 0(External Reference Clock) (초기화할 때 레지스터의 값을 결정하기 위한 플래그)
 * @return int 0(성공) / 음수(실패)
 */
int Ar1010Init(ar1010_dev_t* ar, sem_t* sem, const uint8_t xo_en)
{
	if(ar == NULL)
	{
		printf("No Information Of AR1010!\r\n");
		return AR1010_EINVAL;
	}

	int ret = AR1010_OK;
	uint16_t* defaultValue = xo_en ? ar1010DefaultRegValIn : ar1010DefaultRegValEx;

	printf("FMR - AR1010 Init Start!\r\n");
	InitAr1010Dev(ar, sem);

	// sem_wait(ar->lock);

	ret = Ar1010InitSequence(ar, defaultValue);
	if(ret < 0)
	{
		printf("AR1010 Init fail!\r\n");
	}
	// sem_post(ar->lock);

	// ar->onoff = 1;

	printf("FMR - AR1010 Init End!\r\n");
	return ret;
}
/**
 * @brief 
 * 
 * @param ar 
 * @param xo_en 
 * @return int 
 */
int Ar1010Reset(ar1010_dev_t* ar, uint8_t xo_en)
{
	if(ar == NULL)
	{
		printf("No Information Of AR1010!\r\n");
		return AR1010_EINVAL;
	}

	// sem_wait(ar->lock);

	int ret = AR1010_OK;
	uint16_t* defaultValue = xo_en ? ar1010DefaultRegValIn : ar1010DefaultRegValEx;
	
	ret = Ar1010Off(ar);
	if(ret < 0)
	{
		printf("AR1010 Reset fail!\r\n");
		// sem_post(ar->lock);
		return ret;
	}
	
	ret = InitAr1010Reg(ar);
	if(ret < 0)
	{
		printf("AR1010 Reset fail!\r\n");
		// sem_post(ar->lock);
		return ret;
	}

	// 3초 대기
	usleep(3000);

	ret = Ar1010InitSequence(ar, defaultValue);
	if(ret < 0)
	{
		printf("AR1010 Reset fail!\r\n");
	}

	// ar->onoff = 1;

	// sem_post(ar->lock);

	return ret;
}

/**
 * @brief 
 * 
 * @param ar 
 * @return int 
 */
int Ar1010Wakeup(ar1010_dev_t* ar)
{
	int ret = AR1010_EINVAL;

	if(ar == NULL)
	{
		printf("No Information Of AR1010!\r\n");
		return ret;
	}

	// sem_wait(ar->lock);
	
	/*
	uint16_t writeData[AR1010_WR_REG_SIZE] = { 0, };

	int r;
	for(r = AR1010_REG0; r < AR1010_WR_REG_SIZE; r++)
	{
		writeData[r] = GetAr1010Reg(ar, r);
	}
	*/

	ret = Ar1010On(ar);
	if(ret < 0)
	{
		printf("AR1010 can't wakeup need to reset!\r\n");
		// sem_post(ar->lock);
		return ret;
	}

	ret = Ar1010WaitStc(ar, AR1010_STC_TIMEOUT_MS);
	if(ret < 0)
	{
		printf("AR1010 can't wakeup need to reset!\r\n");
	}

	ar->onoff = 1;

	/*
	ret = Ar1010InitSequence(ar, writeData);
	if(ret < 0)
	{
		printf("Ar1010\r\n");
	}
	*/

	// sem_post(ar->lock);

	return ret;
}

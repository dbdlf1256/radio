#ifndef __API_MULTI_AR1010_H__
#define __API_MULTI_AR1010_H__

#include <stdint.h>
#include <semaphore.h>

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
#define AR1010_VOL_STEP_SIZE	19

// AR1010 Register size
#define AR1010_WR_REG_SIZE	18
#define AR1010_RD_REG_SIZE	29

// AR1010 band별 기본 주파수 설정 값
#define AR1010_DEFAULT_FREQ_US_EU	87.5
#define AR1010_DEFAULT_FREQ_JP		76.0
#define AR1010_DEFAULT_FREQ_JP_EX	76.0

// Ar1010Wrtie 함수의 인자 valueLength는 16bit 기준이지만 내부에서는 8bit 기준으로 변경해야 하기 때문에 이를 계산하기 위한 매크로
#define AR1010_WR_LENGTH(L)	(((L) * 2) + 1)

// STC 플래그 확인 등에 사용할 timeout 값
#define AR1010_STC_TIMEOUT_MS	50

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

// AR1010의 목표 주파수에 맞는 CHAN 비트의 값을 계산 혹은 그 반대 
#define AR1010_FREQ2CHAN(f)	(uint16_t)(((f) - 69) * 10)
#define AR1010_CHAN2FREQ(c)	(double)(69 + 0.1 * (c))

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

// 조금 더 세분화할 필요가 있음
typedef enum
{
	AR1010_OK = 0,
	AR1010_EIO = -1,
	AR1010_EINVAL = -2,
	AR1010_ETIMEOUT = -3,
	AR1010_EFAIL = -4
} ar1010_err_t;

/*
typedef struct _ar1010Reg_t
{
	uint16_t r0;
	uint16_t r1;
	uint16_t r2;
	uint16_t r3;
	uint16_t r4;
	uint16_t r5;
	uint16_t r6;
	uint16_t r7;
	uint16_t r8;
	uint16_t r9;
	uint16_t r10;
	uint16_t r11;
	uint16_t r12;
	uint16_t r13;
	uint16_t r14;
	uint16_t r15;
	uint16_t r16;
	uint16_t r17;
	uint16_t rssi;
	uint16_t rstatus;
	uint16_t rbs;
	uint16_t rds1;
	uint16_t rds2;
	uint16_t rds3;
	uint16_t rds4;
	uint16_t rds5;
	uint16_t rds6;
	uint16_t rdevice;
	uint16_t rchipid;
} ar1010Reg_t;
*/

/*
typedef union
{
	uint16_t wcache[AR1010_WR_REG_SIZE];
	uint16_t rcache[AR1010_RD_REG_SIZE];
} ar1010_reg_t;
*/

// typedef int (*chanSel)(unsigned char);

typedef struct
{
	// ar1010_reg_t reg;
	uint16_t reg[AR1010_RD_REG_SIZE];
	sem_t* lock;
	// uint8_t chan;
	// chanSel select;
	uint8_t onoff; // 1: on / 0: off
} ar1010_dev_t;

// Volume Step Value
uint8_t ar1010VolStep[AR1010_VOL_STEP_SIZE] = {
	0x0F, 0xCF, 0xDF, 0xFF, 0xCB,
	0xDB, 0xFB, 0xFA, 0xF9, 0xF8,
	0xF7, 0xD6, 0xE6, 0xF6, 0xE3,
	0xF3, 0xF2, 0xF1, 0xF0
};

int InitAr1010Dev(ar1010_dev_t* ar, sem_t* sem);
int GetAr1010Reg(ar1010_dev_t* ar, uint8_t reg);
int GetAr1010Regs(ar1010_dev_t* ar, uint8_t reg, uint16_t* regBuff, int length);
int SetAr1010Reg(ar1010_dev_t* ar, uint8_t reg, uint16_t val);
int SetAr1010Regs(ar1010_dev_t* ar, uint8_t reg, uint16_t* val, int length);
int Ar1010Write(ar1010_dev_t* ar, const uint8_t reg, uint16_t* value, uint32_t valueLength);
int Ar1010Read(ar1010_dev_t* ar, const uint8_t reg, uint32_t readLength);
int Ar1010WaitStc(ar1010_dev_t* ar, int timeout);
int Ar1010Update(ar1010_dev_t* ar);
int Ar1010UpdateAll(ar1010_dev_t* ar);
int Ar1010XoEnable(ar1010_dev_t* ar, uint16_t enable);
int Ar1010Off(ar1010_dev_t* ar);
int Ar1010On(ar1010_dev_t* ar);
int Ar1010StcInterruptEnable(ar1010_dev_t* ar, uint16_t enable);
int Ar1010DeempSet(ar1010_dev_t* ar, uint16_t set);
int Ar1010MonoEnable(ar1010_dev_t* ar, uint16_t enable);
int Ar1010SmuteEnable(ar1010_dev_t* ar, uint16_t enable);
int Ar1010HmuteEnable(ar1010_dev_t* ar, uint16_t enable);
int Ar1010TuneEnable(ar1010_dev_t* ar, uint16_t enable);
int Ar1010CheckBandFreq(double freq, uint16_t band);
int Ar1010ChannelSet(ar1010_dev_t* ar, uint16_t chan);
int Ar1010SeekDirection(ar1010_dev_t* ar, uint16_t upDown);
int Ar1010SeekEnable(ar1010_dev_t* ar, uint16_t enable);
int Ar1010SpaceSet(ar1010_dev_t* ar, uint16_t set);
int Ar1010BandSelect(ar1010_dev_t* ar, uint16_t band);
int Ar1010Volume1Set(ar1010_dev_t* ar, uint16_t vol1);
int Ar1010SeekThSet(ar1010_dev_t* ar, uint16_t th);
int Ar1010SeekWrapEnable(ar1010_dev_t* ar, uint16_t enable);
int Ar1010HighSideInjection(ar1010_dev_t* ar);
int Ar1010LowSideInjection(ar1010_dev_t* ar);
int Ar1010GpioSet(ar1010_dev_t* ar, uint8_t port, uint32_t func);
int Ar1010Volume2Set(ar1010_dev_t* ar, uint16_t vol2);
int GetAr1010VolumeStep(ar1010_dev_t* ar);
int SetAr1010Volume(ar1010_dev_t* ar, uint8_t stepVal);
int SetAr1010VolumeStep(ar1010_dev_t* ar, int step);
int Ar1010Tune(ar1010_dev_t* ar, /*uint16_t band, uint16_t space, */double freq);
int Ar1010HiloTune(ar1010_dev_t* ar, double freq);
int Ar1010Seek(ar1010_dev_t* ar);
int Ar1010HiloSeek(ar1010_dev_t* ar);
int Ar1010InitSequence(ar1010_dev_t* ar, uint16_t* custum);
int Ar1010Init(ar1010_dev_t* ar, sem_t* sem, uint8_t xo_en);
int Ar1010Reset(ar1010_dev_t* ar, uint8_t xo_en);
int Ar1010Wakeup(ar1010_dev_t* ar);
inline void Pack16(uint8_t* buff, uint16_t value);
inline uint16_t Unpack16(uint8_t* buff);

#endif
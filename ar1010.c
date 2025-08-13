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
#define R14_VOLUME2	0x	// 4bit i don't know location

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
#define AR1010_VOL_STEP3	0xff
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
const unsigned short ar1010DefualtregValIn[AR1010_W_REG_SIZE] = {
	0xFFFB,
	0x5B15,
	0xD0B9,
	0xA010,
	0x0780,
	0x28AB,
	0x6400,
	0x1EE7,
	0x7141,
	0x007D,
	0x82C6,
	0x4E55,
	0x970C,
	0xB845,
	0xFC2D,
	0x8097,
	0x04A1,
	0xDF6A
};
const unsigned short ar1010DefualtregValEx[AR1010_W_REG_SIZE] = {
	0xFF7B,
	0x5B15,
	0xD0B9,
	0xA010,
	0x0780,
	0x28AB,
	0x6400,
	0x1EE7,
	0x7141,
	0x007D,
	0x82C6,
	0x4F55,
	0x970C,
	0xB845,
	0xFC2D,
	0x8097,
	0x04A1,
	0xDF6A
};



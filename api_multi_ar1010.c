#include <stdint.h>
#include <semaphore.h>

// 
#define AR1010_WR_REG_SIZE	18
#define AR1010_RD_REG_SIZE	28

typedef union
{
	uint16_t wcache[AR1010_WR_REG_SIZE];
	uint16_t rcache[AR1010_RD_REG_SIZE];
} ar1010_reg_t;

typedef void (*chanSel)(uint8_t);

typedef struct
{
	ar1010_reg_t reg;
	sem_t lock;
	uint8_t chan;
	chanSel select;
} ar1010_t;




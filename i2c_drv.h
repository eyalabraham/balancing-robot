/*****************************************************************************
* i2c_drv.h
*
* Driver hearder for AVT TWI interface
*
* This is driver is interrupt driven.
* All functionality is controlled through passing information to and
* from functions.
*
* Created: December 14, 2014
*
*****************************************************************************/

#ifndef __I2C_DRV_H__
#define __I2C_DRV_H__

/****************************************************************************
  Definitions
****************************************************************************/
#define TWI_BUFF_LEN	8 // data byte buffer

/****************************************************************************
  Function prototypes
****************************************************************************/
void i2c_s_initialize(uint8_t, uint8_t);
int i2c_s_getData(uint8_t *, uint8_t);
int i2c_s_setData(uint8_t *, uint8_t);

#endif /* __I2C_DRV_H__ */

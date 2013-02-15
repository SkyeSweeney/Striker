#ifndef __I2C_H__
#define __I2C_H__


INT8U i2c_read(INT8U add, RegisterID_e reg, REG_u *val);
INT8U i2c_write(INT8U add, RegisterID_e reg, REG_u val);


#endif


/**********************************************************************
 *
 * I2C routine to support a AS3935 Lighting detector chip
 *
 *********************************************************************/
#include <Wire.h>
#include "MyTypes.h"
#include "as3935.h"
#include "I2C.h"


#ifdef USE_I2C

#define SOFT_I2C
#ifdef SOFT_I2C

SoftI2cMaster si2c(SDA_PIN, SCL_PIN);  /* Bit-Bang I2C */


/**********************************************************************
 *
 * I2C read routine
 * Read a value from the specified register at the specified address
 * @param add The 7 bit address of the desired chip
 * @param reg The register number to read from
 * @param val Pointer to the storage for the read value
 * @return A coded error number
 * 0 - OK
 * 1 - Too long
 * 2 - NACK on address
 * 3 - NACK on data
 * 4 - Unknown error
 * 5 - Invalid number of bytes 
 *
 *********************************************************************/
INT8U i2c_read(INT8U add, RegisterID_e reg, REG_u *val) {
  INT8U  retval = 0;

  /* Set a default return value */
  val->data = 0x00;
  
  si2c.start( add | I2C_WRITE);
  si2c.write(reg);
  si2c.restart(add | I2C_READ);
  val->data = si2c.read(1);  /* Read just one byte */
  si2c.stop();
    
  return(retval);
  
}

/**********************************************************************
 *
 * I2C write routine
 * Write a value to the specified register at the specified address
 * @param add The 7 bit address of the desired chip
 * @param reg The register number to write to
 * @param val The value to put into the register
 * @return A coded error number
 *
 *********************************************************************/
INT8U i2c_write(INT8U add, RegisterID_e reg, REG_u val) {
  INT8U retval = 0;

  si2c.start( add | I2C_WRITE);
  si2c.write(reg);
  si2c.write(val.data);
  si2c.stop();
  
  return(retval);

}

#else


/**********************************************************************
 *
 * I2C read routine
 * Read a value from the specified register at the specified address
 * @param add The 7 bit address of the desired chip
 * @param reg The register number to read from
 * @param val Pointer to the storage for the read value
 * @return A coded error number
 * 0 - OK
 * 1 - Too long
 * 2 - NACK on address
 * 3 - NACK on data
 * 4 - Unknown error
 * 5 - Invalid number of bytes 
 *
 *********************************************************************/
INT8U i2c_read(INT8U add, RegisterID_e reg, REG_u *val) {
  INT8U  retval = 0;
  int    n;
  INT8U  err;
  int    i;

  /* Set a default return value */
  val->data = 0x00;
  
  /* Write word address to read */
  Wire.beginTransmission(add);
  Wire.write((INT8U)reg);
  retval = Wire.endTransmission(false);
  
  if (retval == 0) {
  
    /* Read the single byte from the specified word address */
    Wire.requestFrom((int)add, (int)1);
    
    /* Make sure we got the byte we needed */
    n = Wire.available();
    
    /* Check for the right number of bytes */
    if (n != 1) {
     
      retval = 5;
      
    } else {
    
      /* Get the byte */
      val->data = Wire.read();
    }
    
  } else {
    Serial.println("kdkd");
  }
  
  return(retval);
  
}

/**********************************************************************
 *
 * I2C write routine
 * Write a value to the specified register at the specified address
 * @param add The 7 bit address of the desired chip
 * @param reg The register number to write to
 * @param val The value to put into the register
 * @return A coded error number
 *
 *********************************************************************/
INT8U i2c_write(INT8U add, RegisterID_e reg, REG_u val) {
  INT8U retval;
  
  Wire.beginTransmission(add);
  Wire.write((INT8U)reg);
  Wire.write(val.data);
  retval = Wire.endTransmission();
  
  return(retval);

}
#endif
#endif

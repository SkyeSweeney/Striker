/**********************************************************************
 *
 * I2C routine to support a AS3935 Lighting detector chip
 *
 *********************************************************************/
#include <Wire.h>
#include "MyTypes.h"
#include "as3935.h"
#include "I2C.h"


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
  err = Wire.endTransmission(false);
  if (err != 0) return(err);
  
  /* Read the single byte from the specified word address */
  Wire.requestFrom((int)add, (int)1);
  
  /* Make sure we got the byte we needed */
  n = Wire.available();
  
  /* Check for the right number of bytes */
  if (n != 1) {
    return(5);
  }
  
  /* Get the byte */
  val->data = Wire.read();

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


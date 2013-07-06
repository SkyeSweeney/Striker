

#ifdef USE_I2C
  INT8U as3935_addr;
#endif



void io_init(void)
{
  
#ifdef USE_I2C
  as3935_addr = EAR1_AS3935_ADDR;
#endif

#ifdef USE_SPI
  SPI.begin();
  SPI.setDataMode(SPI_MODE1);
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV16);
  pinMode(SS_PIN, OUTPUT);
  digitalWrite(SS_PIN, HIGH);
#endif
}

/**********************************************************************
 *
 * Generic read routine
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
INT8U io_read(RegisterID_e reg, REG_u *val) 
{
#ifdef USE_I2C
  return(i2c_read(as3935_addr, reg, val));
#endif
#ifdef USE_SPI
  INT8U c;
  
  /* Enable chip select */
  digitalWrite(SS_PIN, LOW);
  
  /* Write register address */
  c = SPI.transfer(((INT8U)reg&0x3f) | 0x40);
  
  /* Read value */
  c = SPI.transfer(0x00);
  
  /* Disable chip select */
  digitalWrite(SS_PIN, HIGH);
  
  return(c);
#endif
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
INT8U io_write(RegisterID_e reg, REG_u val) 
{
#ifdef USE_I2C
  return(i2c_write(as3935_addr, reg, val));
#endif
#ifdef USE_SPI
  unsigned int err;
  
  /* Enable chip select */
  digitalWrite(SS_PIN, LOW);
  
  /* Write register address with read bit set */
  err = SPI.transfer(((INT8U)reg & 0x3f) | 0x00);
  
  /* write value */
  err = SPI.transfer(val.data);
  
  /* Disable chip select */
  digitalWrite(SS_PIN, HIGH);
  
#endif
}


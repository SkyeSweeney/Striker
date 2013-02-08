#include <Wire.h>
#include "types.h"
#include "as3935.h"


volatile INT32U counter = 0;
volatile INT8U  isrFlag = 0;

/**********************************************************************
 *
 * Setup routine
 *
 *********************************************************************/
void setup(void) {
  REG_u reg;
  int i;
  int n;
  byte err;
  
  Wire.begin();
  Serial.begin(115200);
  Serial.println("start");
  
  //attachInterrupt(0, as3935_isr, RISING);
  
  /* Set unit into operation mode */
  //as3935_set_powerdown(0);
  
  
#if 0
  for (i=0; i<0x2; i++) {
    reg.data = 0x24;
    i2c_write(AS3935_ADDR, (RegisterID_e)0, reg);
    reg.data = 0x24;
    i2c_write(AS3935_ADDR, (RegisterID_e)0, reg);
    reg.data = 0x41;
    i2c_write(AS3935_ADDR, (RegisterID_e)1, reg);
    reg.data = 0xC2;
    i2c_write(AS3935_ADDR, (RegisterID_e)2, reg);
    reg.data = 0xAA;
    i2c_write(AS3935_ADDR, (RegisterID_e)4, reg);
  }
#endif

  reg = i2c_read(AS3935_ADDR, (RegisterID_e)0x00);
  reg = i2c_read(AS3935_ADDR, (RegisterID_e)0x09);
    
  /* Calibrate the unit */
  
}



/**********************************************************************
 *
 * I2C read routine
 *
 *********************************************************************/
REG_u i2c_read(INT8U add, RegisterID_e reg) {
  REG_u retval;
  int   n;
  byte  err;
  int i;

  retval.data = 0xff;
  
  Serial.print("Reading from register: ");
  Serial.print(reg);
  Serial.print(" at address: ");
  Serial.print(add);
  
  // Write word address to read
  Wire.beginTransmission(add);
  Wire.write((INT8U)reg);
  err = Wire.endTransmission(false);
  switch(err) {
    case 0: Serial.print(" OK "); break;
    case 1: Serial.print(" Too Long "); break;
    case 2: Serial.print(" NACK on add "); break;
    case 3: Serial.print(" NACK on data "); break;
    default: Serial.print(" UNK error "); break;
  }
  
  // Read the single byte from the specified word address  
  Wire.requestFrom((int)add, (int)1);
  
  // Make sure we have the byte we needed
  n = Wire.available();
  Serial.print("  Got bytes: ");
  Serial.print(n);
  
  if (n != 1) {
    Serial.println("Invalid number of bytes");
    return(retval);
  }
  
  // Get the byte
  retval.data = Wire.read();

  Serial.print(", ");
  Serial.print(retval.data, HEX);
  
  Serial.println("");


  return(retval);
}

/**********************************************************************
 *
 * I2C write routine
 *
 *********************************************************************/
void i2c_write(INT8U add, RegisterID_e reg, REG_u val) {
  byte err;
  
  Wire.beginTransmission(add);
  Wire.write((INT8U)reg);
  Wire.write(val.data);
  err = Wire.endTransmission();
  switch(err) {
    case 0: Serial.println("OK"); break;
    case 1: Serial.println("Too Long"); break;
    case 2: Serial.println("NACK on add"); break;
    case 3: Serial.println("NACK on data"); break;
    default: Serial.println("UNK error"); break;
  }

}





/**********************************************************************
 *
 * Main loop
 *
 *********************************************************************/
void loop(void) {
  InterruptReason_e reason;
  
  /* If the ISR flag has been set */
  if (isrFlag) {
    
    isrFlag = 0;
    
    delay(2); // Delay claimed by datasheet
    reason = as3935_get_interrupt_reason();
    
    Serial.print("ISR: ");
    Serial.print(reason);
    Serial.println("");
    
    switch (reason){
      case INT_NOISY:
        Serial.println("Noise");
        break;
      case INT_DISTURBER:
        Serial.println("Disturbed");
        break;
      case INT_STRIKE:
        Serial.println("Strike");
        break;
      default:
        Serial.println("Unkown ISR");
        break;
    }

  }
  
}


/**********************************************************************
 *
 * ISR for the AS3935 interrupt
 *
 *********************************************************************/
void as3935_isr(void) {
  /* Due to the stupidity of this chip, the event can't be handled in
   * the ISR. This is because the 'reason' register is not available
   * till 2 ms AFTER the interrupt occurs! Totaly nuts.
   * So the only thing we can do is to set a flag and use that in the
   * main loop to poll against.
   * We will also increment a counter here for when we perform auto cal.
   */
   
   counter++;
   
   isrFlag = 1;

}



/**********************************************************************
 *
 * Set the powerdown flag
 *
 *********************************************************************/
void as3935_set_powerdown(INT8U pwr) {
  REG_u reg0;
  
  reg0 = i2c_read(AS3935_ADDR, REG00);
  reg0.R0.PWD = pwr;
  i2c_write(AS3935_ADDR, REG00, reg0);
}


/**********************************************************************
 *
 * Get the powerdown status
 *
 *********************************************************************/
INT8U as3935_get_powerdown(void) {
  REG_u reg0;
  reg0 = i2c_read(AS3935_ADDR, REG00);
  return reg0.R0.PWD;
}
  

/**********************************************************************
 *
 * Set the gain
 *
 *********************************************************************/
void as3935_set_afe(INT8U gain) {
  REG_u reg0;
  
  reg0 = i2c_read(AS3935_ADDR, REG00);
  reg0.R0.AFE_GB = gain;
  i2c_write(AS3935_ADDR, REG00, reg0);
}


/**********************************************************************
 *
 * Return the current gain
 *
 *********************************************************************/
INT8U as3935_get_afe(void) {
  REG_u reg0;

  reg0 = i2c_read(AS3935_ADDR, REG00);

  return reg0.R0.AFE_GB;
}


/**********************************************************************
 *
 * Set watchdog threshold
 *
 *********************************************************************/
void as3935_set_watchdog_threshold(INT8U threshold) {
  REG_u reg1;
  
  reg1 = i2c_read(AS3935_ADDR, REG01);
  reg1.R1.WDTH = threshold;
  i2c_write(AS3935_ADDR, REG01, reg1);
}


/**********************************************************************
 *
 * Get the watchdog threshold
 *
 *********************************************************************/
INT8U as3935_get_watchdog_threshold(void) {
  REG_u reg1;

  reg1 = i2c_read(AS3935_ADDR, REG01);

  return reg1.R1.WDTH;
}
  


/**********************************************************************
 *
 * Set the noise floor level
 *
 *********************************************************************/
void as3935_set_noise_floor_level(INT8U nfl) {
  REG_u reg1;
  
  reg1 = i2c_read(AS3935_ADDR, REG01);
  reg1.R1.NF_LEV = nfl;
  i2c_write(AS3935_ADDR, REG01, reg1);
}


/**********************************************************************
 *
 * Get the noise floor level
 *
 *********************************************************************/
INT8U as3935_get_noise_floor_level(void) {
  REG_u reg1;

  reg1 = i2c_read(AS3935_ADDR, REG01);

  return reg1.R1.NF_LEV;
}


/**********************************************************************
 *
 * Set the minimum number of strikes
 *
 *********************************************************************/
void as3935_set_spike_rejection(INT8U srej) {
  REG_u reg2;

  reg2 = i2c_read(AS3935_ADDR, REG02);
  reg2.R2.SREJ = srej;
  i2c_write(AS3935_ADDR, REG02, reg2);
  
}


/**********************************************************************
 *
 * Get the minimum number of strikes
 *
 *********************************************************************/
INT8U as3935_get_spike_rejection(void) {
  REG_u reg2;

  reg2 = i2c_read(AS3935_ADDR, REG02);

  return reg2.R2.SREJ;
}  


/**********************************************************************
 *
 * Calibrate the RCO
 *
 *********************************************************************/
void as3935_calibrate_rco(void) {
  REG_u reg8;
  INT8U  v = 0x61;
  
  
  // Send Direct command CALIB_RCO
  i2c_write(AS3935_ADDR, REG_CAL_RCO, *((REG_u*)&v));

  // Wait for it to stabilize
  delay(10);

  // Set the display TRCO bit
  reg8 = i2c_read(AS3935_ADDR, REG08);
  reg8.R8.DISP_TRCO = 1;
  i2c_write(AS3935_ADDR, REG08, reg8);
  
  // Wait 
  delay(10);
  
  // Clear the display TRCO bit
  reg8.R8.DISP_TRCO = 0;
  i2c_write(AS3935_ADDR, REG08, reg8);
}  


/**********************************************************************
 *
 * Get energy value from last strike
 *
 *********************************************************************/
INT32U as3935_get_energy_calc(void) {
  Energy_u e;
  REG_u  reg6;

  e.bits8[0] = 0;
  
  //  REG0x06[4:0] S_LIG_MM
  reg6 = i2c_read(AS3935_ADDR, REG06);
  e.bits8[1] = reg6.R6.S_LIG_MM;
  
  //   REG0x05[7:0] S_LIG_M
  e.bits8[2] = i2c_read(AS3935_ADDR, REG05).data;
  
  //  REG0x04[7:0]  S_LIG_L
  e.bits8[3] = i2c_read(AS3935_ADDR, REG04).data;
  
  return e.val;
  
}


/**********************************************************************
 *
 * Get storm distance
 *
 *********************************************************************/
INT8U as3935_get_storm_distance(void) {
  REG_u reg7;

  reg7 = i2c_read(AS3935_ADDR, REG07);

  return reg7.R7.DISTANCE;
}


/**********************************************************************
 *
 * Get reason for interrupt
 *
 *********************************************************************/
InterruptReason_e as3935_get_interrupt_reason(void) {
  REG_u reg3;

  // Datasheet indicates that the INT field only updates after 2ms!!!
  // What a piece of crap.
  delay(2); 
  reg3 = i2c_read(AS3935_ADDR, REG03);

  return((InterruptReason_e)reg3.R3.INT);
}  


/**********************************************************************
 *
 * Set the minimum number of strikes needed
 *
 *********************************************************************/
void as3935_set_minimum_lightning(MinStrikes_e min) {
  REG_u reg2;

  reg2 = i2c_read(AS3935_ADDR, REG02);
  reg2.R2.MIN_NUM_LIGHT = min;
  i2c_write(AS3935_ADDR, REG02, reg2);
}


/**********************************************************************
 *
 * Get the minimum number of strikes
 *
 *********************************************************************/
MinStrikes_e as3935_get_minimum_lightning(void) {
  REG_u reg2;
  
  reg2 = i2c_read(AS3935_ADDR, REG02);

  return (MinStrikes_e)reg2.R2.MIN_NUM_LIGHT;
}


/**********************************************************************
 *
 * Clear statistics
 *
 *********************************************************************/
void as3936_clear_statistics(void) {
  REG_u reg2;

  reg2 = i2c_read(AS3935_ADDR, REG02);
  reg2.R2.CL_STAT = 1;
  i2c_write(AS3935_ADDR, REG02, reg2);
  reg2.R2.CL_STAT = 0;
  i2c_write(AS3935_ADDR, REG02, reg2);
  reg2.R2.CL_STAT = 1;
  i2c_write(AS3935_ADDR, REG02, reg2);
}


/**********************************************************************
 *
 * Set the LCO divider
 *
 *********************************************************************/
void as3935_set_freq_div_ratio(LCO_DIV_e div_ratio) {
  REG_u reg3;

  reg3 = i2c_read(AS3935_ADDR, REG03);
  reg3.R3.LCO_FDIV = div_ratio;
  i2c_write(AS3935_ADDR, REG03, reg3);
}
  

/**********************************************************************
 *
 * Set the Mask Disruptor flag
 *
 *********************************************************************/
void as3935_set_mask_disturber(INT8U mask) {
  REG_u reg3;

  reg3 = i2c_read(AS3935_ADDR, REG03);
  reg3.R3.MASK_DIST = mask;
  i2c_write(AS3935_ADDR, REG03, reg3);
}
    

/**********************************************************************
 *
 * Reset chip back to normal
 *
 *********************************************************************/
void as3935_reset_registers(void) {
  REG_u val;
  val.data = MAGIC_VALUE;
  i2c_write(AS3935_ADDR, REG_RESET, val);
}


/**********************************************************************
 *
 * Display LCO freq on irq pin
 *
 *********************************************************************/
void as3935_display_responance_freq_on_irq(INT8U on) {
  REG_u reg8;

  reg8 = i2c_read(AS3935_ADDR, REG08);
  reg8.R8.DISP_LCO = on;
  i2c_write(AS3935_ADDR, REG08, reg8);
}    

 
/**********************************************************************
 *
 * Display SRCO freq on irq pin
 *
 *********************************************************************/
void as3935_display_srco_on_irq(INT8U on) {
  REG_u reg8;

  reg8 = i2c_read(AS3935_ADDR, REG08);
  reg8.R8.DISP_SRCO = on;
  i2c_write(AS3935_ADDR, REG08, reg8);
}    


/**********************************************************************
 *
 * Display TRCO freq on irq pin
 *
 *********************************************************************/
void as3935_display_trco_on_irq(INT8U on) {
  REG_u reg8;

  reg8 = i2c_read(AS3935_ADDR, REG08);
  reg8.R8.DISP_TRCO = on;
  i2c_write(AS3935_ADDR, REG08, reg8);
}    

  
/**********************************************************************
 *
 * Set the tune capacitor
 *
 *********************************************************************/
void as3935_set_tune_cap(INT8U cap) {
  REG_u reg8;

  reg8 = i2c_read(AS3935_ADDR, REG08);
  reg8.R8.TUN_CAP = cap;
  i2c_write(AS3935_ADDR, REG08, reg8);
}


/**********************************************************************
 *
 * Get the tune cap value
 *
 *********************************************************************/
INT8U as3935_get_tune_cap(void) {
  REG_u reg8;

  reg8 = i2c_read(AS3935_ADDR, REG08);

  return reg8.R8.TUN_CAP;
}


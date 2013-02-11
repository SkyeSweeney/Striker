/**********************************************************************
 *
 * AS3935 Lighting detector project.
 *
 * Arduino is connected to an Embedded Adventure AS3935 lightning
 * detector breakout board (BOB). The BOB is connected using the
 * I2C SDA and SCL lines, as well as power (5V0) and ground. The
 * last connection is the interrupt line from the BOB going to
 * pin 2 (int0) of the Arduino.
 *
 * The serial port of the Arduino reports to a master what events
 * are occuring.
 *
 *********************************************************************/
#include <Wire.h>
#include "types.h"
#include "as3935.h"


/**********************************************************************
 *
 * Global data
 *
 *********************************************************************/
volatile INT32U counter = 0; /* ISR counter */
volatile INT8U  isrFlag = 0; /* ISR flag */


/**********************************************************************
 *
 * Setup routine
 *
 *********************************************************************/
void setup(void) {
  REG_u reg;
  INT8U err;
  
  /* Open I2C library */
  Wire.begin();
  
  /* Open serial port */
  Serial.begin(115200);
  Serial.println("start");
  
  /* For debugging */
  pinMode(7, OUTPUT);
  pinMode(6, OUTPUT);

  /* Must read a register other than 0 at the start */
  /* This appears to be a limitation with the AS3935 */
  err = i2c_read(AS3935_ADDR, REG01, &reg);
  
  /* Attach the ISR */
  attachInterrupt(0, as3935_isr, RISING);
  
  /* Set unit into operation mode */
  as3935_set_powerdown(0);
  
  /* Calibrate the unit */
  as3935_calibrate();
  
} /* end setup */



/**********************************************************************
 *
 * Dump the first n registers
 *
 *********************************************************************/
void as3935_dump(INT8U n) {
  INT8U  i;
  REG_u  reg;
  INT8U  err;
  
  for (i=0; i<n; i++) {
    err = i2c_read(AS3935_ADDR, (RegisterID_e)i, &reg);
    Serial.print("Reg ");
    Serial.print(i);
    Serial.print(" = ");
    Serial.println(reg.data, HEX);
  }
   
} /* end as3935_dump */


/**********************************************************************
 *
 * I2C read routine
 * Returns:
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
  
  /* Make sure we have the byte we needed */
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





/**********************************************************************
 *
 * Main loop
 *
 *********************************************************************/
void loop(void) {
  InterruptReason_e reason;
  static INT32U then=0;
  INT32U now;
  
  /* If the ISR flag has been set */
  if (isrFlag) {
    
    isrFlag = 0;
    
    delay(2+1); // Delay claimed by datasheet
    reason = as3935_get_interrupt_reason();
    
    Serial.print("ISR: ");
    Serial.print(reason);
    Serial.print(" ");
    
    switch (reason){
      case INT_NONE:
        Serial.println("None?");
        break;
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
  now = millis();
  if (now > then) {
    Serial.print(".");
    then = now + 1000;
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
  INT8U err;
  
  err = i2c_read(AS3935_ADDR, REG00, &reg0);
  reg0.R0.PWD = pwr;
  err = i2c_write(AS3935_ADDR, REG00, reg0);
}


/**********************************************************************
 *
 * Get the powerdown status
 *
 *********************************************************************/
INT8U as3935_get_powerdown(void) {
  REG_u reg0;
  INT8U err;
  
  err = i2c_read(AS3935_ADDR, REG00, &reg0);
  return reg0.R0.PWD;
}
  

/**********************************************************************
 *
 * Set the gain
 *
 *********************************************************************/
void as3935_set_afe(INT8U gain) {
  REG_u reg0;
  INT8U err;
  
  err = i2c_read(AS3935_ADDR, REG00, &reg0);
  reg0.R0.AFE_GB = gain;
  err = i2c_write(AS3935_ADDR, REG00, reg0);
}


/**********************************************************************
 *
 * Return the current gain
 *
 *********************************************************************/
INT8U as3935_get_afe(void) {
  REG_u reg0;
  INT8U err;

  err = i2c_read(AS3935_ADDR, REG00, &reg0);

  return reg0.R0.AFE_GB;
}


/**********************************************************************
 *
 * Set watchdog threshold
 *
 *********************************************************************/
void as3935_set_watchdog_threshold(INT8U threshold) {
  REG_u reg1;
  INT8U err;
  
  err = i2c_read(AS3935_ADDR, REG01, &reg1);
  reg1.R1.WDTH = threshold;
  err = i2c_write(AS3935_ADDR, REG01, reg1);
}


/**********************************************************************
 *
 * Get the watchdog threshold
 *
 *********************************************************************/
INT8U as3935_get_watchdog_threshold(void) {
  REG_u reg1;
  INT8U err;

  err = i2c_read(AS3935_ADDR, REG01, &reg1);

  return reg1.R1.WDTH;
}
  


/**********************************************************************
 *
 * Set the noise floor level
 *
 *********************************************************************/
void as3935_set_noise_floor_level(INT8U nfl) {
  REG_u reg1;
  INT8U err;
  
  err = i2c_read(AS3935_ADDR, REG01, &reg1);
  reg1.R1.NF_LEV = nfl;
  err = i2c_write(AS3935_ADDR, REG01, reg1);
}


/**********************************************************************
 *
 * Get the noise floor level
 *
 *********************************************************************/
INT8U as3935_get_noise_floor_level(void) {
  REG_u reg1;
  INT8U err;

  err = i2c_read(AS3935_ADDR, REG01, &reg1);

  return reg1.R1.NF_LEV;
}


/**********************************************************************
 *
 * Set the minimum number of strikes
 *
 *********************************************************************/
void as3935_set_spike_rejection(INT8U srej) {
  REG_u reg2;
  INT8U err;

  err = i2c_read(AS3935_ADDR, REG02, &reg2);
  reg2.R2.SREJ = srej;
  err = i2c_write(AS3935_ADDR, REG02, reg2);
  
}


/**********************************************************************
 *
 * Get the minimum number of strikes
 *
 *********************************************************************/
INT8U as3935_get_spike_rejection(void) {
  REG_u reg2;
  INT8U err;

  err = i2c_read(AS3935_ADDR, REG02, &reg2);

  return reg2.R2.SREJ;
}  


/**********************************************************************
 *
 * Calibrate the RCO
 *
 *********************************************************************/
void as3935_calibrate_rco(void) {
  REG_u  reg8;
  INT8U  err;
  INT8U  v = 0x61;
  
  
  // Send Direct command CALIB_RCO
  err = i2c_write(AS3935_ADDR, REG_CAL_RCO, *((REG_u*)&v));

  // Wait for it to stabilize
  delay(10);

  // Set the display TRCO bit
  err = i2c_read(AS3935_ADDR, REG08, &reg8);
  reg8.R8.DISP_TRCO = 1;
  err = i2c_write(AS3935_ADDR, REG08, reg8);
  
  // Wait 
  delay(10);
  
  // Clear the display TRCO bit
  reg8.R8.DISP_TRCO = 0;
  err = i2c_write(AS3935_ADDR, REG08, reg8);
  
}  


/**********************************************************************
 *
 * Get energy value from last strike
 *
 *********************************************************************/
INT32U as3935_get_energy_calc(void) {
  Energy_u e;
  REG_u    reg, reg6;
  INT8U    err;

  e.bits8[0] = 0;
  
  //  REG0x06[4:0] S_LIG_MM
  err = i2c_read(AS3935_ADDR, REG06, &reg6);
  e.bits8[1] = reg6.R6.S_LIG_MM;
  
  //   REG0x05[7:0] S_LIG_M
  err = i2c_read(AS3935_ADDR, REG05, &reg);
  e.bits8[2] = reg.data;
  
  //  REG0x04[7:0]  S_LIG_L
  err = i2c_read(AS3935_ADDR, REG04, &reg);
  e.bits8[3] = reg.data;
  
  return e.val;
  
}


/**********************************************************************
 *
 * Get storm distance
 *
 *********************************************************************/
INT8U as3935_get_storm_distance(void) {
  REG_u reg7;
  INT8U err;

  err = i2c_read(AS3935_ADDR, REG07, &reg7);

  return reg7.R7.DISTANCE;
}


/**********************************************************************
 *
 * Get reason for interrupt
 *
 *********************************************************************/
InterruptReason_e as3935_get_interrupt_reason(void) {
  REG_u reg3;
  INT8U err;

  // Datasheet indicates that the INT field only updates after 2ms!!!
  // What a piece of crap.
  delay(2); 
  err = i2c_read(AS3935_ADDR, REG03, &reg3);

  return((InterruptReason_e)reg3.R3.INT);
}  


/**********************************************************************
 *
 * Set the minimum number of strikes needed
 *
 *********************************************************************/
void as3935_set_minimum_lightning(MinStrikes_e min) {
  REG_u reg2;
  INT8U err;

  err = i2c_read(AS3935_ADDR, REG02, &reg2);
  reg2.R2.MIN_NUM_LIGHT = min;
  err = i2c_write(AS3935_ADDR, REG02, reg2);
}


/**********************************************************************
 *
 * Get the minimum number of strikes
 *
 *********************************************************************/
MinStrikes_e as3935_get_minimum_lightning(void) {
  REG_u reg2;
  INT8U err;
  
  err = i2c_read(AS3935_ADDR, REG02, &reg2);

  return (MinStrikes_e)reg2.R2.MIN_NUM_LIGHT;
}


/**********************************************************************
 *
 * Clear statistics
 *
 *********************************************************************/
void as3936_clear_statistics(void) {
  REG_u reg2;
  INT8U err;

  err = i2c_read(AS3935_ADDR, REG02, &reg2);
  reg2.R2.CL_STAT = 1;
  err = i2c_write(AS3935_ADDR, REG02, reg2);
  reg2.R2.CL_STAT = 0;
  err = i2c_write(AS3935_ADDR, REG02, reg2);
  reg2.R2.CL_STAT = 1;
  err = i2c_write(AS3935_ADDR, REG02, reg2);
}


/**********************************************************************
 *
 * Set the LCO divider
 *
 *********************************************************************/
void as3935_set_freq_div_ratio(LCO_DIV_e div_ratio) {
  REG_u reg3;
  INT8U err;

  err = i2c_read(AS3935_ADDR, REG03, &reg3);
  reg3.R3.LCO_FDIV = div_ratio;
  err = i2c_write(AS3935_ADDR, REG03, reg3);
}
  

/**********************************************************************
 *
 * Set the Mask Disruptor flag
 *
 *********************************************************************/
void as3935_set_mask_disturber(INT8U mask) {
  REG_u reg3;
  INT8U err;

  err = i2c_read(AS3935_ADDR, REG03, &reg3);
  reg3.R3.MASK_DIST = mask;
  err = i2c_write(AS3935_ADDR, REG03, reg3);
}
    

/**********************************************************************
 *
 * Reset chip back to normal
 *
 *********************************************************************/
void as3935_reset_registers(void) {
  REG_u val;
  INT8U err;
  
  val.data = MAGIC_VALUE;
  err = i2c_write(AS3935_ADDR, REG_RESET, val);
}


/**********************************************************************
 *
 * Display LCO freq on irq pin
 *
 *********************************************************************/
void as3935_display_responance_freq_on_irq(INT8U on) {
  REG_u reg8;
  INT8U err;

  err = i2c_read(AS3935_ADDR, REG08, &reg8);
  reg8.R8.DISP_LCO = on;
  err = i2c_write(AS3935_ADDR, REG08, reg8);
}    

 
/**********************************************************************
 *
 * Display SRCO freq on irq pin
 *
 *********************************************************************/
void as3935_display_srco_on_irq(INT8U on) {
  REG_u reg8;
  INT8U err;

  err = i2c_read(AS3935_ADDR, REG08, &reg8);
  reg8.R8.DISP_SRCO = on;
  err = i2c_write(AS3935_ADDR, REG08, reg8);
}    


/**********************************************************************
 *
 * Display TRCO freq on irq pin
 *
 *********************************************************************/
void as3935_display_trco_on_irq(INT8U on) {
  REG_u reg8;
  INT8U err;

  err = i2c_read(AS3935_ADDR, REG08, &reg8);
  reg8.R8.DISP_TRCO = on;
  err = i2c_write(AS3935_ADDR, REG08, reg8);
}    

  
/**********************************************************************
 *
 * Set the tune capacitor
 *
 *********************************************************************/
void as3935_set_tune_cap(INT8U cap) {
  REG_u reg8;
  INT8U err;

  err = i2c_read(AS3935_ADDR, REG08, &reg8);
  reg8.R8.TUN_CAP = cap;
  err = i2c_write(AS3935_ADDR, REG08, reg8);
}


/**********************************************************************
 *
 * Get the tune cap value
 *
 *********************************************************************/
INT8U as3935_get_tune_cap(void) {
  REG_u reg8;
  INT8U err;

  err = i2c_read(AS3935_ADDR, REG08, &reg8);

  return reg8.R8.TUN_CAP;
}

/**********************************************************************
 *
 * Calibrate
 *
 *********************************************************************/
void as3935_calibrate(void) {
  INT8U   bestTuneValue = 0;
  INT8U   bestDivider   = 0;
  INT8U   i;
  INT32U bestCountError = 100000;
  INT32U err;
  INT16U target;
  INT32U cnt;
 
  /* Put the LCO onto the interrupt pin */
  as3935_display_responance_freq_on_irq(1);
    
  /* Set the LCO output divider to 16 */
  as3935_set_freq_div_ratio(LCO_DIV_16);

  /* Antenna should be outputing 500KHz +- 3.5% or 17.5KHz */
#define TARGET 1250 /* 500000 / 16 * 0.04 */  
      
  /* Do for each tuning selection */
  for (i=0; i<16; i++) {
    
    /* Set the tuning selection */
    as3935_set_tune_cap(i);

    /* Wait for it to setle */
    delay(10);
    
    /* Measure the number of interrupts in a set amount of time */
    noInterrupts();
    counter = 0;
    interrupts();
    
    delay(40);
    
    /* Capture the best value */
    noInterrupts();
    cnt = counter;
    interrupts();
    
    /* Determine absolute error */
    if (cnt > TARGET) {
      err = cnt - TARGET;
    } else {
      err = TARGET - cnt;
    }

#ifdef DEBUG
    Serial.print("Tune: ");
    Serial.print(i);
    Serial.print(" = ");
    Serial.println(err);
#endif
    
    /* Capture the smallest error */
    if (err < bestCountError) {
      bestTuneValue = i;
      bestCountError = err;
    }
    
  } /* Do next tune selection */
    
#ifdef DEBUG  
  Serial.print("Best tune value: ");
  Serial.println(bestTuneValue);
#endif  

  /* Now set the tune value of the best match */
  as3935_set_tune_cap(bestTuneValue);

  /* Restore interrupt pin to normal operation */
  as3935_display_responance_freq_on_irq(0);
        
} /* end calibrate */


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
#include "MyTypes.h"
#include "as3935.h"
#include "I2C.h"


/**********************************************************************
 *
 * Standard error handling
 *
 *********************************************************************/
void as3935_err(INT8U err, char *str) {
  if (err != 0) {
    Serial.print("as3935 error: ");
    Serial.print(err);
    Serial.print(" at: ");
    Serial.println(str);
  }
}


/**********************************************************************
 *
 * Dump the first n registers
 *
 *********************************************************************/
INT8U as3935_dump(INT8U n) {
  INT8U  i;
  REG_u  reg;
  INT8U  err;
  
  for (i=1; i<n; i++) {
    err = i2c_read(AS3935_ADDR, (RegisterID_e)i, &reg);
    if (err != 0) {
      Serial.println("Failed");
      break;
    }
    Serial.print("Reg ");
    Serial.print(i);
    Serial.print(" = ");
    Serial.println(reg.data, HEX);
  }
  
  return(err);
   
} /* end as3935_dump */


/**********************************************************************
 *
 * Set the powerdown flag
 *
 *********************************************************************/
INT8U as3935_set_powerdown(INT8U pwr) {
  REG_u reg0;
  INT8U err;
  
  err = i2c_read(AS3935_ADDR, REG00, &reg0);
  if (err == 0) {
    reg0.R0.PWD = pwr;
    err = i2c_write(AS3935_ADDR, REG00, reg0);
  }
  return(err);
}


/**********************************************************************
 *
 * Get the powerdown status
 *
 *********************************************************************/
INT8U as3935_get_powerdown(INT8U *val) {
  REG_u reg0;
  INT8U err;
  
  err = i2c_read(AS3935_ADDR, REG00, &reg0);
  if (err == 0) {
    *val = reg0.R0.PWD;
  } else {
    *val = 0;
  }
  return(err);
}
  

/**********************************************************************
 *
 * Set the gain
 *
 *********************************************************************/
INT8U as3935_set_afe(INT8U gain) {
  REG_u reg0;
  INT8U err;
  
  err = i2c_read(AS3935_ADDR, REG00, &reg0);
  if (err == 0) {
    reg0.R0.AFE_GB = gain;
    err = i2c_write(AS3935_ADDR, REG00, reg0);
  }
  return(err);
}


/**********************************************************************
 *
 * Return the current gain
 *
 *********************************************************************/
INT8U as3935_get_afe(INT8U *val) {
  REG_u reg0;
  INT8U err;

  err = i2c_read(AS3935_ADDR, REG00, &reg0);
  if (err == 0) {
    *val = reg0.R0.AFE_GB;
  } else {
    *val = 0;
  }
  return(err);
}


/**********************************************************************
 *
 * Set watchdog threshold
 *
 *********************************************************************/
INT8U as3935_set_watchdog_threshold(INT8U threshold) {
  REG_u reg1;
  INT8U err;
  
  err = i2c_read(AS3935_ADDR, REG01, &reg1);
  if (err == 0) {
    reg1.R1.WDTH = threshold;
    err = i2c_write(AS3935_ADDR, REG01, reg1);
  }
  return(err);
}


/**********************************************************************
 *
 * Get the watchdog threshold
 *
 *********************************************************************/
INT8U as3935_get_watchdog_threshold(INT8U *val) {
  REG_u reg1;
  INT8U err;

  err = i2c_read(AS3935_ADDR, REG01, &reg1);
  if (err == 0) {
    *val = reg1.R1.WDTH;
  } else {
    *val = 0;
  }
  return(err);
}
  


/**********************************************************************
 *
 * Set the noise floor level
 *
 *********************************************************************/
INT8U as3935_set_noise_floor_level(INT8U nfl) {
  REG_u reg1;
  INT8U err;
  
  err = i2c_read(AS3935_ADDR, REG01, &reg1);
  if (err == 0) {
    reg1.R1.NF_LEV = nfl;
    err = i2c_write(AS3935_ADDR, REG01, reg1);
  }
  return(err);
}


/**********************************************************************
 *
 * Get the noise floor level
 *
 *********************************************************************/
INT8U as3935_get_noise_floor_level(INT8U *val) {
  REG_u reg1;
  INT8U err;

  err = i2c_read(AS3935_ADDR, REG01, &reg1);
  if (err == 0) {
    *val = reg1.R1.NF_LEV;
  } else {
    *val = 0;
  }
  return(err);
}


/**********************************************************************
 *
 * Set the minimum number of strikes
 *
 *********************************************************************/
INT8U as3935_set_spike_rejection(INT8U srej) {
  REG_u reg2;
  INT8U err;

  err = i2c_read(AS3935_ADDR, REG02, &reg2);
  if (err == 0) {
    reg2.R2.SREJ = srej;
    err = i2c_write(AS3935_ADDR, REG02, reg2);
  } 
  return(err);
  
}


/**********************************************************************
 *
 * Get the minimum number of strikes
 *
 *********************************************************************/
INT8U as3935_get_spike_rejection(INT8U *val) {
  REG_u reg2;
  INT8U err;

  err = i2c_read(AS3935_ADDR, REG02, &reg2);
  if (err == 0) {
    *val = reg2.R2.SREJ;
  } else {
    *val = 0;
  }
  return(err);
}  


/**********************************************************************
 *
 * Calibrate the RCO
 *
 *********************************************************************/
INT8U as3935_calibrate_rco(void) {
  REG_u  reg8;
  INT8U  err;
  INT8U  v = 0x61;
  
  
  // Send Direct command CALIB_RCO
  err = i2c_write(AS3935_ADDR, REG_CAL_RCO, *((REG_u*)&v));
  
  if (err == 0) {

    // Wait for it to stabilize
    delay(10);
  
    // Set the display TRCO bit
    err = i2c_read(AS3935_ADDR, REG08, &reg8);
    if (err == 0) {
      reg8.R8.DISP_TRCO = 1;
      err = i2c_write(AS3935_ADDR, REG08, reg8);
      if (err == 0) {
        
        // Wait 
        delay(10);
        
        // Clear the display TRCO bit
        reg8.R8.DISP_TRCO = 0;
        err = i2c_write(AS3935_ADDR, REG08, reg8);
      }
    }
  }
  return (err);
  
}  


/**********************************************************************
 *
 * Get energy value from last strike
 *
 *********************************************************************/
INT8U as3935_get_energy_calc(INT32U *val) {
  Energy_u e;
  REG_u    reg, reg6;
  INT8U    err;

  *val = 0;
  e.bits8[0] = 0;
  
  //  REG0x06[4:0] S_LIG_MM
  err = i2c_read(AS3935_ADDR, REG06, &reg6);
  if (err == 0) {
    
    e.bits8[1] = reg6.R6.S_LIG_MM;
    
    //   REG0x05[7:0] S_LIG_M
    err = i2c_read(AS3935_ADDR, REG05, &reg);
    if (err == 0) {
      e.bits8[2] = reg.data;
      
      //  REG0x04[7:0]  S_LIG_L
      err = i2c_read(AS3935_ADDR, REG04, &reg);
      if (err == 0) {
        e.bits8[3] = reg.data;
      
        *val = e.val;
      }
    }
  }
  return(err);
}


/**********************************************************************
 *
 * Get storm distance
 *
 *********************************************************************/
INT8U as3935_get_storm_distance(INT8U *val) {
  REG_u reg7;
  INT8U err;
 
  *val = 0;

  err = i2c_read(AS3935_ADDR, REG07, &reg7);
  if (err == 0) {
    *val = reg7.R7.DISTANCE;
  }
  return(err);
}


/**********************************************************************
 *
 * Get reason for interrupt
 *
 *********************************************************************/
INT8U as3935_get_interrupt_reason(InterruptReason_e *val) {
  REG_u reg3;
  INT8U err;
  
  *val = INT_NONE;

  /* Datasheet indicates that the INT field only updates after 2ms!!!
   * What a piece of crap. */
  delay(2); 
  err = i2c_read(AS3935_ADDR, REG03, &reg3);
  if (err == 0) {
    *val = (InterruptReason_e)reg3.R3.INT;
  }
  return(err);
}  


/**********************************************************************
 *
 * Set the minimum number of strikes needed
 *
 *********************************************************************/
INT8U as3935_set_minimum_lightning(MinStrikes_e min) {
  REG_u reg2;
  INT8U err;

  err = i2c_read(AS3935_ADDR, REG02, &reg2);
  if (err == 0) {
    reg2.R2.MIN_NUM_LIGHT = min;
    err = i2c_write(AS3935_ADDR, REG02, reg2);
  }
  return(err);
}


/**********************************************************************
 *
 * Get the minimum number of strikes
 *
 *********************************************************************/
INT8U as3935_get_minimum_lightning(MinStrikes_e *val) {
  REG_u reg2;
  INT8U err;
  
  err = i2c_read(AS3935_ADDR, REG02, &reg2);
  if (err == 0) {
    *val = (MinStrikes_e)reg2.R2.MIN_NUM_LIGHT;
  } else {
    *val = MIN_1;
  }
  return(err);
}


/**********************************************************************
 *
 * Clear statistics
 *
 *********************************************************************/
INT8U as3936_clear_statistics(void) {
  REG_u reg2;
  INT8U err;

  err = i2c_read(AS3935_ADDR, REG02, &reg2);
  if (err == 0) {
    reg2.R2.CL_STAT = 1;
    err = i2c_write(AS3935_ADDR, REG02, reg2);
    if (err == 0) {
      reg2.R2.CL_STAT = 0;
      err = i2c_write(AS3935_ADDR, REG02, reg2);
      if (err == 0) {
        reg2.R2.CL_STAT = 1;
        err = i2c_write(AS3935_ADDR, REG02, reg2);
      }
    }
  }
  return(err);
}


/**********************************************************************
 *
 * Set the LCO divider
 *
 *********************************************************************/
INT8U as3935_set_freq_div_ratio(LCO_DIV_e div_ratio) {
  REG_u reg3;
  INT8U err;

  err = i2c_read(AS3935_ADDR, REG03, &reg3);
  if (err == 0) {
    reg3.R3.LCO_FDIV = div_ratio;
    err = i2c_write(AS3935_ADDR, REG03, reg3);
  }
  return(err);
}
  

/**********************************************************************
 *
 * Set the Mask Disruptor flag
 *
 *********************************************************************/
INT8U as3935_set_mask_disturber(INT8U mask) {
  REG_u reg3;
  INT8U err;

  err = i2c_read(AS3935_ADDR, REG03, &reg3);
  if (err == 0) {
    reg3.R3.MASK_DIST = mask;
    err = i2c_write(AS3935_ADDR, REG03, reg3);
  }
  return(err);
}
    

/**********************************************************************
 *
 * Reset chip back to normal
 *
 *********************************************************************/
INT8U as3935_reset_registers(void) {
  REG_u val;
  INT8U err;
  
  val.data = MAGIC_VALUE;
  err = i2c_write(AS3935_ADDR, REG_RESET, val);
  return(err);
}


/**********************************************************************
 *
 * Display LCO freq on irq pin
 *
 *********************************************************************/
INT8U as3935_display_responance_freq_on_irq(INT8U on) {
  REG_u reg8;
  INT8U err;

  err = i2c_read(AS3935_ADDR, REG08, &reg8);
  if (err == 0) {
    reg8.R8.DISP_LCO = on;
    err = i2c_write(AS3935_ADDR, REG08, reg8);
  } 
  return(err);
}    

 
/**********************************************************************
 *
 * Display SRCO freq on irq pin
 *
 *********************************************************************/
INT8U as3935_display_srco_on_irq(INT8U on) {
  REG_u reg8;
  INT8U err;

  err = i2c_read(AS3935_ADDR, REG08, &reg8);
  if (err == 0) {
    reg8.R8.DISP_SRCO = on;
    err = i2c_write(AS3935_ADDR, REG08, reg8);
  }
  return(err);
}    


/**********************************************************************
 *
 * Display TRCO freq on irq pin
 *
 *********************************************************************/
INT8U as3935_display_trco_on_irq(INT8U on) {
  REG_u reg8;
  INT8U err;

  err = i2c_read(AS3935_ADDR, REG08, &reg8);
  if (err == 0) {
    reg8.R8.DISP_TRCO = on;
    err = i2c_write(AS3935_ADDR, REG08, reg8);
  }
  return(err);
}    

  
/**********************************************************************
 *
 * Set the tune capacitor
 *
 *********************************************************************/
INT8U as3935_set_tune_cap(INT8U cap) {
  REG_u reg8, reg8a;
  INT8U err;
  INT8U i;

  err = i2c_read(AS3935_ADDR, REG08, &reg8);
  if (err == 0) {
    reg8.R8.TUN_CAP = cap;
    err = i2c_write(AS3935_ADDR, REG08, reg8);
  }
  return(err);
}


/**********************************************************************
 *
 * Get the tune cap value
 *
 *********************************************************************/
INT8U as3935_get_tune_cap(INT8U *val) {
  REG_u reg8;
  INT8U err;

  err = i2c_read(AS3935_ADDR, REG08, &reg8);
  if (err == 0) {
    *val = reg8.R8.TUN_CAP;
  } else {
    *val = 0;
  }
  return(err);
}


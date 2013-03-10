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

#define DEBUG

#define SEC_TO_MS  (1000L)
#define MIN_TO_MS  (1000L*60L)
#define HR_TO_MS   (1000L*60L*60L)

#define ISR_DELAY (3)

/**********************************************************************
 *
 * Global data
 *
 *********************************************************************/
volatile INT32U counter = 0; /* ISR counter */
volatile INT8U  isrFlag = 0; /* Normal ISR flag */
volatile INT8U  bitFlag = 0; /* BIT ISR flag */
INT32U dotTime=0;
INT32U calTime=0;
INT32U bitTime=0;
INT8U             hbCnt;


/**********************************************************************
 *
 * Setup routine
 *
 *********************************************************************/
void setup(void) {
  REG_u  reg;
  INT8U  err;
  INT32U now;
  
  /* Open I2C library */
  Wire.begin();
  
  pinMode(7, OUTPUT);
  pinMode(6, OUTPUT);
  
  digitalWrite(6, LOW);
  digitalWrite(7, LOW);
  
  /* Open serial port */
  Serial.begin(115200);

  /* Indicate we started */
  Serial.println("Striker starting");

  /* Must read a register other than 0 at the start */
  /* This appears to be a limitation with the AS3935 */
  err = i2c_read(AS3935_ADDR, REG01, &reg);
  
  /* Attach the ISR */
  attachInterrupt(0, normalIsr, RISING);
  
  /* Set unit into operation mode */
  as3935_set_powerdown(0);

  now = millis();
  dotTime = now+1*SEC_TO_MS;   /* First heartbeat in one second */
  calTime = now+1*SEC_TO_MS;   /* First cal to be done in one second */
  bitTime = now+10*SEC_TO_MS;  /* First BIT to be done in 10 seconds */
  
  
} /* end setup */


/**********************************************************************
 *
 * Main loop
 *
 *********************************************************************/
void loop(void) {
  InterruptReason_e reason;
  INT32U            now;
  INT8U             bitResults;
  INT8U             calResult;

  readTest();
  
  /* If the ISR flag has been set */
  if (isrFlag) {
    
    isrFlag = 0;
    
    delay(ISR_DELAY); // Delay claimed by datasheet
    reason = as3935_get_interrupt_reason();
    
    Serial.print("ISR: ");
    
    switch (reason) {

      case INT_NONE:
        //Serial.println("None?");
        break;

      case INT_NOISY:
        Serial.println("Noisy");
        break;

      case INT_DISTURBER:
        Serial.println("Disturber");
        break;

      case INT_STRIKE:
        Serial.println("Strike");
        break;

      default:
        Serial.println("Unkown ISR");
        break;
    }

  }

  /* Get current time */
  now = millis();

  /***************************/
  /* Execute scheduled tasks */
  /***************************/

  /* Print a dot once a second for a heart beat */
  if (now > dotTime) {

    dotTime = now + 1*SEC_TO_MS;
    Serial.print(".");
    hbCnt++;
    if (hbCnt >= 72) {
      Serial.println("");
      hbCnt = 0;
    }

  }

  /* Perform calibration every 30 minutes */
  if (now > calTime) {

    calTime = now + 30*MIN_TO_MS;
    Serial.println("Calibration starting");
    calResult = calibrate();

    /* Process calibration results */
    if (calResult != 1) {
      Serial.println("Calibration failed");
    }

  }

  /* Perform self test every 60 minutes */
  if (now > bitTime) {

    bitTime = now + 60*MIN_TO_MS;

    /* Run BIT */
    bitResults = bitTest();

    /* Process BIT results */
    if (bitResults != 1) {
      Serial.println("BIT failed");
    }

  }
  
}


/**********************************************************************
 *
 * Normal ISR for the AS3935 interrupt
 *
 *********************************************************************/
void normalIsr(void) {
  /* Due to the stupidity of this chip, the event can't be handled in
   * the ISR. This is because the 'reason' register is not available
   * till 2 ms AFTER the interrupt occurs! Totaly nuts.
   * So the only thing we can do is to set a flag and use that in the
   * main loop to poll against.
   * We will also increment a counter here for when we perform auto cal.
   */
   
   //isrFlag = 1;
}

/**********************************************************************
 *
 * Calibration ISR for the AS3935 interrupt
 *
 *********************************************************************/
void calIsr(void) {
   counter++;
}

/**********************************************************************
 *
 * BIT ISR for the AS3935 interrupt
 *
 *********************************************************************/
void bitIsr(void) {
   bitFlag = 1;
}


/**********************************************************************
 *
 *********************************************************************/
void readTest(void) {
  INT8U e;
  REG_u reg, reg8;
  unsigned long i;

  e = i2c_read(AS3935_ADDR, REG08, &reg8);
  reg8.R8.DISP_LCO = 1;
  reg8.R8.TUN_CAP  = 15;
  e = i2c_write(AS3935_ADDR, REG08, reg8);
  
  delay(10);

#if 1
  for (i=0; i<100000; i++) {
    
    // Read a random register
    //e = i2c_read(AS3935_ADDR, (RegisterID_e)(i%15), &reg);
    
    // Read R8 should be 0x8f
    e = i2c_read(AS3935_ADDR, REG08, &reg8);
    if ((reg8.data != 0x8f) || (e != 0)) {
      PORTD = PORTD | 0x80;
      //digitalWrite(7, HIGH);
      Serial.print("Read failed: ");
      Serial.print(reg8.data, HEX);
      Serial.print(" err:");
      Serial.print(e);
      Serial.println("");
      PORTD = PORTD & ~0x80;
      //DigitalWrite(7, LOW);
    } else {
      PORTD = PORTD | 0x40;
      PORTD = PORTD & ~0x40;
    }
    //Serial.println(e);
    delay(1);
  }
  
  Serial.println("Test done");
  for(;;){delay(1);}

#endif

#if 1
  for (i=0; i<10000 ; i++) {
    
    // Write register 8
    e = i2c_write(AS3935_ADDR, REG08, reg8);
    
    // Read it back; Should be 0x8f
    e = i2c_read(AS3935_ADDR, REG08, &reg8);
    if (reg8.data != 0x8f) {
      digitalWrite(7, HIGH);
      Serial.println(reg8.data, HEX);
      Serial.println("write/read failed");
      for(;;){delay(1);}
      break;
    }
    delay(1);
  }
#endif
  
#if 0
  for (i=0; i<1000; i++) {
    e = i2c_read(AS3935_ADDR, REG08, &reg8);
    reg8.R8.TUN_CAP = i%16;
    e = i2c_write(AS3935_ADDR, REG08, reg8);
    e = i2c_read(AS3935_ADDR, REG08, &reg8);
    if ((reg8.R8.DISP_LCO != 1) || (reg8.R8.TUN_CAP != i%16)) {
      digitalWrite(7, HIGH);
      Serial.println(reg8.data, HEX);
      Serial.println("read/write failed");
      for(;;){delay(1);}
      break;
    }
    delay(1);
  }
#endif


  Serial.println("Testing done");
  
  for (;;) {
    delay(1);
  }

 
}


/**********************************************************************
 *
 * Calibrate
 * returns 1 for pass
 *********************************************************************/
INT8U calibrate(void) {
  INT8U   bestTuneValue = 0;
  INT8U   bestDivider   = 0;
  INT8U   i;
  INT32U  bestTuneError = 100000;
  INT32U  err;
  INT16U  target;
  INT32U  cnt;
  INT8U   retval;
  
  digitalWrite(6, HIGH);
  
  /* Attach the calibration ISR */
  Serial.println("Install ISR");
  attachInterrupt(0, calIsr, RISING);
  
  /* Set the LCO output divider to 16 */
  as3935_set_freq_div_ratio(LCO_DIV_16);
 
  /* Put the LCO onto the interrupt pin */
  Serial.println("Put LCO on ISR pin");
  as3935_display_responance_freq_on_irq(1);
    

  /********************************************************************
   * Antenna should be tuned to 500KHz +- 3.5%
   * Since we have set the divider to 16, we should see a 31.25 KHz
   * square wave at the interrupt pin. 
   * If we sample this for 40ms we should get 1250 counts +- 3.5%
   * 3.5% of 1250 is +- 43.75 counts. Make sure our choice meets this.
   *
   *******************************************************************/
#define TARGET         1250         /* 500000 / 16 * 0.04 */  
#define ERR_THRESHOLD  43           /* 1250 * 3.5 / 100 */
      
  /* Do for each tuning selection */
  for (i=0; i<16; i++) {
  
    Serial.print("Trying ");
    Serial.print(i);
    
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
    
    Serial.print(" cnt:");
    Serial.print(cnt);
    Serial.print(" ");
    
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
    if (err < bestTuneError) {
      bestTuneValue  = i;
      bestTuneError  = err;
    }
    
  } /* Do next tune selection */
    
#ifdef DEBUG  
  Serial.print("Best tune value: ");
  Serial.println(bestTuneValue);
#endif  

  /* Insure this error meets the 3.5% */
  if (bestTuneError < ERR_THRESHOLD) {
    Serial.println("Cal pass");
    retval = 1;
  } else {
    Serial.println("Cal failed");
    retval = 0;
  }

  /* Now set the tune value of the best match */
  as3935_set_tune_cap(bestTuneValue);

  /* Restore interrupt pin to normal operation */
  as3935_display_responance_freq_on_irq(0);
  
  /* Attach the normal ISR */
  attachInterrupt(0, normalIsr, RISING);
  
  digitalWrite(6, LOW);

  return retval;
        
} /* end calibrate */


/**********************************************************************
 *
 * Built in test
 *
 *********************************************************************/
INT8U bitTest(void) {
  INT8U retval;
  InterruptReason_e reason;

  /* Attach the BIT isr */
  attachInterrupt(0, bitIsr, RISING);

  bitFlag = 0;

  /* Turn on the noise generator for n ms */
  // TODO

  /* Wait a bit for interrupt to have occured and processed */
  delay(2+1);

  /* This should cause a interrupt */
  if (bitFlag) {

    /* Read the ISR to clear the interrupt */
    reason = as3935_get_interrupt_reason();

    retval = 1;

  } else {

    retval = 0;
  }

  /* Attach the normal isr */
  attachInterrupt(0, normalIsr, RISING);

  return retval;

}

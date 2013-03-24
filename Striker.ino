/**********************************************************************
 *
 * AS3935 Lighting Detector
 *
 * Synopsis:
 * The project utilizes the AS3935 lighting detector chip to monitor
 * the enviroment for a lightning storm. The sensor is connected to an
 * Arduino for command and control. The Arduino's serial output connects
 * to a second computer that logs and processes the data as per the 
 * user's desire. Possibilities include sounding an alarm, performing a
 * backup, or shutting down a computer network.
 *
 * Hardware:
 * The Embedded Adventure AS3935 lightning detector breakout board (BoB)
 * is the basis of this system. It connects to an Arduino with a few
 * simple connections. These are power and ground and the clock (SCL)
 * and the data (SDA) lines for the I2C bus as well a an interrupt line.
 * The I2C lines connect to the pins on the Arduino that support the 
 * hardware I2C peripheral. These pins on an Arduino Uno are labeled
 * A4 and A5. The interrupt line connects to the D2 pin on an Arduino
 * Uno. The serial connection to the PC is done via the USB cable. 
 *
 * Software:
 * The software configures the AS3935 device and then sits in a loop
 * waiting for an interrupt indicating some event has occured. When the
 * interrupt occurs, the ISR simply sets a flag that is then polled in
 * main loop. The reason for this is a limitation with the AS3935. 
 * Unlike most chips, when the AS3935 generates an interrupt, it is not
 * ready to tell you why! You need to give the chip 2ms for it to post 
 * its result to the ISR register. This prevents the normal behavior
 * of an interrupt service routine to read the interrupt service 
 * register and perform the needed action from there. (What an idiotic
 * design). 
 * On startup, the software also performs various autocalibration 
 * routines to insure the device is running at peak efficiency. It then
 * continues to perform these calibrations periodicaly should 
 * enviromental factors (temperature, ...) change the needed tunning
 * values.
 * Finaly, the software runs a built in self test periodicaly to insure
 * system is still functioning. It does this by generating the
 * appropriate signature using a 500kHz antenna. If the AS3935 does not
 * see this strike, then it declares a BIT failure.
 *
 * This software was written by Skye Sweeney. It is provided under the 
 * TBD license. 
 *
 *
 *********************************************************************/
#include <Wire.h>
#include "MyTypes.h"
#include "as3935.h"
#include "I2C.h"
#include "I2cMaster.h"


#define DEBUG

/* Helpful conversions */
#define SEC_TO_MS  (1000L)
#define MIN_TO_MS  (1000L*60L)
#define HR_TO_MS   (1000L*60L*60L)

/* Pins used for I2C */
#define SDA_PIN A4
#define SCL_PIN A5
//#define SDA_PIN 7
//#define SCL_PIN 6

/* Pin used for strike generator */
#define STRIKE_PIN 4

/* Delay needed from INT to reading ISR */
#define ISR_DELAY (3)

/**********************************************************************
 *
 * Global data
 *
 *********************************************************************/
volatile INT32U counter   = 0; /* ISR counter */
volatile INT8U  isrFlag   = 0; /* Normal ISR flag */
volatile INT32U bitCnt    = 0; /* BIT ISR counter */
         INT32U calTime   = 0; /* Time to do the next calibration */
         INT32U bitTime   = 0; /* Time to do the next BIT */
SoftI2cMaster si2c(SDA_PIN, SCL_PIN);  /* Bit-Bang I2C */

/**********************************************************************
 *
 * Setup routine
 *
 *********************************************************************/
void setup(void) {
  REG_u  reg;
  INT8U  err;
  INT32U now;
  INT8U  thres;
  INT8U  val;
  
  /* Open serial port */
  Serial.begin(115200);

  /* Indicate we started */
  Serial.println("Striker starting");
  
  /* In order to overcome a bug in the AS3935, one must read
   * register 0 the first time using a bit-bang approach so
   * that you can ignore the incorrect NAK the chip generates.
   * After that, we can switch to the Wire library 
   */
  delay(250) ;
  si2c.start( AS3935_ADDR| I2C_WRITE);
  si2c.write(0);
  si2c.restart(AS3935_ADDR| I2C_READ);
  val = si2c.read(1);  /* Read just one byte */
  si2c.stop();
  Serial.println(val, HEX);

  si2c.start( AS3935_ADDR| I2C_WRITE);
  si2c.write(1);
  si2c.restart(AS3935_ADDR| I2C_READ);
  val = si2c.read(1);  /* Read just one byte */
  si2c.stop();
  Serial.println(val, HEX);
  
  /* Open I2C library */
  Wire.begin();

  /* Slow the bus down */
  //TWBR = 140; 
  
  /* Attach the normal ISR */
  attachInterrupt(0, normalIsr, RISING);
  
  /* Set unit into operation mode */
  as3935_err(as3935_set_powerdown(0), "pwron");

  /* Prime the various times */
  now = millis();
  calTime = now+1*SEC_TO_MS;   /* First cal to be done in one second */
  bitTime = now+10*SEC_TO_MS;  /* First BIT to be done in 10 seconds */
  calTime = 0xffffffff;
  bitTime = 0xffffffff;
  
} /* end setup */


/**********************************************************************
 *
 * Main loop
 *
 *********************************************************************/
void loop(void) {
  InterruptReason_e reason;
  INT32U            now;
  INT32U            power;
  INT8U             bitResults;
  INT8U             calResult;
  INT8U             err;
  INT8U             dist;
  INT8U             val;
  char              c;
  REG_u             reg;
  int               i;
  

  /* If a character is recieved */  
  if (Serial.available() > 0) {
    
    /* Get the character */
    c = Serial.read();
    
    /* Help */
    if (c == '?') {
      Serial.println("Help:");
      Serial.println(" D - Dump all registers");
      Serial.println(" d - Dump prime registers");
      Serial.println(" c - Force calibration");
      Serial.println(" b - Force BIT");
      
    /* Dump all registers */  
    } else if (c == 'Dd') {
      as3935_err(as3935_dump(0x0, 0x32), "d");
      
    /* Dump top registers */  
    } else if (c == 'd') {
      as3935_err(as3935_dump(0x0, 0x32), "d");
      
    /* Force a calibration */  
    } else if (c == 'c') {
      
      calResult = calibrate();
  
      /* Process calibration results */
      if (calResult == 1) {
        Serial.println("Calibration passed");
      } else {
        Serial.println("Calibration failed");
      }      
    
    /* Force a BIT */  
    } else if (c == 'b') {
      
      /* Run BIT */
      bitResults = bitTest();
  
      /* Process BIT results */
      if (bitResults == 1) {
        Serial.println("BIT passed");
      } else {
        Serial.println("BIT failed");
      }

    } else if (c == '0') {
      as3935_err(i2c_read(AS3935_ADDR, REG00, &reg), "0");
      Serial.print("Reg: 0x");
      Serial.println(reg.data, HEX);
      
    } else if (c == '1') {
      as3935_err(i2c_read(AS3935_ADDR, REG01, &reg), "1");
      Serial.print("Reg: 0x");
      Serial.println(reg.data, HEX);
      
      
    } else if (c == 'q') {
      si2c.start( AS3935_ADDR| I2C_WRITE);
      si2c.write(0);
      si2c.restart(AS3935_ADDR| I2C_READ);
      val = si2c.read(1);  /* Read just one byte */
      si2c.stop();
      Serial.println(val, HEX);
      
      
    } else if (c == 'w') {  
      si2c.start( AS3935_ADDR| I2C_WRITE);
      si2c.write(1);
      si2c.restart(AS3935_ADDR| I2C_READ);
      val = si2c.read(1);  /* Read just one byte */
      si2c.stop();
      Serial.println(val, HEX);
      
    } else if (c == 'D') {  
      for (i=0; i<2; i++) {
        si2c.start( AS3935_ADDR| I2C_WRITE);
        si2c.write(i);
        si2c.restart(AS3935_ADDR| I2C_READ);
        val = si2c.read(1);  /* Read just one byte */
        si2c.stop();
        Serial.print(i, HEX);
        Serial.print(" = ");
        Serial.println(val, HEX);
      }
      
    } else {
      /* Ignore anything else */
    }
    

  } /* if a character is available */

  /* If the ISR flag has been set */
  if (isrFlag) {

    /* Clear the flag */    
    isrFlag = 0;

    /* Get the reason for the interrupt */    
    as3935_err(as3935_get_interrupt_reason(&reason), "get_isr");
    
    Serial.print("ISR: ");
    
    switch (reason) {

      case INT_NONE:
        Serial.println("None?");
        break;

      case INT_NOISY:
        Serial.println("Noisy");
        break;

      case INT_DISTURBER:
        Serial.println("Disturber");
        break;

      case INT_STRIKE:
        Serial.println("Strike");
        as3935_err(as3935_get_energy_calc(&power), "get-power");
        as3935_err(as3935_get_storm_distance(&dist), "get-dist");
        Serial.print("Pwr: ");
        Serial.print(power);
        Serial.print(" Dist: ");
        Serial.print(dist);
        Serial.println("");

        break;

      default:
        Serial.println("Unkown ISR");
        break;
    }

  }

  /* Get current time */
  now = millis();


  /****************************************/
  /* Perform calibration every 30 minutes */
  /****************************************/
  if (now > calTime) {

    /* Set next cal time */
    calTime = now + 30*MIN_TO_MS;
    
    /* Perform calibration */
    calResult = calibrate();

    /* Process calibration results */
    if (calResult != 1) {
      Serial.println("Calibration failed");
    }
    
    /* Dump registers for review */
    as3935_err(as3935_dump(0, 0x32), "dump");

  }

  /****************************************/
  /* Perform self test every 60 minutes   */
  /****************************************/
  if (now > bitTime) {

    /* Set next BIT time */
    bitTime = now + 60*MIN_TO_MS;

    /* Run BIT */
    bitResults = bitTest();

    /* Process BIT results */
    if (bitResults != 1) {
      Serial.println("BIT failed");
    }
    
    /* Dump registers for review */
    as3935_err(as3935_dump(0, 0x32), "dump");
    
  }
  
} /* end loop */


/**********************************************************************
 *
 * Normal ISR for the AS3935 interrupt
 *
 *********************************************************************/
void normalIsr(void) {
  /* Due to the stupidity of this chip, the event can't be handled in
   * the ISR. This is because the Interrupt Service Register is not 
   * available till 2 ms AFTER the interrupt occurs! Totaly nuts.
   * So the only thing we can do is to set a flag and use that in the
   * main loop to poll against.
   */
   
   isrFlag = 1;
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
   bitCnt ++;
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
  INT32U  bestTuneDiff = 100000;
  INT32U  diff;
  INT8U   err;
  INT16U  target;
  INT32U  cnt;
  INT8U   retval;
  

  /* Attach the calibration ISR */
  attachInterrupt(0, calIsr, RISING);
  
  /* Set the LCO output divider to 16 */
  as3935_err(as3935_set_freq_div_ratio(LCO_DIV_16), "LCO/16");
 
  /* Put the LCO onto the interrupt pin */
  as3935_err(as3935_display_responance_freq_on_irq(1), "LCO-isr");
    

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
  
    /* Set the tuning selection */
    as3935_err(as3935_set_tune_cap(i), "set cap");

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
      diff = cnt - TARGET;
    } else {
      diff = TARGET - cnt;
    }

#ifdef DEBUG
    Serial.print("Tune: ");
    Serial.print(i);
    Serial.print(" = ");
    Serial.println(diff);
#endif
    
    /* Capture the smallest error */
    if (diff < bestTuneDiff) {
      bestTuneValue  = i;
      bestTuneDiff  = diff;
    }
    
  } /* Do next tune selection */
    
#ifdef DEBUG  
  Serial.print("Best tune value: ");
  Serial.println(bestTuneValue);
#endif  

  /* Insure this error meets the 3.5% */
  if (bestTuneDiff < ERR_THRESHOLD) {
    retval = 1;
  } else {
    retval = 0;
  }

  /* Now set the tune value of the best match */
  as3935_err(as3935_set_tune_cap(bestTuneValue), "set best cap");

  /* Now tune the RCO */
  as3935_err(as3935_calibrate_rco(), "cal rco");

  /* Restore interrupt pin to normal operation */
  as3935_err(as3935_display_responance_freq_on_irq(0), "LCO-off");
  
  /* Attach the normal ISR */
  attachInterrupt(0, normalIsr, RISING);
  
  return retval;
        
} /* end calibrate */


/**********************************************************************
 *
 * Built in test
 *
 *********************************************************************/
INT8U bitTest(void) {
  INT8U             retval;
  InterruptReason_e reason;
  INT8U             err;

  /* Attach the BIT isr */
  attachInterrupt(0, bitIsr, RISING);

  /* Reset ISR counter */
  bitCnt = 0;

  /* Turn on the noise generator for n ms */
  digitalWrite(STRIKE_PIN, HIGH);
  digitalWrite(STRIKE_PIN, LOW);

  /* Wait a bit for interrupt to have occured and processed */
  delay(2+1);

  /* This should cause a interrupt */
  if (bitCnt > 0) {

    /* Read the ISR to clear the interrupt */
    as3935_err(as3935_get_interrupt_reason(&reason), "get isr");

    retval = 1;

  } else {

    retval = 0;
  }

  /* Attach the normal isr */
  attachInterrupt(0, normalIsr, RISING);

  return retval;

} /* end bitTest */


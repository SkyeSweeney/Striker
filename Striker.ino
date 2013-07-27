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
 * This project is Arduino based. It was implemented with an Arduino Pro
 * Mini 328 5V0 from Sparkfun. Most any Arduino could be used with minor
 * alterations to the wiring or code.
 * Connected to the Arduino are the following devices:
 * 1) AS3935 Lightning Detector 
 * The Embedded Adventure AS3935 lightning detector breakout board (BoB)
 * was used for this system. It takes power and ground from the Arduino.
 * Communication to the Arduino is provided by a two wire I2C bus.
 * Although the I2C lines are connected to the pins on the Arduino that
 * support the hardware I2C peripheral, the system uses bit-banging for
 * communication. This is only because the @#$#! chip has a bug in its
 * I2C interface. If the chip is strapped for device ID 0, it will not
 * allow you to read register 0 unless you ignore the NAK from the device
 * read address cycle. This is not a simple job using an unmodified Arduino
 * Wire library. The I2C pins on an Arduino Uno are labeled A4 and A5. 
 * An interrupt line is also used to allow the AS3935 to signal the Arduino
 * when it detects various events including lightning strikes. This singal
 * is connected to D2 on the Arduino that acts as Interrupt 0.
 *
 * 2) RS-232 level converter
 * The Arduino talks to a host over RS-232. Since the Pro Mini does not 
 * include an RS-232 level converter, a Sparkfun RS232 Shifter (PRT-00449)
 * is used. This is connected to the serial TX and RX lines. This serial
 * port is also used to program the Arduino using the serial bootloader.
 *
 * 3) LED
 * A simple LED is connected to the Arduino for status. It will flash for
 * each interrupt occuring.
 *
 * 4) Audio Alarm
 * An audio alarm is connected to the Arduino to provide an alarm for 
 * approaching storms. The duration of the warming sounds will increase as
 * the storm approaches.
 *
 * 5) Momentary switch
 * A momentary switch is provided to silence the audio alarm. Pressing the
 * switch will silence the alarm till the detector does not see a strike in
 * in one hour.
 *
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

/*!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 * Only one interface should be selected.
 *!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!*/
#define USE_I2C
//#define USE_SPI


#include <Wire.h>
#include "MyTypes.h"
#include "as3935.h"

#ifdef USE_I2C
#include "I2C.h"
#include "I2cMaster.h"
#endif

#ifdef USE_SPI
  #include <SPI.h>
#endif


/* Helpful conversions */
#define SEC_TO_MS  (1000L)
#define MIN_TO_MS  (1000L*60L)
#define HR_TO_MS   (1000L*60L*60L)

/* Pins used for I2C */
#ifdef USE_I2C
#define SDA_PIN A4
#define SCL_PIN A5
#endif

/* Pins for SPI */
#ifdef USE_SPI
#define SS_PIN   10
#define MOSI_PIN 11
#define MISO_PIN 12
#define SCLK_PIN 13
#endif


/* Pins used for other features */
#define STRIKE_PIN  4    /* Pin used for strike generator */
#define ALARM_PIN   5    /* Pin used for an audio alarm */
#define LED_PIN     6    /* Pin used for an LED alarm */
#define SILENCE_PIN 7    /* Pin used for an alarm silence alarm */

/* Delay needed from INT to reading ISR */
#define ISR_DELAY (3)

/* Enable these to allow functionality */
#define ENABLE_CAL
#define ENABLE_BIT

/**********************************************************************
 *
 * Global data
 *
 *********************************************************************/
volatile INT32U counter     = 0; /* ISR counter */
volatile INT8U  isrFlag     = 0; /* Normal ISR flag */
volatile INT32U bitCnt      = 0; /* BIT ISR counter */
         INT32U calTime     = 0; /* Time to do the next calibration */
         INT32U bitTime     = 0; /* Time to do the next BIT */
         INT32U ledTime     = 0; /* Time to turn off the LED */
         INT32U alarmTime   = 0; /* Time to turn off the alarm */
         INT32U silenceTime = 0; /* Time to turn off the alarm */
         bool   silence     = false;
         char   cmd[32];
         INT32U now;
         INT8U  iCmd        = 0;
       

INT16 determineDistance(INT8U val);


/**********************************************************************
 *
 * Setup routine
 *
 *********************************************************************/
void setup(void) {
  REG_u  reg;
  INT8U  err;
  INT8U  thres;
  INT8U  val;
  
  /* Open serial port */
  Serial.begin(115200);

  /* Indicate we started */
  Serial.println("Striker starting");
  
  /* Configure lightning simulator control pin */
  pinMode(STRIKE_PIN, OUTPUT);
  digitalWrite(STRIKE_PIN, LOW);
  
  /* Configure audio alarm pin */
  pinMode(ALARM_PIN, OUTPUT);
  digitalWrite(ALARM_PIN, LOW);
  
  /* Configure LED alarm pin */
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  
  /* Configure alarm silence switch input pin */
  pinMode(SILENCE_PIN, INPUT);
  digitalWrite(SILENCE_PIN, HIGH); /* enable pullup */
  
  /* Attach the normal ISR */
  attachInterrupt(0, normalIsr, RISING);

  /* Initialize the IO subsystem to talk to the AS3935 */  
  io_init();
  
  /* Set unit into operation mode ???*/

  /* Prime the various times */
  now = millis();
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
  INT32U            power;
  INT8U             bitResults;
  INT8U             calResult;
  INT8U             err;
  INT8U             dist;
  INT8U             val;
  char              c;
  REG_u             reg;
  int               i;
  INT16             km;
  INT16             dt;


  /* Get current time */
  now = millis();

  /* If a character is recieved */  
  if (Serial.available() > 0) {
    
    /* Get the character */
    c = Serial.read();
    //Serial.println(c);
    
    /* If end of string */
    if (c == '\r') {
      
      cmd[iCmd] = 0;
      parseCommand();
      iCmd = 0;
      
    } else if (c == '\n') {
      /* Ignore */
      
    } else {
      cmd[iCmd] = c;
      iCmd++;
    }
    

  } /* if a character is available */

  /* If the ISR flag has been set */
  if (isrFlag) {

    /* Clear the flag */    
    isrFlag = 0;
    
    /* Wait the time delay before the service register can be read */
    delay(ISR_DELAY);
    
    /* Turn on the LED */
    digitalWrite(LED_PIN, HIGH);
    ledTime = now + 500;

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
                
        /* Dump registers for review */
        as3935_err(as3935_dump(0, 0x9), "strike");

        as3935_err(as3935_get_energy_calc(&power), "get-power");
        as3935_err(as3935_get_storm_distance(&dist), "get-dist");
        Serial.print("Pwr: ");
        Serial.print(power);
        Serial.print(" Dist: ");
        km = determineDistance(dist);
        Serial.print(km);
        Serial.println("");
        
        startAlarm(now, km);

        /* Set time for silence experation */
        silenceTime = now + 1*60*60*1000;


        break;

      default:
        Serial.println("Unkown ISR");
        break;
    }

  }

  /****************************************/
  /* Turn off LED if needed               */
  /****************************************/
  if ((ledTime != 0) && (now > ledTime)) {
    digitalWrite(LED_PIN, LOW);
    ledTime = 0;
  }

  /****************************************/
  /* Turn off alarm if needed             */
  /****************************************/
  if ((alarmTime != 0) && (now > alarmTime)) {
    digitalWrite(ALARM_PIN, LOW);
    alarmTime = 0;
  }

  /****************************************/
  /* Reanable alarm                       */
  /****************************************/
  if ((silenceTime != 0) && (now > silenceTime)) {
    silence = false;
    silenceTime = 0xffffffff;
    Serial.println("Silence renabled");
  }
  
  /****************************************/
  /* Monitor alarm silence switch         */
  /****************************************/
  if ((!silence) && (digitalRead(SILENCE_PIN) == LOW)) {
    digitalWrite(ALARM_PIN, LOW);
    silence = true;
    Serial.println("Alarm silenced");
  }

  /****************************************/
  /* Perform calibration every 30 minutes */
  /****************************************/
  if (now > calTime) {

    /* Set next cal time */
    calTime = now + 30*MIN_TO_MS;
    
#ifdef ENABLE_CAL
    /* Perform calibration */
    calResult = calibrate();

    /* Process calibration results */
    if (calResult != 1) {
      Serial.println("Calibration failed");
    }
    
    /* Dump registers for review */
    as3935_err(as3935_dump(0, 0x9), "cal");
#endif

  }

  /****************************************/
  /* Perform self test every 60 minutes   */
  /****************************************/
  if (now > bitTime) {

    /* Set next BIT time */
    bitTime = now + 60*MIN_TO_MS;

#ifdef ENABLE_BIT
    /* Run BIT */
    bitResults = bitTest();

    /* Process BIT results */
    if (bitResults != 1) {
      Serial.println("BIT failed");
    }
    
    /* Dump registers for review */
    as3935_err(as3935_dump(0, 0x9), "dump");
#endif
    
  }
  
} /* end loop */



/**********************************************************************
 *
 * Parse an incomming command in string cmd
 *
 *********************************************************************/
void parseCommand(void) {
  char              c;
  INT8U             calResult;
  INT8U             bitResults;
  char             *p;
  REG_u             reg;
  INT8U             r;
  INT8U             val;

  /* Start parsing the string */
  p = strtok(cmd, " ");
  if (p == NULL) return;
  
  /* First character used as command */
  c = *p;

  /* Help */
  if (c == '?') {
    Serial.println("Help:");
    Serial.println(" D - Dump all registers");
    Serial.println(" d - Dump prime registers");
    Serial.println(" c - Force calibration");
    Serial.println(" b - Force BIT");
    Serial.println(" r reg - Read reg");
    Serial.println(" w reg val - Write cal to reg");
    Serial.println(" R - Reset chip");
    Serial.println(" i - Disable interrupts");
    Serial.println(" I - Enable interrupts");
    Serial.println(" g - Get gain");
    Serial.println(" G val - Set gain to val");
    Serial.println(" t - Get WDTH threshold");
    Serial.println(" T val - Set WDTH threshold");
    Serial.println(" s - Get spike rejection");
    Serial.println(" S val - Set spike rejection");
    Serial.println(" n - Get noise level");
    Serial.println(" N val - Set noise level");
    Serial.println(" q addr - Set I2C address");
    Serial.println(" Q - Get I2C address");
    
  /* Read register */  
  } else if (c == 'r') {
    p = strtok(NULL, " ");
    if (p == NULL) return;
    r = strtol(p, NULL, 16);
    as3935_err(as3935_read((RegisterID_e)r, &reg), "r");
    
    Serial.print("Reg 0x");
    Serial.print(r, HEX);
    Serial.print(" = ");
    Serial.println(reg.data, HEX);

    
  /* Write register */  
  } else if (c == 'w') {
    
    /* Read register */
    p = strtok(NULL, " ");
    if (p == NULL) return;
    r = strtol(p, NULL, 16);
    
    /* Read value */
    p = strtok(NULL, " ");
    if (p == NULL) return;
    reg.data = strtol(p, NULL, 16);
    
    as3935_err(as3935_write((RegisterID_e)r, reg), "w");
    
    Serial.print("Reg 0x");
    Serial.print(r, HEX);
    Serial.print(" <= ");
    Serial.println(reg.data, HEX);
    
    
  /* Disable interrupts */  
  } else if (c == 'i') {
    attachInterrupt(0, NULL, RISING);
    Serial.println("Interrupts disabled");


  /* Enable interrupts */  
  } else if (c == 'I') {
    attachInterrupt(0, normalIsr, RISING);
    Serial.println("Interrupts enabled");


  /* Set gain */  
  } else if (c == 'G') {
    p = strtok(NULL, " ");
    if (p == NULL) return;
    val = strtol(p, NULL, 16);
    
    as3935_err(as3935_set_afe(val), "G");
    
    Serial.print("Gain <= ");
    Serial.println(val, HEX);


  /* Get gain */  
  } else if (c == 'g') {
    
    as3935_err(as3935_get_afe(&val), "g");
    
    Serial.print("Gain = ");
    Serial.println(val, HEX);


  /* Set WDTH threshold */  
  } else if (c == 'T') {
    p = strtok(NULL, " ");
    if (p == NULL) return;
    val = strtol(p, NULL, 16);
    
    as3935_err(as3935_set_watchdog_threshold(val), "T");
    
    Serial.print("WDTH <= ");
    Serial.println(val, HEX);

  /* Get WDTH threshold */  
  } else if (c == 't') {
    
    as3935_err(as3935_get_watchdog_threshold(&val), "t");
    
    Serial.print("WDTH = ");
    Serial.println(val, HEX);


  /* Set Spike threshold */  
  } else if (c == 'S') {
    p = strtok(NULL, " ");
    if (p == NULL) return;
    val = strtol(p, NULL, 16);
    
    as3935_err(as3935_set_spike_rejection(val), "S");
    
    Serial.print("SREJ <= ");
    Serial.println(val, HEX);


  /* Get Spike threshold */  
  } else if (c == 's') {
    
    as3935_err(as3935_get_spike_rejection(&val), "s");
    
    Serial.print("SREJ = ");
    Serial.println(val, HEX);


  /* Set noise floor level */  
  } else if (c == 'N') {
    p = strtok(NULL, " ");
    if (p == NULL) return;
    val = strtol(p, NULL, 16);
    
    as3935_err(as3935_set_noise_floor_level(val), "N");
    
    Serial.print("NF_LEV <= ");
    Serial.println(val, HEX);


  /* Get noise floor level */  
  } else if (c == 'n') {
    
    as3935_err(as3935_get_noise_floor_level(&val), "n");
    
    Serial.print("NF_LEV = ");
    Serial.println(val, HEX);


#ifdef USE_I2C
  /* Set I2C address */  
  } else if (c == 'Q') {
    p = strtok(NULL, " ");
    if (p == NULL) return;
    val = strtol(p, NULL, 16);
    
    as3935_addr = val;
    
    Serial.print("I2C addr <= ");
    Serial.println(val, HEX);


  /* Get I2C address */  
  } else if (c == 'q') {
    Serial.print("I2C addr = ");
    Serial.println(as3935_addr, HEX);
#endif        
    
  /* Dump all registers */  
  } else if (c == 'D') {
    as3935_err(as3935_dump(0x0, 0x33), "D");
    
  /* Dump top registers */  
  } else if (c == 'd') {
    as3935_err(as3935_dump(0x0, 0x9), "d");
    
  /* Force a Reset */  
  } else if (c == 'R') {
    as3935_err(as3935_reset_registers(), "R");
    Serial.println("Chip reset");
    
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
    
  /* Force an long alarm */  
  } else if (c == 'A') {
    startAlarm(now, 1);
    Serial.println("Long alarm");
    
  /* Force an short alarm */  
  } else if (c == 'a') {
    startAlarm(now, 40);
    Serial.println("Short alarm");
    
  /* Reenable alarm */  
  } else if (c == 'S') {
    silence = false;
    Serial.println("Alarm activated");
    
  /* Silence alarm */  
  } else if (c == 's') {
    silence = true;
    Serial.println("Alarm silenced");
    
  } else {
    /* Ignore anything else */
    Serial.println("Unknown command");
  }
}

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
  REG_u   reg;
  

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

    Serial.print("Tune: ");
    Serial.print(i);
    Serial.print(" = ");
    Serial.println(diff);
    
    /* Capture the smallest error */
    if (diff < bestTuneDiff) {
      bestTuneValue  = i;
      bestTuneDiff  = diff;
    }
    
  } /* Do next tune selection */
    
  Serial.print("Best tune value: ");
  Serial.println(bestTuneValue);

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
  
  /* Dump registers for review */
  as3935_err(as3935_dump(0, 0x9), "cal");

  return retval;

} /* end bitTest */


/**********************************************************************
 *
 * Converted coded value to distance in km
 * Equation is the best fit from the datasheet
 *
 *********************************************************************/
INT16 determineDistance(INT8U val) {
  INT16 retval;
  if (val == 0x3f) {
    retval = 100;
  } else {
    retval = val;
  }
  return retval;
}

/**********************************************************************
 *
 *
 *********************************************************************/
void startAlarm(INT32U now, INT16 km) {
  INT16 dt;
  
  /* Turn on audio alarm */
  /* The equation gives us a 1/2 second alarm at 40km and 2 seconds overhead */
  if (!silence) {
    digitalWrite(ALARM_PIN, HIGH);
    dt = (2000 - km*38);
    if (dt < 500) dt = 500;
    if (dt > 2000) dt = 2000;
    alarmTime = now + (INT32U)dt;
  }
}

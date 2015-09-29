Striker
=======
AS3935 Lightning Detector Arduino code
This project is to maintain an Arduino application to utilize the AMS AS3935 chip to detect lightning in the 
user's vicinity. The Embedded Artists AS3935 break out board (BoB) is used to ease integration with this small
SMT chip and associated analog antenna circuitry. Thhe interface to the Bob is via I2C using device ID 0. 
Due to a bug on the revision of the AS3935, the Arduino 'Wire' library could not be used. A bing-bang approach using
the TBD library was used. The bug prevents ready access to register 0 as the chip NAKs the read address cycle.
The application uses a library for the I2C interface as well as one for the AS3935 specific functions. The code
provides periodinc tuning of the antenna circuit as well as built-in-test is an auxiliary spark generator is 
connected to the system. 

The principle output of the Arduino is a serial data stream that contains strings that can be parsed by a host PC to 
provide the functionality needed. It also provides a LED and horn alarm with a silence button.

# ABSTRACT

Military strength plays a key role in present scenario for establishing and protecting the sovereignty of a country. Hence, there is a huge competition as well as necessity to all the countries across the globe to achieve continuous advancement in missile and nuclear weapon technology. Therefore, through various strategies every nation desires to be a front runner in its military operations to exhibit their power. Electronic warfare is one modern technology which provides high-end strength to the defence power of nation.

Radar technology is past, present & future in electronic warfare. This technology is being integrated in every warplane and ground bases for their protection against enemy attacks. If a warplane or ground base is attacked through a missile or any tanker or other intrusions, then radar will detect the intrusion and set the data immediately to the base for further commands.

In this project the Radar is intelligent and tracks the object along with detecting it. 
This system detects any object at any time irrespective of visibility (it also works during nights).










# CONTENTS


## Overview of the project
1. **Hardware Description**
2. **Arduino**
3. **Servo motor**
4. **Ultrasonic sensors**
5. **Interfacing**
6. **Program code**
7. **Result and Conclusion**
8. **Reference**
















## OVERVIEW OF THE PROJECT

*INTRODUCTION:*
     Embedded system consists of application specified hardware and application software optimized for compact size and efficient power and performance.
 Applications of Embedded systems in Warfare techniques have caused a tremendous change in defence and space fields.
The embedded system developed for the object detection and tracking can be used as a model for missile tracking and firing.

*1.2 AIM:*
 OBJECT DETECTION & TRACKING USING ULTRASONIC SENSORS WITH ARDUINO is mainly used on ground and can also be used in the aircrafts. Considering the ground crew we developed this for just 180 degrees of range. This mainly consists of 3 parts,
Object detection.
Object tracking.

*1.3 METHODOLGIES:*
 This can be obtained in various ways.
Using ultrasonic sensors.
Using IR distance measurement sensors.
Using camera module and image processing.
Using other proximity sensors.

*1.4 WORKING:*
When the system is turned on, it continuously scans the specified range of area and transmits the data to controller through serial connection. When any object comes in the range of 20 cm then the system tracks the object and also indicates it using a laser (representing firing if the missile).  
  

## HARDWARE DESCRIPTION

There are mainly 3 basic parts used as hardware in this project.
Arduino uno.
Mini servo motor.
Ultrasonic sensors.

In further lessons there is a detailed discussion on above hardware parts.
There are also other basic (common) hardware used for the integration such as connectors, breadboard, jumper wires, etc.,












## 3. AURDUINO       
**3.1. Overview**
Arduino Uno is a microcontroller board based on 8-bit ATmega328P microcontroller. Along with ATmega328P, it consists other components such as crystal oscillator, serial communication, voltage regulator, etc. to support the microcontroller. Arduino Uno has 14 digital input/output pins (out of which 6 can be used as PWM outputs), 6 analog input pins, a USB connection, A Power barrel jack, an ICSP header and a reset button.

**3.2. Board description:**
What's on the board?
There are many varieties of Arduino boards that can be used for different purposes. Some boards look a bit different from the one below, but most Arduinos have the majority of these components in common
Power (USB / Barrel Jack)
Every Arduino board needs a way to be connected to a power source. The Arduino UNO can be powered from a USB cable coming from your computer or a wall power supply (like this) that is terminated in a barrel jack. In the picture above the USB connection is labeled (1) and the barrel jack is labeled (2).

The USB connection is also how you will load code onto your Arduino board. More on how to program with Arduino can be found in our Installing and Programming Arduino tutorial.

NOTE: Do NOT use a power supply greater than 20 Volts as you will overpower (and thereby destroy) your Arduino. The recommended voltage for most Arduino models is between 6 and 12 Volts.

Pins (5V, 3.3V, GND, Analog, Digital, PWM, AREF)
The pins on your Arduino are the places where you connect wires to construct a circuit (probably in conjuction with a breadboard and some wire. They usually have black plastic ‘headers’ that allow you to just plug a wire right into the board. The Arduino has several different kinds of pins, each of which is labeled on the board and used for different functions.

GND (3): Short for ‘Ground’. There are several GND pins on the Arduino, any of which can be used to ground your circuit.


5V (4) & 3.3V (5): As you might guess, the 5V pin supplies 5 volts of power, and the 3.3V pin supplies 3.3 volts of power. Most of the simple components used with the Arduino run happily off of 5 or 3.3 volts.


Analog (6): The area of pins under the ‘Analog In’ label (A0 through A5 on the UNO) are Analog In pins. These pins can read the signal from an analog sensor (like a temperature sensor) and convert it into a digital value that we can read.


Digital (7): Across from the analog pins are the digital pins (0 through 13 on the UNO). These pins can be used for both digital input (like telling if a button is pushed) and digital output (like powering an LED).


PWM (8): You may have noticed the tilde (~) next to some of the digital pins (3, 5, 6, 9, 10, and 11 on the UNO). These pins act as normal digital pins, but can also be used for something called Pulse-Width Modulation (PWM). We have a tutorial on PWM, but for now, think of these pins as being able to simulate analog output (like fading an LED in and out).


AREF (9): Stands for Analog Reference. Most of the time you can leave this pin alone. It is sometimes used to set an external reference voltage (between 0 and 5 Volts) as the upper limit for the analog input pins.


Reset Button:
Just like the original Nintendo, the Arduino has a reset button (10). Pushing it will temporarily connect the reset pin to ground and restart any code that is loaded on the Arduino. This can be very useful if your code doesn’t repeat, but you want to test it multiple times. Unlike the original Nintendo however, blowing on the Arduino doesn't usually fix any problems.

Power LED Indicator:
Just beneath and to the right of the word “UNO” on your circuit board, there’s a tiny LED next to the word ‘ON’ (11). This LED should light up whenever you plug your Arduino into a power source. If this light doesn’t turn on, there’s a good chance something is wrong. Time to re-check your circuit!

TX RX LEDs:
TX is short for transmit, RX is short for receive. These markings appear quite a bit in electronics to indicate the pins responsible for serial communication. In our case, there are two places on the Arduino UNO where TX and RX appear -- once by digital pins 0 and 1, and a second time next to the TX and RX indicator LEDs (12). These LEDs will give us some nice visual indications whenever our Arduino is receiving or transmitting data.

Main IC:
The black thing with all the metal legs is an IC, or Integrated Circuit (13). Think of it as the brains of our Arduino. The main IC on the Arduino is slightly different from board type to board type, but is usually from the ATmega line of IC’s from the ATMEL company. This can be important, as you may need to know the IC type (along with your board type) before loading up a new program from the Arduino software. This information can usually be found in writing on the top side of the IC. If you want to know more about the difference between various IC's, reading the datasheets is often a good idea.

Voltage Regulator:
The voltage regulator (14) is not actually something you can (or should) interact with on the Arduino. But it is potentially useful to know that it is there and what it’s for. The voltage regulator does exactly what it says -- it controls the amount of voltage that is let into the Arduino board. Think of it as a kind of gatekeeper; it will turn away an extra voltage that might harm the circuit. Of course, it has its limits, so don’t hook up your Arduino to anything greater than 20 volts.

3.3 The Arduino Family:
Arduino makes several different boards, each with different capabilities. In addition, part of being open source hardware means that others can modify and produce derivatives of Arduino boards that provide even more form factors and functionality. If you’re not sure which one is right for your project, check this guide for some helpful hints. Here are a few options that are well-suited to someone new to the world of Arduino
Arduino Uno (R3):
The Uno is a great choice for your first Arduino. It's got everything you need to get started, and nothing you don't. It has 14 digital input/output pins (of which 6 can be used as PWM outputs), 6 analog inputs, a USB connection, a power jack, a reset button and more. It contains everything needed to support the microcontroller; simply connect it to a computer with a USB cable or power it with a AC-to-DC adapter or battery to get started.
3.3. Pin description:


Power:
Vin, 3.3V, 5V, GND

Vin: Input voltage to Arduino when using an external power source.

5V: Regulated power supply used to power microcontroller and other components on the board

3.3V: 3.3V supply generated by on-board voltage regulator. Maximum current draw is 50mA.


GND:
ground pins.

Reset:
Resets the microcontroller

Analog Pins:
A0 – A5
Used to provide analog input in the range of
0-5V

Input/Output Pins:
Digital Pins 0 - 13
Can be used as input or output pins.

Serial:
0(Rx), 1(Tx)
Used to receive and transmit TTL serial data.

External Interrupts:
2, 3.
To trigger an interrupt.

PWM:
3, 5, 6, 9, 11
Provides 8-bit PWM output.

SPI:
10 (SS), 11 (MOSI), 12 (MISO) and 13 (SCK)
Used for SPI communication.

Inbuilt LED:
Pin number is 13.
To turn on the inbuilt LED.

TWI:
A4 (SDA), A5 (SCA)
Used for TWI communication.

AREF:
It is AREF pin.
To provide reference voltage for input voltage.

 ARDUINO UNO TECHNICAL SPECIFICATIONS:

 Microcontroller:  ATmega328P – 8 bit AVR family microcontroller

**ATmega328p:**
ATmega328p 28 pin narrow dual inline package

Atmega8 32 thin quad flat pack:

The ATmega328 is a single-chip microcontroller created by Atmel in the megaAVR family (later Microchip Technology acquired Atmel in 2016). It has a modified Harvard architecture 8-bit RISC processor core. 
Specifications:
The Atmel 8-bit AVR RISC-based microcontroller combines 32 kB ISP flash memory with read-while-write capabilities, 1 kB EEPROM, 2 kB SRAM, 23 general purpose I/O lines, 32 general purpose working registers, three flexible timer/counters with compare modes, internal and external interrupts, serial programmable USART, a byte-oriented 2-wire serial interface, SPI serial port, 6-channel 10-bit A/D converter (8-channels in TQFP and QFN/MLF packages), programmable watchdog timer with internal oscillator, and five software selectable power saving modes. The device operates between 1.8-5.5 volts. The device achieves throughput approaching 1 MIPS per MHz.

Key parameters of Atmega328:

CPU type							:	8-bit AVR
Performance						:	20 MIPS at 20 MHz
Flash memory						:	32 kB
SRAM								:	2 kB
EEPROM							:	1 kB
Pin count							:	28 or 32 pin 
Maximum operating frequency	:	20 MHz
Number of touch channels		:	16
Hardware QTouchAcquisition	:	No
Maximum I/O pins				:	23
External interrupts					:	2
USB Interface						:	No

 Working :


The basic working of CPU of ATmega328:-


1. The data is uploaded in serial via the port (being uploaded from the computer’s Arduino IDE). The data is decoded and then the instructions are sent to instruction register and it decodes the instructions on the same clock pulse.


2. On the next clock pulse the next set of instructions are loaded in instruction register.


3. In general purpose registers the registers are of 8-bit but there are 3 16-bit registers also.
a. 8-bit registers are used to store data for normal calculations and results.
b. 16-bit registers are used to store data of timer counter in 2 different register. Eg. X-low & X-high. They are fast, and are used to store specific hardware functions.


4. EEPROM stores data permanently even if the power is cut out. Programming inside a EEPROM is slow.


5. Interrupt Unit checks whether there is an interrupt for the execution of instruction to be executed in ISR (Interrupt Service Routine).


6. Serial Peripheral Interface (SPI) is an interface bus commonly used to send data between microcontrollers and small peripherals such as Camera, Display, SD cards, etc. It uses separate clock and data lines, along with a select line to choose the device you wish to talk to.


7. Watchdog timer is used to detect and recover from MCU malfunctioning.


8. Analog comparator compares the input values on the positive and negative pin, when the value of positive pin is higher the output is set.


9. Status and control is used to control the flow of execution of commands by checking other blocks inside the CPU at regular intervals.


10. ALU (Arithmetic and Logical unit)
The high performance AVR ALU operates in direct connection with all the 32 general purpose working registers. Within a single clock cycle, arithmetic operations b/w general purpose registers are executed. The ALU operations are divided into 3 main categories – arithmetic, logical and bit-function.


I/O pins 
The digital inputs and outputs (digital I/O) on the Arduino are what allow you to connect the Arduino sensors, actuators, and other ICs. Learning how to use them will allow you to use the Arduino to do some really useful things, such as reading switch inputs, lighting indicators, and controlling relay outputs.

Series alternatives:
common alternative to the ATmega328 is the "picoPower" ATmega328P. A comprehensive list of all other members of the megaAVR series can be found on the Atmel website.[3]

.  ATmega328
.  ATmega328P and ATmega328P-AUTOMOTIVE
.  ATmega328PB and ATmega328PB-AUTOMOTIVE 
  (superset of ATmega328P)


**3.4.3. Applications :**
As of 2013 the ATmega328 is commonly used in many projects and autonomous systems where a simple, low-powered, low-cost micro-controller is needed. Perhaps the most common implementation of this chip is on the popular Arduino development platform, namely the Arduino Uno and Arduino Nano models. 
**3.3.2**
Operating Voltage					:	5V
Recommended Input Voltage		:	7-12V
Input Voltage Limits				:	6-20V
Analog Input Pins					:	6 (A0 – A5)
Digital I/O Pins					:	14 (Out of which 6 provide PWM output)
DC Current on I/O Pins			:	40 mA
DC Current on 3.3V Pin			:	50 mA
Flash Memory						:	32 KB (0.5 KB is used for Bootloader)
SRAM								:	2 KB
EEPROM							:	1 KB
Frequency (Clock Speed)			:	16 MHz

Other Arduino Boards
Arduino Nano, Arduino Pro Mini, Arduino Mega, Arduino Due, Arduino Leonardo

 Working:
How to use Arduino Board?

The 14 digital input/output pins can be used as input or output pins by using pinMode(), digitalRead() and digitalWrite() functions in arduino programming. Each pin operate at 5V and can provide or receive a maximum of 40mA current,




and has an internal pull-up resistor of 20-50 KOhms which are disconnected by default.  Out of these 14 pins, some pins have specific functions as listed below:

Serial Pins 0 (Rx) and 1 (Tx): Rx and Tx pins are used to receive and transmit TTL serial data. They are connected with the corresponding ATmega328P USB to TTL serial chip.
External Interrupt Pins 2 and 3: These pins can be configured to trigger an interrupt on a low value, a rising or falling edge, or a change in value.
PWM Pins 3, 5, 6, 9 and 11: These pins provide an 8-bit PWM output by using analogWrite() function.
SPI Pins 10 (SS), 11 (MOSI), 12 (MISO) and 13 (SCK): These pins are used for SPI communication.
In-built LED Pin 13: This pin is connected with an built-in LED, when pin 13 is HIGH – LED is on and when pin 13 is LOW, its off.
Along with 14 Digital pins, there are 6 analog input pins, each of which provide 10 bits of resolution, i.e. 1024 different values. They measure from 0 to 5 volts but this limit can be increased by using AREF pin with analog Reference() function.  

Analog pin 4 (SDA) and pin 5 (SCA) also used for TWI communication using Wire library.
Arduino Uno has a couple of other pins as explained below:

AREF: Used to provide reference voltage for analog inputs with analogReference() function.

Reset Pin: Making this pin LOW, resets the microcontroller.

**Communication:**
Arduino can be used to communicate with a computer, another Arduino board or other microcontrollers. The ATmega328P microcontroller provides UART TTL (5V) serial communication which can be done using digital pin 0 (Rx) and digital pin 1 (Tx). An ATmega16U2 on the board channels this serial communication over USB and appears as a virtual com port to software on the computer. The ATmega16U2 firmware uses the standard USB COM drivers, and no external driver is needed. However, on Windows, a .inf file is required. The Arduino software includes a serial monitor which allows simple textual data to be sent to and from the Arduino board. There are two RX and TX LEDs on the arduino board which will flash when data is being transmitted via the USB-to-serial chip and USB connection to the computer (not for serial communication on pins 0 and 1). A SoftwareSerial library allows for serial communication on any of the Uno's digital pins. The ATmega328P also supports I2C (TWI) and SPI communication. The Arduino software includes a Wire library to simplify use of the I2C bus.


Arduino Uno to ATmega328 Pin Mapping
When ATmega328 chip is used in place of Arduino Uno, or vice versa, the image below shows the pin mapping between the two.

Arduino Uno ATmega328P Pin Mapping

## Software:
Arduino IDE (Integrated Development Environment) is required to program the Arduino Uno board. Download it here.

## Programming Arduino:
Arduino programs are written in the Arduino Integrated Development Environment (IDE). Arduino IDE is a special software running on your system that allows you to write sketches (synonym for program in Arduino language) for different Arduino boards. The Arduino programming language is based on a very simple hardware programming language called processing, which is similar to the C language. After the sketch is written in the Arduino IDE, it should be uploaded on the Arduino board for execution.

The first step in programming the Arduino board is downloading and installing the Arduino IDE. The open source Arduino IDE runs on Windows, Mac OS X, and Linux. Download the Arduino software (depending on your OS) from the official website and follow the instructions to install.

Now let’s discuss the basics of Arduino programming.

The structure of Arduino program is pretty simple. Arduino programs have a minimum of 2 blocks,

Preparation & Execution:
Each block has a set of statements enclosed in curly braces:

void setup( )
{
statements-1;
.
.
.
statement-n;
}

void loop ( )
{
statement-1;
.
.
.
statement-n;
}

Here, setup ( ) is the preparation block and loop ( ) is an execution block.
The setup function is the first to execute when the program is executed, and this function is called only once. The setup function is used to initialize the pin modes and start serial communication. This function has to be included even if there are no statements to execute.

void setup ( )
{
pinMode (pin-number, OUTPUT); // set the ‘pin-number’ as output
pinMode (pin-number, INPUT); // set the ‘pin-number’ as output	
}
After the setup ( ) function is executed, the execution block runs next. The execution block hosts statements like reading inputs, triggering outputs, checking conditions etc.

In the above example loop ( ) function is a part of execution block. As the name suggests, the loop( ) function executes the set of statements (enclosed in curly braces) repeatedly.

Void loop ( )
{
digitalWrite (pin-number,HIGH); // turns ON the component connected to ‘pin-number’
delay (1000); // wait for 1 sec
digitalWrite (pin-number, LOW); // turns OFF the component connected to ‘pin-number’
delay (1000); //wait for 1sec
}

Note: Arduino always measures the time duration in millisecond. Therefore, whenever you mention the delay, keep it in milli seconds.

Applications:

Prototyping of Electronics Products and Systems
Multiple DIY Projects.
Easy to use for beginner level DIYers and makers.
Projects requiring Multiple I/O interfaces and communications.






**SERVO MOTOR**
A servo motor is a closed-loop system that uses position feedback to control its motion and final position. In industrial type servo motors the position feedback sensor is usually a high precision encoder, while in the smaller RC or hobby servos the position sensor is usually a simple potentiometer.




There are many types of servo motors and their main feature is the ability to precisely control the position of their shaft. A servo motor is a closed-loop system that uses position feedback to control its motion and final position.

 In industrial type servo motors the position feedback sensor is usually a high precision encoder, while in the smaller RC or hobby servos the position sensor is usually a simple potentiometer. The actual position captured by these devices is fed back to the error detector where it is compared to the target position. Then according to the error the controller corrects the actual position of the motor to match with the target position.


A servo motor is controlled by sending a series of pulses through the signal line. The frequency of the control signal should be 50Hz or a pulse should occur every 20ms. The width of pulse determines angular position of the servo and these type of servos can usually rotate 180 degrees 
Generally pulses with 1ms duration correspond to 0 degrees position, 1.5ms duration to 90 degrees and 2ms to 180 degrees. Though the minimum and maximum duration of the pulses can sometimes vary with different brands and they can be 0.5ms for 0 degrees and 2.5ms for 180 degrees position.


**4.1. Arduino Servo Motors Control**

Let’s put the above said to test and make a practical example of controlling a hobby servo using Arduino. I will use the MG996R which is a high-torque servo featuring metal gearing with stall torque of 10 kg-cm. The high torque comes at a price and that’s the stall current of the servo which is 2.5A. The running current is from 500mA to 900mA and the operating voltage is from 4.8 to 7.2V.



The current ratings indicate that we cannot directly connect this servo to the Arduino, but we must use a separate power supply for it. Here’s the circuit schematic for this example.

We simply need to connect the control pin of the servo to any digital pin of the Arduino board, connect the Ground and the positive wires to the external 5V power supply, and also connect the Arduino ground to the servo ground.


**4.2 Arduino Servo Motor Control Code:**

Now let’s take a look at the Arduino code for controlling the servo motor

#include <Servo.h>
Servo myservo; // create servo object to control a servo
void setup() 
{	myservo.attach(9,600,2300); // (pin, min, max)	}
void loop() 
{
myservo.write(0); // tell servo to go to a particular angle
delay(1000);
myservo.write(90); 
delay(500); 
myservo.write(135); 
delay(500)
myservo.write(180); 
delay(1500); 
}

Here we just need to include the library, define the servo object, and using the attach() function define the pin to which the servo is connected as well as define the minimum and maximum values of the pulses durations. Then using the write() function we simply set the position of the servo from 0 to 180 degrees. With this library we can drive up to 12 servos at the same time or 48 servos using Arduino Mega board.








## ULTRASONIC SENSOR
Ultrasonic detection sensor is most commonly used in industrial applications to detect hidden tracks, discontinuities in metals, composites, plastics, ceramics, and for water level detection. For this purpose the laws of physics which are indicating the propagation of sound waves through solid materials have been used since ultrasonic sensors using sound instead of light for detection.
What is the principle of Ultrasonic Detection?
**5.1 Defining sound wave**
Sound is a mechanical wave travelling through the mediums, which may be a solid, or liquid or gas. Sound waves can travel through the mediums with specific velocity depends on the medium of propagation. The sound waves which are having high frequency reflect from boundaries and produces distinctive echo patterns.
**5.2 Laws of physics for sound waves**
Sound waves are having specific frequencies or number of oscillations per second. Humans can detect sounds in a frequency range from about 20Hz to 20 KHz. However the frequency range normally employed in ultrasonic detection is 100 KHz to 50MHz. The velocity of ultrasound at a particular time and temperature is constant in a medium.
W = C/F (or) W = CT
Where 
W		=	Wave length
C		=	Velocity of sound in a medium
F		=	Frequency of wave
T		=	Time Period
The most common methods of ultrasonic examination utilize either longitudinal waves or shear waves. The longitudinal wave is a compression wave in which the particle motion is in the same direction of the propagation wave. The shear wave is a wave motion in which the particle motion is perpendicular to the direction of propagation. Ultrasonic detection introduces high frequency sound waves into a test object to obtain information about the object without altering or damaging it in any way. Two values are measured in ultrasonic detection.
The amount of time, taking for the sound to travel through the medium and amplitude of the received signal. Based on velocity and time thickness can be calculated.
Thickness of material = Material sound velocity X Time of Fight
Transducers for Wave Propagation and particle detection
For sending sound waves and receiving echo, ultrasonic sensors, normally called transceivers or transducers will be used. They work on a principle similar to radar that will convert electrical energy into mechanical energy in the form of sound, and vice versa.
5.3 Pin diagram:
It consists of 4 pins 
Vcc (positive power supply)
Trig (for transmitter)
Echo (for receiver)
Gnd (for negative power supply)

The commonly used transducers are contact transducers, angle beam transducers, delay line transducers, immersion transducers, and dual element transducers. Contact transducers are typically used for locating voids and cracks to the outside surface of a part as well as measuring thickness. Angle beam transducers use the principle of reflection and mode conversion to produce refracted shear or longitudinal waves in the test material.
Delay line transducers are single element longitudinal wave transducers used in conjunction with a replaceable delay line. One of the reasons for choosing delay line transducer is that near surface resolution can be improved. The delay allows the element to stop vibrating before a return signal from the reflector can be received.
The major advantages offered by immersion transducers over contact transducers are Uniform coupling reduces sensitivity variations, Reduction in scan time, and increases sensitivity to small reflectors.
 
**5.4. Operation of ultrasonic sensors:**
When an electrical pulse of high voltage is applied to the ultrasonic transducer it vibrates across a specific spectrum of frequencies and generates a burst of sound waves. Whenever any obstacle comes ahead of the ultrasonic sensor the sound waves will reflect back in the form of echo and generates an electric pulse. It calculates the time taken between sending sound waves and receiving echo. The echo patterns will be compared with the patterns of sound waves to determine detected signal’s condition.
 Three applications involving Ultrasonic detection:
The distance of obstacle or discontinuities in metals is related to velocity of sound waves in a medium through which waves are passed and the time taken for echo reception. Hence the ultrasonic detection can be used for finding the distances between particles, for detecting the discontinuities in metals and for indicating the liquid level.
Ultrasonic Distance Measurement
Ultrasonic sensors are used for distance measuring applications. These gadgets regularly transmit a short burst of ultrasonic sound to a target, which reflects the sound back to the sensor. The system then measures the time for the echo to return to the sensor and computes the distance to the target using the speed of sound within the medium.
Different sorts of transducers are utilized within industrially accessible ultrasonic cleaning devices. An ultrasonic transducer is affixed to a stainless steel pan which is filled with a solvent and a square wave is applied to it, conferring vibration energy on the liquid.
Ultrasonic Distance Sensor
The ultrasonic distance sensors measures distance using sonar; an ultrasonic (well above human hearing) beat is transmitted from the unit and distance-to-target is determined by measuring the time required for the echo return. Output from the ultrasonic sensor is a variable-width beat that compares to the distance to the target.

 EightFeatures of Ultrasonic Distance Sensor:
Supply voltage			:	5V (DC).
Supply current			:	15mA.
Modulation frequency	:	40Hz.
Output					:	0 – 5V (Output high when obstacle detected in 
range).
Beam Angle				:	Max 15 degree.
Distance				:	2 cm – 400 cm.
Accuracy				:	0.3 cm.
Communication		:	Positive TTL pulse.

Operation of Ultrasonic distance Sensor:
Ultrasonic sensor module comprises of one transmitter and one receiver. The transmitter can deliver 40 KHz ultrasonic sound while the maximum receiver is designed to accept only 40 KHz sound waves. The receiver ultrasonic sensor that is kept next to the transmitter shall thus be able to receive reflected 40 KHz, once the module faces any obstacle in front. Thus whenever any obstacles come ahead of the ultrasonic module it calculates the time taken from sending the signals to receiving them since  time and distance are related for sound waves passing through air medium at 343.2m/sec. Upon receiving the signal MC program while executed displays the data i.e. the distance measured on a LCD interfaced to the microcontroller in centimeters.
 
 
 
## Ultrasonic Distance Sensor Circuit for testing (ping test).

Characteristically, robotics applications are very popular but you’ll also find this product to be useful in security systems or as an infrared replacement if so desired.
**5.7. Ultrasonic Obstacle Detection**
Ultrasonic sensors are used to detect the presence of targets and to measure the distance to targets in many robotized processing plants and process plants. Sensors with an ON or OFF digital output are available for detecting the presence of objects and sensors with an analog output which changes relatively to the sensor to target separation distance are commercially available.
Ultrasonic obstacle sensor consists of a set of ultrasonic receiver and transmitter which operate at the same frequency. The point when the something moves in the zone secured the circuit’s fine offset is aggravated and the buzzer/alarm is triggered.
**5.8. Features:**
Power consumption of 20mA
Pulse in/out communication
Narrow acceptance angle
Provides exact, non-contact separation estimations within 2cm to 3m
The explosion point LED shows estimations in advancement
3-pin header makes it simple to connect utilizing a servo development link
 
 **5.9. Specifications:**
Power supply			:	5V DC
Quiescent current		:	<15mA
Effectual angle			:	 <15°
Ranging distance		:	 2cm – 350 cm
Resolution				:	 0.3 cm
Output cycle			:	 50ms

 The sensor detects objects by emitting a short ultrasonic burst and then listening for the eco. Under control of a host microcontroller, the sensor emits a short 40 KHz explosion. This explosion ventures or travels through the air, hits an article and after that bounces once again to the sensor.
The sensor provides an output pulse to the host that will terminate when the echo is detected; hence the width of one pulse to the next is taken into calculation by a program to provide result in distance of the object.
Note: ultrasonic sensor fails in the following cases:
This is the main disadvantage using an ultrasonic sensor.

## INTERFACING
All the connections are made with Arduino and a chassis for the system is designed such that the ultrasonic sensors cover the range of a specified angle (120 degrees).
 Connections are as follows:
All the Vcc pins or positive pins are connected to 5volts pin of Arduino board, Using a breadboard.
All the ground or negative pins are connected to ground pin of Arduino board, using a breadboard.
Trigger pins of three ultrasonic sensors are connected to 5,8,10 digital pins of Arduino.
Echo pins of three ultrasonic sensors are connected to 4,7,11 digital pins of Arduino. 
Servo control pin is connected to digital pin 9 of Arduino.  

Now Arduino is connected to the PC using connecting cable provided and code is dumped into it along with supply of power to it.
After the code is dumped, there is no need for the pc to be connected. Arduino can be operated using the 9v battery using connector. 

## TESTING
The assembled embedded system has been tested over different cases (range: 20cm, 30cm, 40cm...etc max range is 1m)
The ranges of the ultrasonic sensor has been modified over the course of time and the stability of the system has been checked 
It has been tested with different shapes of objects
Observation:
From above testing we found out that its works efficiently with flatter objects as the sensor needs to transmit and receive the waves through the object and also its efficacy is high at 30 degrees rather than a whole 90 degrees. 

## RESULT
An embedded system using arduino board is designed for detection and tracking of an object. The required hardware is assembled and tested completely and found satisfactory.
The design is found to be perfect interface with Arduino and the required output is clearly seen to be working.
The further advancement for this will be searching and tracking in a 3 dimensional space instead of just a 2d space. Other advancement is to build a robot that can follow a human or an object continuously.













## References 

https://www.elprocus.com/ultrasonic-detection-basics-application/

https://youtu.be/tw5raQmmzQA

https://youtu.be/kQRYIH2HwfY

https://www.elprocus.com/laser-diode-construction-working-applications/

https://youtu.be/KJXTUGZBYIw

https://youtu.be/Hq9dn5gWrsU

https://marcosmiller.wixsite.com/myhomelab/project-01

https://youtu.be/jYiEWkRVALc

https://youtu.be/Uo-pGq0lP6M

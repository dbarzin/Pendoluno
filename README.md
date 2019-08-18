# Penduluno

Modeling of a Damped Triple Pendulum on an Arduino UNO

The idea comes from a video of the coding train [Coding Challenge #93: Double Pendulum](https://www.youtube.com/watch?v=uWzPe_S-RVE&t=1s)

Then I imagine to make it run with three Pendulum on an Arduiono UNO.

![screen](https://raw.githubusercontent.com/dbarzin/Pëndoluno/master/images/screen.jpg)

Make it run on an Arduino with only 32k bytes of Flash was a little chalange !

##How it works

It works with a 2.8'' TFP LCD Schield

![device](https://raw.githubusercontent.com/dbarzin/Pëndoluno/master/images/device.jpg)

## Reference

The math is based on a paper from Nick Eyre & Jeff Holzgrafe [Modeling of a Damped Triple Pendulum](https://www.nickeyre.com/images/triplependulum.pdf)

## How to install it  

**1** Install [Arduino](https://www.arduino.cc/en/Main/Software)  
**3** Download this project and open it with Arduino
**4** Maybe customize the code depending on the LCD screen you have :
```
#define LCD_CS A3              // Chip Select goes to Analog 3
#define LCD_CD A2              // Command/Data goes to Analog 2
#define LCD_WR A1              // LCD Write goes to Analog 1
#define LCD_RD A0              // LCD Read goes to Analog 0

#define LCD_RESET A4           // Can alternately just connect to Arduino's reset pin

#include <MCUFRIEND_kbv.h>
MCUFRIEND_kbv tft;

#define SCREEN_WIDTH  320      // Screen Size
#define SCREEN_HEIGHT 240      

```
**5** Upload the code to your Arduino UNO (don't forget to set it to the right upload settings!)

**Done**


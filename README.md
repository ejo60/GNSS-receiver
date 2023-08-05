# GNSS-receiver
GNSS receiver based on Adafruit ultimate v3 breakout board, it includes a 1.3 inch display and a BMP085 
pressure temperature sensor. The OLED display will show:

line 1: Banner line
line 2: UTC date and time
line 3: latitude and longitude
line 4: Maidenhead locator, Temperature and Pressure
line 5: Number of satellites, Antenna indicator, Speed, Heading, Altitude 

Lines 3 to 5 are only updated when there is a position fix
This GPS receiver from Adafruit has a real time clock, so if the time
and date were updated, and if there was an initialization of the RTC 
then date and time are shown when there is no reception of the satellite 
signal.

For the rest there are two 18650 batteries in the enclosure, a charge 
controller with a max current of 0.25Amp and a max voltage of 6V, and
a DC-DC converter which turns the battery voltage into 6V going to 
Vin of the Arduino Nano board.

Never tested this, but I think it can run for a couple of days in this 
way, 5000 mAh capacity and some 170mA, should be 1 or 2 days it can run
in this way.

Charging should be done with a current limited (250 mA) max 6V charger.

Several libraries are used, credits can be found in the source file. The
code was tested on a Arduino Nano Every. 

Some images

IMG_4924 : on the breadboard
IMG_4926 : be careful with the OLED display, pins may be different and 
           the display should be isolated from the enclosure. It is a
           fragile component that may, or may not come with a
           voltage regulator.
IMG_4928 : The way it should look
IMG_4931 : The way it fits in the enclosure

The battery management controller for 18650 cells, any one may do.

The DC-DC converter is a Otronic MT3608 2A Max DC-DC step up converter

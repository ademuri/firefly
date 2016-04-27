# firefly
LED lights that sync with each other via radio, and the music via a microphone

Uses a CC1101 to communicate with other Firefly nodes. Uses the electret microphone/preamp circuit from 
[Damian Peckett](http://dpeckett.com/beat-detection-on-the-arduino).

Uses a light-modified version of Panstamp cc1101 library. I updated the default configuration for GDO2 on the cc1101
to assert when a packet is available and unread. Then, receiveData checks whether GDO2 is asserted before trying to
read from the RX FIFO. Without this, frequent calls to receiveData will prevent the cc1101 from receiving data (!),
as it appears to not pick up packets when communicating over SPI.

LED driver circuits inspired by [this](http://www.tbideas.com/blog/build-an-arduino-shield-to-drive-high-power-rgb-led/).
I'm using "3W" RGB LEDs from [Adafruit](https://www.adafruit.com/products/2530), and providing them each with 330mA.

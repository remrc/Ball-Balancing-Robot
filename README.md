# Ball-Balancing-Robot

ESP32, Nidec 24H motors, MPU6050, 3S 2200 mAh LiPo battery.

The controller is the same one I used in the self-balancing cube. I use encoders, so the red connections in the schematic are important.

<img src="/pictures/foto.jpg" alt="Ball balancing robot"/>

First connect to controller over bluetooth. You will see a message that you need to calibrate the balancing point. Send c+ from serial monitor. This activate calibrating procedure. Set the robot to balancing point. Hold still when the robot does not fall to either side. Send c- from serial monitor. This will write the offsets to the EEPROM and begin to balance.

More about this:

https://youtu.be/P4Ni7JJW_-U

Source code coming soon...

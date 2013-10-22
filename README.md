PiLCD
=====

PiLCD example code

cd /home/pi/wiringPi/examples
make /home/pi/PiLCD/lcd_ip2.c
This will create the lcd_ip2


# cd /home/pi/piLCD
# cp LCDinit.sh /etc/init.d/.
# chmod +x /etc/init.d/LCDinit.sh
# update-rc.d LCDinit.sh defaults

to remove
# update-rc.d -f LCDinit.sh remove

to test without installing
# /etc/init.d/LCDinit.sh start  

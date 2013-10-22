#! /bin/sh

### BEGIN INIT INFO
# Provides:          LCDinit
# Required-Start:    
# Required-Stop:     
# Default-Start:     2 3 4 5
# Default-Stop:	     0 1 6 
# Short-Description: PiLCD for WebMSO28 startup
# Description:       Gamepad to Mindstorm NXT 
#                    Bluetooth bridge
### END INIT INFO
#. /lib/lsb/init-functions

#/root/run_js.sh

#set -e

case "$1" in
	start|force-reload|restart|reload)
		/home/pi/PiLCD/lcd_ip2
	;;
  	stop|status)
	;;
	*)
	echo "Usage: $N {start|stop|restart|force-reload|status}" >&2
	exit 1
	;;
esac

exit 0

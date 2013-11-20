#!/bin/bash

### BEGIN INIT INFO
# Provides:          spawn-fcgi
# Required-Start:    $all      
# Required-Stop:     $all      
# Default-Start:     2 3 4 5   
# Default-Stop:      0 1 6     
# Short-Description: starts FastCGI for PHP
# Description:       starts FastCGI for PHP using start-stop-daemon
### END INIT INFO

USER=root
GROUP=root
CHILDREN=6
HOST=127.0.0.1
PORT=9000

PATH=/usr/local/sbin:/usr/local/bin:/sbin:/bin:/usr/sbin:/usr/bin
NAME=spawn-fcgi
PID=/var/run/spawn-fcgi.pid
DAEMON=/usr/bin/spawn-fcgi
DAEMON_OPTS="-d /var/www/fcgi-bin -f mso28ctl.fcgi -a $HOST -p $PORT -C $CHILDREN"

test -x $DAEMON || exit 0

set -e

case "$1" in
  start)
        echo "Starting $NAME: "
        start-stop-daemon --start --pidfile $PID --exec $DAEMON -- $DAEMON_OPTS
        echo "done."
        ;;
  stop)
        echo "Stopping $NAME: "
        start-stop-daemon --stop  --pidfile $PID --retry 5
        rm -f $PID
        echo "done."
        ;;
  restart)
        echo "Stopping $NAME: "
        start-stop-daemon --stop  --pidfile $PID --retry 5
        rm -f $PID
        echo "done..."
        sleep 1
        echo "Starting $NAME: "
        start-stop-daemon --start --pidfile $PID --exec $DAEMON -- $DAEMON_OPTS
        echo "done."
        ;;
  *)
        echo "Usage: /etc/init.d/$NAME {start|stop|restart}" >&2
        exit 1
        ;;
esac

exit 0

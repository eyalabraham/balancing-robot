#!/usr/bin/python
###############################################################################
# 
# remote.py
#
#   <tbd>.
#
#   Resources:
#       <list>
#
#   January 12, 2019
#
###############################################################################

import time

###############################################################################
#
# main()
#
def main():
    """<tbd>"""

###############################################################################
#
# send_cmd()
#
def send_cmd(cmd_string, time_out = 1):
    """
    Send a command string to the controller and block until it returns with prompt or times out.
    Return 0 if prompt ok, -1 if time-out is reached before prompt.
    """

    print 'cmd: {}'.format(cmd_string)
    send_bytes = bytearray()
    send_bytes.extend((cmd_string + '\r'))
    self.__ser.write(send_bytes)

    # Wait for prompt or time-out
    start = time.time()

    while True:
        pr = self.__ser.read(3)

        if pr.strip('\r\n\t ') == '>':
            return 0

        end = time.time()
        if ( end - start > time_out ):
            break

    return -1

###############################################################################
#
# Startup
#
if __name__ == '__main__':
    main()


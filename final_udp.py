import socket
import curses
import udp_struct
from ctypes import *
UDP_IP = "192.168.1.1"  # match microcontroller ip
UDP_PORT = 2363  # must match port microcontroller listening

# Internet           #UDP
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
msg = udp_struct.message()
msg.velocity = 0
msg.theta = 0
msg.rst = 0
screen = curses.initscr()
curses.noecho()  # makes it so key presses are not displayed on terminal screen
curses.cbreak()  # terminal responds to input right away rather than waiting for enter
screen.keypad(1)  # enable keypad mode
key = {
    curses.KEY_LEFT: "left",
    curses.KEY_RIGHT: "right",
    curses.KEY_UP: "up",
    curses.KEY_DOWN: "down"}
wasd = {
    119: 0,
    97:  270,
    115: 180,
    100: 90}
velocity = {curses.KEY_UP: 0.1, curses.KEY_DOWN: -0.1}
theta = {curses.KEY_LEFT: 15, curses.KEY_RIGHT: -15}


def pack(ctype_instance):
	buf = string_at(byref(ctype_instance), sizeof(ctype_instance))
	cstring = create_string_buffer(buf)
	return cstring

try:
    while True:
	keypress=screen.getch()
        if(keypress in key.keys()):
            msg.reset=0
            p=key[keypress]
            screen.addstr("Keypress registered: " + p + " arrow key\n")
            if(keypress in velocity.keys()):
                msg.velocity += velocity[keypress]
		screen.addstr("Velocity in msg changed to " + str(msg.velocity) + "mm/s\n")
		msg_send=pack(msg)
		sock.sendto(msg_send, (UDP_IP, UDP_PORT))
            elif(keypress in theta.keys()):
                msg.theta+= theta[keypress]
		msg_send=pack(msg)
            	sock.sendto(msg_send, (UDP_IP, UDP_PORT))
	elif(curses.keyname(keypress) == " "):
            screen.addstr("Spacebar(a.k.a. reset) pressed\n")
            msg.rst=1
            msg.velocity=0
	    msg_send=pack(msg)
            sock.sendto(msg_send, (UDP_IP, UDP_PORT))
	elif(keypress in wasd.keys()):
            msg.rst=0
            msg.theta+= theta[keypress]
	    msg_send=pack(msg)
            sock.sendto(msg_send, (UDP_IP, UDP_PORT))
finally:
    curses.nocbreak()
    screen.keypad(0)
    curses.echo()
    curses.endwin()

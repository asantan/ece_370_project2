import socket
import curses
import udp_struct
UDP_IP = "192.168.1.192"  # match microcontroller ip
UDP_PORT = 2363  # must match port microcontroller listening

# Internet           #UDP
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#sock.bind((UDP_IP, UDP_PORT))
msg = udp_struct.message()
msg.velocity = 0
msg.theta = 0
msg.reset = 0
screen = curses.initscr()
curses.noecho()  # makes it so key presses are not displayed on terminal screen
curses.cbreak()  # terminal responds to input right away rather than waiting for enter
screen.keypad(1)  # enable keypad mode
key = {
    curses.KEY_LEFT: "left",
    curses.KEY_RIGHT: "right",
    curses.KEY_UP: "up",
    curses.KEY_DOWN: "down"}
keyval = {"left": 1, "right": -1, "up": 1, "down": -1, "reset": 1}
velocity = {curses.KEY_UP: 0.1, curses.KEY_DOWN: -0.1}
theta = {curses.KEY_LEFT: 15, curses.KEY_RIGHT: -15}

try:
    while True:
        keypress = screen.getch()
        if(keypress in key.keys()):
            msg.reset = 0
            p = key[keypress]
            screen.addstr("Keypress registered: " + p + " arrow key\n")
            if(p in velocity.keys()):
                msg.velocity += velocity[p]
            elif(p in theta.keys):
                msg.velocity += theta[p]
            sock.sendto(msg, (UDP_IP, UDP_PORT))
        elif(curses.keyname(keypress) == " "):
            screen.addstr("Spacebar(a.k.a. reset) pressed\n")
            msg.reset = 1
            msg.velocity = 0
            sock.sendto(msg, (UDP_IP, UDP_PORT))

finally:
    curses.nocbreak()
    screen.keypad(0)
    curses.echo()
    curses.endwin()

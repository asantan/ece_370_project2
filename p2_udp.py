import socket
import curses
import time

UDP_IP = "127.0.0.01"
UDP_PORT = 2363
MESSAGE = "Hello, World!"

print "UDP target IP: " + UDP_IP
print "UDP target port: " + str(UDP_PORT)
print "message: " + MESSAGE

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.sendto(MESSAGE, (UDP_IP, UDP_PORT))

screen = curses.initscr()
screen.refresh()
screen.nodelay(1)

while True:
    try:
        arrowPress = screen.getch()
        if arrowPress == curses.KEY_LEFT:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.sendto(curses.KEY_LEFT, (UDP_IP, UDP_PORT))
        if arrowPress == curses.KEY_RIGHT:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.sendto(curses.KEY_RIGHT, (UDP_IP, UDP_PORT))
        if arrowPress == curses.KEY_UP:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.sendto(curses.KEY_LEFT, (UDP_IP, UDP_PORT))
        if arrowPress == curses.KEY_DOWN:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.sendto(curses.DOWN, (UDP_IP, UDP_PORT))
    except KeyboardInterrupt:
        curses.echo()
        curses.nocbreak()
        curses.endwin()



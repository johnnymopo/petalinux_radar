
import sys
import socket
import select

try:
    IP = sys.argv[1]
    dat = sys.argv[2]
except:
    print 'wrong!'
    sys.exit(1)

PORT = 10001
#IP = '192.168.1.100'

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.bind(('', PORT))

s.sendto(dat, (IP, PORT))


s.close()


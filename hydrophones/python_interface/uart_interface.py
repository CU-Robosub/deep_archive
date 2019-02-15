import os, sys, getopt
import signal
import time
import threading
from maxim_serial import maxim_serial


class deathnote:
    write_name = False
    def __init__(self):
        signal.signal(signal.SIGINT, self.ryuk)
        signal.signal(signal.SIGTERM, self.ryuk)

    def ryuk(self, signum, frame):
        self.write_name = True




#def read_from_port(ser):


if __name__ == "__main__":
    #kill command
    yagami = deathnote()

    #TODO make com port an argument
    try:
        opts, args = getopt.getopt(sys.argv[1:], "b:c:")
    except getopt.GetoptError:
        print " -b <baud rate> -c <comm port>"
    for opt, arg in opts:
        if opt == '-b':
            baudrate = arg
            print("baud rate=" + arg)
        elif opt == '-c':
            comport = arg
    try:
        baudrate
    except NameError:
        print "set to 4800"
        baudrate = 4800
    try:
        comport
    except NameError:
        print "add comport -c <comport>"
        sys.exit()
    else:
        print "comport=" + comport

    #ser = serial.Serial(port=comport, baudrate=baudrate, timeout=1)

    maxim = maxim_serial(comport,baudrate)
    maxim.adc_write_reg()


    #test.adc_config("A", True)\

    while(1):
        if yagami.write_name: #process death note
            break
        while (maxim.inWaiting()>0): #check for serial data
            Rx_data = maxim.read();
            #TODO good enough for now but want better formatting
            print hex(ord(Rx_data)),
        print

        maxim.write('a')
        time.sleep(.1)




    print "Hey, all it did was go around in a full circle."


#print "hello world"
#print os.name
#time.sleep(5)

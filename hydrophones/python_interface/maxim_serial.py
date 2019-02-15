
import binascii
from serial import Serial

class maxim_serial(Serial):
    def __init__(self, comport, baudrate):
        Serial.__init__(self, port=comport, baudrate=baudrate, timeout=1)

        self.CONFIG_REG_ADCa = 0; #first 8 bits
        self.CONFIG_REG_ADCb = 0; #second 8 bits

        self.CONFIG_REGa = 0; #first 8 bits
        self.CONFIG_REGb = 0; #second 8 bits

    def write_adc_reg(self):
        package = bytearray()
        package.append(0x81) #FSM header
        package.append(0x0C<<2) #A address register
        #append 16 bit register valeus
        package.append(self.CONFIG_REG_ADCa)
        package.append(self.CONFIG_REG_ADCb)
        #print ''.join('/{:02x}'.format(x) for x in package)
        self.write(package)
        package[1] = 0x0D<<2 #B address
        self.write(package)
        #print ''.join('/{:02x}'.format(x) for x in package)
        package[1] = 0x0E<<2 # C address
        self.write(package)
        package[1] = 0x0F<<2 #D aadddress
        self.write(package)

    #def write_config_reg(self):

    def adc_set_clock(self, divide=6):
        clock_dict = {2:0,3:1,4:2,6:3}
        if divide not in clock_dict:
            print "not valid clock division"
        else:
            self.CONFIG_REG_ADCa &= ~(3<<5)
            self.CONFIG_REG_ADCa |= (clock_dict[divide]<<5)

    def adc_power_down(self, power):
        self.CONFIG_REG_ADCa &= ~(1<<4)
        self.CONFIG_REG_ADCa |= (int(power==True)<<4)

    def adc_24_bit(self, bit_24):
        self.CONFIG_REG_ADCb &= ~(1<<5)
        self.CONFIG_REG_ADCb |= (int(bit_24==True)<<5)

    def adc_automatic_results(self, auto):
        self.CONFIG_REG_ADCb &= ~(15<<1)
        self.CONFIG_REG_ADCb |= (15*int(auto==True)<<1)

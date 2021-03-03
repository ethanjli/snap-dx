import platform
import serial
import serial.tools.list_ports
import time
import numpy as np

from control._def import *

# add user to the dialout group to avoid the need to use sudo

class Microcontroller():
    def __init__(self,parent=None):
        self.serial = None
        self.platform_name = platform.system()
        self.tx_buffer_length = MicrocontrollerDef.CMD_LENGTH
        self.rx_buffer_length = MCU.MSG_LENGTH

        # AUTO-DETECT the Arduino! By Deepak
        arduino_ports = [
                p.device
                for p in serial.tools.list_ports.comports()
                if 'Arduino' in p.description]
        if not arduino_ports:
            raise IOError("No Arduino found")
        if len(arduino_ports) > 1:
            print('Multiple Arduinos found - using the first')
        else:
            print('Using Arduino found at : {}'.format(arduino_ports[0]))

        # establish serial communication
        self.serial = serial.Serial(arduino_ports[0],2000000)
        time.sleep(0.2)
        print('Serial Connection Open')

    def close(self):
        self.serial.close()

    def move_z(self,delta,v,a,ustepping):
        direction = int((np.sign(delta)+1)/2)
        delta_abs = abs(delta*Motion.STEPS_PER_MM_Z)
        if delta_abs > 65535:
            delta_abs = 65535
        cmd = bytearray(self.tx_buffer_length)
        cmd[0] = 2
        cmd[1] = direction
        cmd[2] = int(delta_abs) >> 8
        cmd[3] = int(delta_abs) & 0xff
        cmd[4] = ustepping
        cmd[5] = int(v) >> 8
        cmd[6] = int(v) & 0xff
        cmd[7] = int(a) >> 8
        cmd[8] = int(a) & 0xff
        self.serial.write(cmd)
        time.sleep(WaitTime.BASE + WaitTime.Z*abs(delta))

    def set_heater1_power(self,power):
        cmd = bytearray(self.tx_buffer_length)
        cmd[0] = 0
        cmd[1] = int(255*power)
        self.serial.write(cmd)
        print('update heater 1 power to ' + str(power))

    def set_heater2_power(self,power):
        cmd = bytearray(self.tx_buffer_length)
        cmd[0] = 1
        cmd[1] = int(255*power)
        self.serial.write(cmd)
        print('update heater 2 power to ' + str(power))
        # print(cmd[1])

    def send_command(self,command):
        cmd = bytearray(self.tx_buffer_length)
        self.serial.write(cmd)

    def read_received_packet(self):
        # wait to receive data
        while self.serial.in_waiting==0:
            pass
        while self.serial.in_waiting % self.rx_buffer_length != 0:
            pass

        num_bytes_in_rx_buffer = self.serial.in_waiting

        # get rid of old data
        if num_bytes_in_rx_buffer > self.rx_buffer_length:
            print('getting rid of old data')
            for i in range(num_bytes_in_rx_buffer-self.rx_buffer_length):
                self.serial.read()
        
        # read the buffer
        data=[]
        for i in range(self.rx_buffer_length):
            data.append(ord(self.serial.read()))
        return data

    def read_received_packet_nowait(self):
        # wait to receive data
        if self.serial.in_waiting==0:
            return None
        if self.serial.in_waiting % self.rx_buffer_length != 0:
            return None
        
        # get rid of old data
        num_bytes_in_rx_buffer = self.serial.in_waiting
        if num_bytes_in_rx_buffer > self.rx_buffer_length:
            print('getting rid of old data')
            for i in range(num_bytes_in_rx_buffer-self.rx_buffer_length):
                self.serial.read()
        
        # read the buffer
        data=[]
        for i in range(self.rx_buffer_length):
            data.append(ord(self.serial.read()))
        return data

class Microcontroller_Simulation():
    def __init__(self,parent=None):
        pass

    def close(self):
        pass

    def move_z(self,delta,v,a,ustepping):
        pass


    def send_command(self,command):
        pass

    def read_received_packet(self):
        pass

    def set_heater1_power(self,power):
        pass

    def set_heater2_power(self,power):
        pass       

    def read_received_packet_nowait(self):
        return None

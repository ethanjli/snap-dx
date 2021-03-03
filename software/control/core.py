# set QT_API environment variable
import os 
os.environ["QT_API"] = "pyqt5"
import qtpy

# qt libraries
from qtpy.QtCore import *
from qtpy.QtWidgets import *
from qtpy.QtGui import *

import control.utils as utils
from control._def import *

from queue import Queue
from threading import Thread, Lock
import time
import numpy as np
import pyqtgraph as pg
from datetime import datetime
from pathlib import Path

class NavigationController(QObject):

    zPos = Signal(float)

    signal_plot1 = Signal(np.ndarray,np.ndarray)
    signal_plot2 = Signal(np.ndarray,np.ndarray)
    signal_plot3 = Signal(np.ndarray,np.ndarray)

    signal_ch1 = Signal(str)
    signal_ch2 = Signal(str)
    signal_ch3 = Signal(str)

    log_message = Signal(str)
    signal_new_log_item = Signal()

    stopwatch_timeleft = Signal(float)

    def __init__(self,microcontroller):
        QObject.__init__(self)
        self.microcontroller = microcontroller
        self.z_pos = 0
        self.timer_read_pos = QTimer()
        self.timer_read_pos.setInterval(PosUpdate.INTERVAL_MS)
        self.timer_read_pos.timeout.connect(self.update_pos)
        self.timer_read_pos.start()
        self.heater_1_power = 0
        self.heater_2_power = 0

        self.file = open(str(Path.home()) + "/Downloads/" + datetime.now().strftime('%Y-%m-%d %H-%M-%-S.%f') + ".csv", "w+")
        self.ch1 = 0
        self.ch2 = 0
        self.ch3 = 0
        self.time = 0
        self.ch1_array = np.array([])
        self.ch2_array = np.array([])
        self.ch3_array = np.array([])
        self.time_array = np.array([])
        self.temp1_array = np.array([])
        self.temp2_array = np.array([])
        self.timer_update_waveform = QTimer()
        self.timer_update_waveform.setInterval(MCU.DATA_INTERVAL_ms/2)
        self.timer_update_waveform.timeout.connect(self.update_waveforms)
        self.timer_update_waveform.start()

        self.first_run = True
        self.time_ticks_start = 0

        self.time_now = 0
        self.time_diff = 0
        self.time_prev = time.time()

        self.counter_display = 0
        self.counter_file_flush = 0

        self.logging_is_on = True
        # self.log_message.emit(utils.timestamp() + 'Temperature Logging Started')

        self.countdown_timer_1s = QTimer()
        self.time_left_seconds = 0
        self.countdown_seconds_total = 0

    def countdown_timer_1s_timeout(self):
        self.time_left_seconds -= 1
        if self.time_left_seconds <= 0:
            self.countdown_timer_1s.stop()
            self.log_message.emit(utils.timestamp() + 'Countdown of ' + str(self.countdown_seconds_total) + ' stopped')
            self.signal_new_log_item.emit()
            QApplication.processEvents()
        self.stopwatch_timeleft.emit(self.time_left_seconds)
        QApplication.processEvents()

    def start_countdown_timer(self,seconds):
        self.time_left_seconds = seconds
        self.countdown_timer_1s.timeout.connect(self.countdown_timer_1s_timeout)
        self.countdown_timer_1s.start(1000)

    def logging_onoff(self,state,experimentID):
        self.logging_is_on = state
        if state == False:
            self.file.close()
            self.log_message.emit(utils.timestamp() + 'Temperature Logging Stopped')
            self.signal_new_log_item.emit()
            QApplication.processEvents()
        else:
            self.experimentID = experimentID
            self.file = open(str(Path.home()) + "/Downloads/" + self.experimentID + '_' + datetime.now().strftime('%Y-%m-%d %H-%M-%-S.%f') + ".csv", "w+")
            self.log_message.emit(utils.timestamp() + 'Temperature Logging Started')
            self.signal_new_log_item.emit()
            QApplication.processEvents()

    def move_z(self,delta_mm,v,a,ustepping):
        delta_usteps = delta_mm*100*ustepping
        velocity = int((v/VELOCITY_MAX)*65535)
        acceleration = int((a/ACCELERATION_MAX)*65535)
        self.microcontroller.move_z(delta_usteps,velocity,acceleration,ustepping)
        self.z_pos = self.z_pos + delta_mm
        print('Z: ' + str(self.z_pos))
        self.zPos.emit(self.z_pos)
        self.log_message.emit(utils.timestamp() + 'Move actuator by ' + str(delta_mm) + ' mm '
            'at v_max = ' + str(v) + ' mm/s and a = ' + str(a) + ' mm/s/s, with ustepping set to ' +
            str(ustepping) )
        self.signal_new_log_item.emit()
        QApplication.processEvents()

    def update_pos(self):
        pass
        '''
        pos = self.microcontroller.read_received_packet_nowait()
        if pos is None:
            return
        self.x_pos = utils.unsigned_to_signed(pos[0:3],MicrocontrollerDef.N_BYTES_POS)/Motion.STEPS_PER_MM_XY # @@@TODO@@@: move to microcontroller?
        self.y_pos = utils.unsigned_to_signed(pos[3:6],MicrocontrollerDef.N_BYTES_POS)/Motion.STEPS_PER_MM_XY # @@@TODO@@@: move to microcontroller?
        self.z_pos = utils.unsigned_to_signed(pos[6:9],MicrocontrollerDef.N_BYTES_POS)/Motion.STEPS_PER_MM_Z  # @@@TODO@@@: move to microcontroller?
        self.xPos.emit(self.x_pos)
        self.yPos.emit(self.y_pos)
        self.zPos.emit(self.z_pos*1000)
        '''

    def home(self):
        self.microcontroller.move_x(-self.x_pos)
        self.microcontroller.move_y(-self.y_pos)

    def set_heater1_power(self,power):
        self.heater_1_power = power
        self.microcontroller.set_heater1_power(power)
        self.log_message.emit(utils.timestamp() + 'Set heater 1 power to ' + str(power) + '/1.0')
        self.signal_new_log_item.emit()
        QApplication.processEvents()

    def set_heater2_power(self,power):
        self.heater_2_power = power
        self.microcontroller.set_heater2_power(power)
        self.log_message.emit(utils.timestamp() + 'Set heater 2 power to ' + str(power) + '/1.0')
        self.signal_new_log_item.emit()
        QApplication.processEvents()

    def close(self):
        self.file.close()

    def update_waveforms(self):
        # self.time = self.time + (1/1000)*WAVEFORMS.UPDATE_INTERVAL_MS
      
        readout = self.microcontroller.read_received_packet_nowait()
        if readout is not None:

            # self.time_now = time.time()
            # self.time_diff = self.time_now - self.time_prev
            # self.time_prev = self.time_now
            # self.time += self.time_diff
            self.time_now = time.time()

            t_chunck = np.array([])
            ch1_chunck = np.array([])
            ch2_chunck = np.array([])
            ch3_chunck = np.array([])
            temp1_chunck = np.array([])
            temp2_chunck = np.array([])

            for i in range(MCU.TIMEPOINT_PER_UPDATE):
                # time
                self.time_ticks = int.from_bytes(readout[i*MCU.RECORD_LENGTH_BYTE:i*MCU.RECORD_LENGTH_BYTE+4], byteorder='big', signed=False)
                if self.first_run:
                    self.time_ticks_start = self.time_ticks
                    self.first_run = False
                self.time = (self.time_ticks - self.time_ticks_start)*MCU.TIMER_PERIOD_ms/1000
                # self.ch1 = utils.unsigned_to_signed(readout[i*MCU.RECORD_LENGTH_BYTE+4:i*MCU.RECORD_LENGTH_BYTE+6],2)/(65536/2)
                # self.ch2 = utils.unsigned_to_signed(readout[i*MCU.RECORD_LENGTH_BYTE+6:i*MCU.RECORD_LENGTH_BYTE+8],2)/(65536/2)
                self.ch1 = utils.unsigned_to_unsigned(readout[i*MCU.RECORD_LENGTH_BYTE+4:i*MCU.RECORD_LENGTH_BYTE+6],2)
                self.ch2 = utils.unsigned_to_unsigned(readout[i*MCU.RECORD_LENGTH_BYTE+6:i*MCU.RECORD_LENGTH_BYTE+8],2)
                self.ch3 = utils.unsigned_to_unsigned(readout[i*MCU.RECORD_LENGTH_BYTE+8:i*MCU.RECORD_LENGTH_BYTE+10],2)
                self.temp1 = utils.DACs_to_temp(self.ch1,self.ch2,1977)
                self.temp2 = utils.DACs_to_temp(self.ch1,self.ch3,1980)

                record_from_MCU = (
                    str(self.time_ticks) + '\t' + str(self.ch1) + '\t' + "{:.2f}".format(self.ch2) + '\t' + "{:.2f}".format(self.temp1) + '\t' + "{:.2f}".format(self.temp2) )
                record_settings = (str(self.time_now) + '\t' + str(self.heater_1_power) + '\t' + str(self.heater_2_power))
               
                # saved variables
                if self.logging_is_on:
                    self.file.write(record_from_MCU + '\t' + record_settings + '\n')

                # append variables for plotting
                t_chunck = np.append(t_chunck,self.time)
                ch1_chunck = np.append(ch1_chunck,self.ch1)
                ch2_chunck = np.append(ch2_chunck,self.ch2)
                ch3_chunck = np.append(ch3_chunck,self.ch3)
                temp1_chunck = np.append(temp1_chunck,self.temp1)
                temp2_chunck = np.append(temp2_chunck,self.temp2)

            self.ch1_array = np.append(self.ch1_array,ch1_chunck)
            self.ch2_array = np.append(self.ch2_array,ch2_chunck)
            self.ch3_array = np.append(self.ch3_array,ch3_chunck)
            self.time_array = np.append(self.time_array,t_chunck)
            self.temp1_array = np.append(self.temp1_array,temp1_chunck)
            self.temp2_array = np.append(self.temp2_array,temp2_chunck)

            # reduce display refresh rate
            self.counter_display = self.counter_display + 1
            if self.counter_display>=1:
                self.counter_display = 0
                # self.signal_plot1.emit(self.t_chunck,self.ch1_chunck)
                # self.signal_plot2.emit(self.t_chunck,self.ch1_chunck)

                # self.signal_plot1.emit(self.time_array,self.ch1_array)
                # self.signal_plot2.emit(self.time_array,self.ch2_array)
                # self.signal_plot3.emit(self.time_array,self.ch3_array)

                # self.signal_ch1.emit("{:.2f}".format(self.ch1))
                # self.signal_ch2.emit("{:.2f}".format(self.ch2))
                # self.signal_ch3.emit("{:.2f}".format(self.ch3))

                self.signal_plot1.emit(self.time_array[-1000:],self.temp1_array[-1000:])
                self.signal_plot2.emit(self.time_array[-1000:],self.temp2_array[-1000:])
                self.signal_ch1.emit("{:.2f}".format(self.temp1))
                self.signal_ch2.emit("{:.2f}".format(self.temp2))

        # file flushing
        if self.logging_is_on:
            self.counter_file_flush = self.counter_file_flush + 1
            if self.counter_file_flush>=500:
                self.counter_file_flush = 0
                self.file.flush()



class Logger(QObject):

    def __init__(self,filepath = os.path.join(str(Path.home()),"Documents","SnapDx dev tool logs.txt")):
        QObject.__init__(self)
        self.file = open(filepath,'a')

    def log(self,log_message):
        self.file.write(log_message + '\n')

    def __del__(self):
        self.file.close()
        
    def close(self):
        self.file.close()


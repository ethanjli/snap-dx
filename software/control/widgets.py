# set QT_API environment variable
import os 
os.environ["QT_API"] = "pyqt5"
import qtpy

# qt libraries
from qtpy.QtCore import *
from qtpy.QtWidgets import *
from qtpy.QtGui import *
import pyqtgraph as pg

from control._def import *

class NavigationWidget(QFrame):

    def __init__(self, navigationController, main=None, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.navigationController = navigationController
        self.add_components()
        self.setFrameStyle(QFrame.Panel | QFrame.Raised)

    def add_components(self):
        self.label_Xpos = QLabel()
        self.label_Xpos.setNum(0)
        self.label_Xpos.setFrameStyle(QFrame.Panel | QFrame.Sunken)
        self.entry_dX = QDoubleSpinBox()
        self.entry_dX.setMinimum(0) 
        self.entry_dX.setMaximum(80) 
        self.entry_dX.setSingleStep(1)
        self.entry_dX.setValue(0)
        self.entry_vX = QDoubleSpinBox()
        self.entry_vX.setMinimum(0) 
        self.entry_vX.setMaximum(100) 
        self.entry_vX.setSingleStep(1)
        self.entry_vX.setValue(5)
        self.entry_aX = QDoubleSpinBox()
        self.entry_aX.setMinimum(0) 
        self.entry_aX.setMaximum(500) 
        self.entry_aX.setSingleStep(1)
        self.entry_aX.setValue(20)
        self.combo_ustepX = QComboBox()
        self.combo_ustepX.addItem('1')
        self.combo_ustepX.addItem('2')
        self.combo_ustepX.addItem('4')
        self.combo_ustepX.addItem('8')
        self.combo_ustepX.addItem('16')
        self.combo_ustepX.addItem('32')
        self.combo_ustepX.addItem('64')
        self.btn_moveX_forward = QPushButton('Forward')
        self.btn_moveX_forward.setDefault(False)
        self.btn_moveX_backward = QPushButton('Backward')
        self.btn_moveX_backward.setDefault(False)
        self.btn_cycleX = QPushButton('Cycle')
        self.btn_cycleX.setDefault(False)
        
        self.label_Ypos = QLabel()
        self.label_Ypos.setNum(0)
        self.label_Ypos.setFrameStyle(QFrame.Panel | QFrame.Sunken)
        self.entry_dY = QDoubleSpinBox()
        self.entry_dY.setMinimum(0)
        self.entry_dY.setMaximum(80)
        self.entry_dY.setSingleStep(1)
        self.entry_dY.setValue(0)
        self.entry_vY = QDoubleSpinBox()
        self.entry_vY.setMinimum(0) 
        self.entry_vY.setMaximum(100) 
        self.entry_vY.setSingleStep(1)
        self.entry_vY.setValue(5)
        self.entry_aY = QDoubleSpinBox()
        self.entry_aY.setMinimum(0) 
        self.entry_aY.setMaximum(500) 
        self.entry_aY.setSingleStep(1)
        self.entry_aY.setValue(20)
        self.combo_ustepY = QComboBox()
        self.combo_ustepY.addItem('1')
        self.combo_ustepY.addItem('2')
        self.combo_ustepY.addItem('4')
        self.combo_ustepY.addItem('8')
        self.combo_ustepY.addItem('16')
        self.combo_ustepY.addItem('32')
        self.combo_ustepY.addItem('64')
        self.btn_moveY_forward = QPushButton('Forward')
        self.btn_moveY_forward.setDefault(False)
        self.btn_moveY_backward = QPushButton('Backward')
        self.btn_moveY_backward.setDefault(False)

        self.label_Zpos = QLabel()
        self.label_Zpos.setNum(0)
        self.label_Zpos.setFrameStyle(QFrame.Panel | QFrame.Sunken)
        self.entry_dZ = QDoubleSpinBox()
        self.entry_dZ.setMinimum(0) 
        self.entry_dZ.setMaximum(150) 
        self.entry_dZ.setSingleStep(1)
        self.entry_dZ.setValue(0)
        self.entry_vZ = QDoubleSpinBox()
        self.entry_vZ.setMinimum(0) 
        self.entry_vZ.setMaximum(100) 
        self.entry_vZ.setSingleStep(1)
        self.entry_vZ.setValue(5)
        self.entry_aZ = QDoubleSpinBox()
        self.entry_aZ.setMinimum(0) 
        self.entry_aZ.setMaximum(500) 
        self.entry_aZ.setSingleStep(1)
        self.entry_aZ.setValue(20)
        self.combo_ustepZ = QComboBox()
        self.combo_ustepZ.addItem('1')
        self.combo_ustepZ.addItem('2')
        self.combo_ustepZ.addItem('4')
        self.combo_ustepZ.addItem('8')
        self.combo_ustepZ.addItem('16')
        self.combo_ustepZ.addItem('32')
        self.combo_ustepZ.addItem('64')
        self.btn_moveZ_forward = QPushButton('Forward')
        self.btn_moveZ_forward.setDefault(False)
        self.btn_moveZ_backward = QPushButton('Backward')
        self.btn_moveZ_backward.setDefault(False)
        self.btn_cycleZ = QPushButton('Cycle')
        self.btn_cycleZ.setDefault(False)

        self.entry_heater1 = QDoubleSpinBox()
        self.entry_heater1.setMinimum(0) 
        self.entry_heater1.setMaximum(1) 
        self.entry_heater1.setSingleStep(0.05)
        self.entry_heater1.setValue(0)
        self.btn_heater1_update = QPushButton('Update')
        self.btn_heater1_update.setDefault(False)

        self.entry_heater2 = QDoubleSpinBox()
        self.entry_heater2.setMinimum(0) 
        self.entry_heater2.setMaximum(1) 
        self.entry_heater2.setSingleStep(0.05)
        self.entry_heater2.setValue(0)
        self.btn_heater2_update = QPushButton('Update')
        self.btn_heater2_update.setDefault(False)


        self.entry_stopwatch = QDoubleSpinBox()
        self.entry_stopwatch.setMinimum(0) 
        self.entry_stopwatch.setMaximum(3600) 
        self.entry_stopwatch.setSingleStep(1)
        self.entry_stopwatch.setValue(0)
        self.label_stopwatch = QLabel()
        self.label_stopwatch.setNum(0)
        self.label_stopwatch.setFrameStyle(QFrame.Panel | QFrame.Sunken)
        self.btn_stopwatch = QPushButton('Count Down')
        self.btn_stopwatch.setDefault(False)
        
        grid_line0 = QGridLayout()
        grid_line0.addWidget(QLabel('X (mm)'), 0,0)
        grid_line0.addWidget(self.label_Xpos, 0,1)
        grid_line0.addWidget(QLabel('d (mm)'), 0,2)
        grid_line0.addWidget(self.entry_dX, 0,3)
        grid_line0.addWidget(QLabel('v (mm/s)'), 0,4)
        grid_line0.addWidget(self.entry_vX, 0,5)
        grid_line0.addWidget(QLabel('a (mm/s/s)'), 0,6)
        grid_line0.addWidget(self.entry_aX, 0,7)
        grid_line0.addWidget(QLabel('ustepping'), 0,8)
        grid_line0.addWidget(self.combo_ustepX, 0,9)
        grid_line0.addWidget(self.btn_moveX_forward, 0,10)
        grid_line0.addWidget(self.btn_moveX_backward, 0,11)
        grid_line0.addWidget(self.btn_cycleX, 0,12)

        grid_line1 = QGridLayout()
        grid_line1.addWidget(QLabel('Y (mm)'), 0,0)
        grid_line1.addWidget(self.label_Ypos, 0,1)
        grid_line1.addWidget(QLabel('d (mm)'), 0,2)
        grid_line1.addWidget(self.entry_dY, 0,3)
        grid_line1.addWidget(QLabel('v (mm/s)'), 0,4)
        grid_line1.addWidget(self.entry_vY, 0,5)
        grid_line1.addWidget(QLabel('a (mm/s/s)'), 0,6)
        grid_line1.addWidget(self.entry_aY, 0,7)
        grid_line1.addWidget(QLabel('ustepping'), 0,8)
        grid_line1.addWidget(self.combo_ustepY, 0,9)
        grid_line1.addWidget(self.btn_moveY_forward, 0,10)
        grid_line1.addWidget(self.btn_moveY_backward, 0,11)

        grid_line2 = QGridLayout()
        grid_line2.addWidget(QLabel('Z (mm)'), 0,0)
        grid_line2.addWidget(self.label_Zpos, 0,1)
        grid_line2.addWidget(QLabel('d (mm)'), 0,2)
        grid_line2.addWidget(self.entry_dZ, 0,3)
        grid_line2.addWidget(QLabel('v (mm/s)'), 0,4)
        grid_line2.addWidget(self.entry_vZ, 0,5)
        grid_line2.addWidget(QLabel('a (mm/s/s)'), 0,6)
        grid_line2.addWidget(self.entry_aZ, 0,7)
        grid_line2.addWidget(QLabel('ustepping'), 0,8)
        grid_line2.addWidget(self.combo_ustepZ, 0,9)
        grid_line2.addWidget(self.btn_moveZ_forward, 0,10)
        grid_line2.addWidget(self.btn_moveZ_backward, 0,11)
        grid_line2.addWidget(self.btn_cycleZ, 0,12)

        grid_line3 = QGridLayout()
        grid_line3.addWidget(QLabel('Heater 1 Power (0-1)'), 0,0)
        grid_line3.addWidget(self.entry_heater1, 0,1)
        grid_line3.addWidget(self.btn_heater1_update,0,2)
        grid_line3.addWidget(QLabel('Heater 2 Power (0-1)'), 0,3)
        grid_line3.addWidget(self.entry_heater2, 0,4)
        grid_line3.addWidget(self.btn_heater2_update,0,5)

        grid_line4 = QGridLayout()
        grid_line4.addWidget(QLabel('Stopwatch (s)'), 0,0)
        grid_line4.addWidget(self.entry_stopwatch, 0,1)
        grid_line4.addWidget(QLabel('Stopwatch'), 0,2)
        grid_line4.addWidget(self.label_stopwatch, 0,3)
        grid_line4.addWidget(self.btn_stopwatch, 0,4)
        grid_line4.addWidget(QLabel(' '), 0,5)
        grid_line4.addWidget(QLabel(' '), 0,6)
        grid_line4.addWidget(QLabel(' '), 0,7)

        self.grid = QGridLayout()
        # self.grid.addLayout(grid_line0,0,0)
        # self.grid.addLayout(grid_line1,1,0)
        self.grid.addLayout(grid_line2,2,0)
        self.grid.addLayout(grid_line3,3,0)
        self.grid.addLayout(grid_line4,4,0)
        self.setLayout(self.grid)

        self.btn_moveX_forward.clicked.connect(self.move_x_forward)
        self.btn_moveX_backward.clicked.connect(self.move_x_backward)
        self.btn_cycleX.clicked.connect(self.cycle_x)

        self.btn_moveY_forward.clicked.connect(self.move_y_forward)
        self.btn_moveY_backward.clicked.connect(self.move_y_backward)
        
        self.btn_moveZ_forward.clicked.connect(self.move_z_forward)
        self.btn_moveZ_backward.clicked.connect(self.move_z_backward)
        self.btn_cycleZ.clicked.connect(self.cycle_z)

        self.btn_heater1_update.clicked.connect(self.set_heater1_power)
        self.btn_heater2_update.clicked.connect(self.set_heater2_power)

        self.btn_stopwatch.clicked.connect(self.start_countdown_timer)
        
    def move_x_forward(self):
        self.navigationController.move_x(self.entry_dX.value(),self.entry_vX.value(),self.entry_aX.value(),int(self.combo_ustepX.currentText()))
    def move_x_backward(self):
        self.navigationController.move_x(-self.entry_dX.value(),self.entry_vX.value(),self.entry_aX.value(),int(self.combo_ustepX.currentText()))
    def move_y_forward(self):
        self.navigationController.move_y(self.entry_dY.value(),self.entry_vY.value(),self.entry_aY.value(),int(self.combo_ustepY.currentText()))
    def move_y_backward(self):
        self.navigationController.move_y(-self.entry_dY.value(),self.entry_vY.value(),self.entry_aY.value(),int(self.combo_ustepY.currentText()))
    def move_z_forward(self):
        self.navigationController.move_z(self.entry_dZ.value(),self.entry_vZ.value(),self.entry_aZ.value(),int(self.combo_ustepZ.currentText()))
    def move_z_backward(self):
        self.navigationController.move_z(-self.entry_dZ.value(),self.entry_vZ.value(),self.entry_aZ.value(),int(self.combo_ustepZ.currentText()))
    def cycle_x(self):
        self.navigationController.cycle_x(self.entry_dX.value(),self.entry_vX.value(),self.entry_aX.value(),int(self.combo_ustepX.currentText()))
    def cycle_z(self):
        self.navigationController.cycle_z(self.entry_dZ.value(),self.entry_vZ.value(),self.entry_aZ.value(),int(self.combo_ustepZ.currentText()))
    def set_heater1_power(self):
        self.navigationController.set_heater1_power(self.entry_heater1.value())
    def set_heater2_power(self):
        self.navigationController.set_heater2_power(self.entry_heater2.value())
    def start_countdown_timer(self):
        self.navigationController.start_countdown_timer(self.entry_stopwatch.value())
        self.label_stopwatch.setNum(self.entry_stopwatch.value())

class ControlPanel(QFrame):

    signal_logging_onoff = Signal(bool,str)

    def __init__(self, main=None, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.font = QFont()
        self.font.setPixelSize(16)
        self.add_components()
        self.setFrameStyle(QFrame.Panel | QFrame.Raised)

    def add_components(self):

        self.lineEdit_experimentID = QLineEdit()

        self.btn_logging_onoff = QPushButton('Logging On/Off')
        self.btn_logging_onoff.setDefault(False)
        self.btn_logging_onoff.setCheckable(True)
        self.btn_logging_onoff.setChecked(True)

        # self.label_print = QLabel()
        # self.label_print.setFrameStyle(QFrame.Panel | QFrame.Sunken)

        grid_line2 = QHBoxLayout()
        grid_line2.addWidget(QLabel('File Prefix'))
        grid_line2.addWidget(self.lineEdit_experimentID)
        grid_line2.addWidget(self.btn_logging_onoff)

        # grid_line11 = QGridLayout()
        # grid_line11.addWidget(self.label_print,0,0,10,0)

        # for displaying stepper position and flow/pressure measurements
        self.label_ch1 = QLabel()
        self.label_ch1.setFrameStyle(QFrame.Panel | QFrame.Sunken)
        self.label_ch1.setFixedWidth(50)
        self.label_ch2 = QLabel()
        self.label_ch2.setFrameStyle(QFrame.Panel | QFrame.Sunken)
        self.label_ch2.setFixedWidth(50)
        self.label_ch3 = QLabel()
        self.label_ch3.setFrameStyle(QFrame.Panel | QFrame.Sunken)
        self.label_ch3.setFixedWidth(50)

        # self.label_print = QLabel()
        # self.label_print.setFrameStyle(QFrame.Panel | QFrame.Sunken)

        grid_line3 = QHBoxLayout()
        grid_line3.addWidget(QLabel('ch1'))
        grid_line3.addWidget(self.label_ch1)
        grid_line3.addWidget(QLabel('ch2'))
        grid_line3.addWidget(self.label_ch2)
        # grid_line3.addWidget(QLabel('temperature (degree C)'))
        # grid_line3.addWidget(self.label_ch3)
        
        self.grid = QGridLayout()
        self.grid.addLayout(grid_line2,2,0)
        self.grid.addLayout(grid_line3,3,0)
        # self.grid.addWidget(self.label_print,3,0,1,8)

        self.setLayout(self.grid)
        self.btn_logging_onoff.clicked.connect(self.logging_onoff)

    def logging_onoff(self,state):
        self.signal_logging_onoff.emit(state,self.lineEdit_experimentID.text())


class WaveformDisplay(QFrame):

    def __init__(self, main=None, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.add_components()
        self.setFrameStyle(QFrame.Panel | QFrame.Raised)

    def add_components(self):
        # self.plotWidgets = {key: PlotWidget(title = key, color = 'b') for key in PLOTS}
        # self.plotWidgets['Airway Pressure'].plot1.setYRange(min=WAVEFORMS.PAW_MIN,max=WAVEFORMS.PAW_MAX)
        # self.plotWidgets['Flow Rate'].plot1.setYRange(min=WAVEFORMS.FLOW_MIN,max=WAVEFORMS.FLOW_MAX)
        # self.plotWidgets['Volume'].plot1.setYRange(min=WAVEFORMS.V_MIN,max=WAVEFORMS.V_MAX)

        # grid = QGridLayout() 
        # for ii, key in enumerate(PLOTS):
        #   grid.addWidget(self.plotWidgets[key], ii, 0,1,2)
        # self.setLayout(grid)

        self.plotWidget = []
        n = 2
        for i in range(n):
            self.plotWidget.append(PlotWidget())
        layout = QGridLayout() #layout = QStackedLayout()
        for i in range(n):
            layout.addWidget(self.plotWidget[i],i,0)
        self.setLayout(layout)


class PlotWidget(pg.GraphicsLayoutWidget):
    def __init__(self, window_title='',parent=None):
        super().__init__(parent)
        self.plotWidget = self.addPlot(title = '')
    def plot(self,x,y):
        self.plotWidget.plot(x,y,pen=(0,3),clear=True)


# set QT_API environment variable
import os 
os.environ["QT_API"] = "pyqt5"
import qtpy

# qt libraries
from qtpy.QtCore import *
from qtpy.QtWidgets import *
from qtpy.QtGui import *

# app specific libraries
import control.widgets as widgets
import control.core as core
import control.microcontroller as microcontroller
import control.utils as utils

from control._def import *

class OctopiGUI(QMainWindow):

	# variables
	fps_software_trigger = 100

	def __init__(self, *args, **kwargs):
		super().__init__(*args, **kwargs)

		# load objects
		self.microcontroller = microcontroller.Microcontroller()
		self.navigationController = core.NavigationController(self.microcontroller)
		self.logger = core.Logger()

		# load widgets
		self.navigationWidget = widgets.NavigationWidget(self.navigationController)
		self.waveformDisplay = widgets.WaveformDisplay()
		self.controlPanel = widgets.ControlPanel()
		self.logWidget = QListWidget()

		# layout widgets
		layout = QGridLayout() #layout = QStackedLayout()
		layout.addWidget(self.waveformDisplay,0,0)
		layout.addWidget(self.controlPanel,1,0)
		layout.addWidget(self.navigationWidget,2,0)
		layout.addWidget(self.logWidget,3,0)

		# transfer the layout to the central widget
		self.centralWidget = QWidget()
		self.centralWidget.setLayout(layout)
		self.setCentralWidget(self.centralWidget)

		# make connections
		self.navigationController.zPos.connect(self.navigationWidget.label_Zpos.setNum)

		self.controlPanel.signal_logging_onoff.connect(self.navigationController.logging_onoff)
		self.navigationController.signal_plot1.connect(self.waveformDisplay.plotWidget[0].plot)
		self.navigationController.signal_plot2.connect(self.waveformDisplay.plotWidget[1].plot)
		self.navigationController.signal_ch1.connect(self.controlPanel.label_ch1.setText)
		self.navigationController.signal_ch2.connect(self.controlPanel.label_ch2.setText)

		self.navigationController.log_message.connect(self.logger.log)
		self.navigationController.log_message.connect(self.logWidget.addItem)
		self.navigationController.signal_new_log_item.connect(self.logWidget.scrollToBottom)  
		# self.logWidget.itemEntered.connect(self.logWidget.scrollToBottom)

		self.navigationController.stopwatch_timeleft.connect(self.navigationWidget.label_stopwatch.setNum)

		self.logger.log(utils.timestamp() + 'Temperature Logging Started')
		self.logWidget.addItem(utils.timestamp() + 'Temperature Logging Started')

	def closeEvent(self, event):
		event.accept()
		# self.navigationController.home()
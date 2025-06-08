from queue import Queue
from PyQt6 import QtWidgets, QtCore
import pyqtgraph as pg

UPDATE_PERIOD = 100     # ms
BUF_SIZE = 1000  # samples

class QtGraph:
    def __init__(self, queue):
        self.app = None
        self.plot = None
        self.window = None
        self.queue = queue

        self.buffer_size = BUF_SIZE
        self.t_data = []
        self.x_data = []

        self.timer = None

    def init_ui(self):
        self.app = QtWidgets.QApplication([])
        self.window = QtWidgets.QMainWindow()
        self.window.setWindowTitle("Plots")

        graph_widget = pg.PlotWidget()
        self.window.setCentralWidget(graph_widget)

        self.plot = graph_widget.plot([], [], pen=pg.mkPen('b', width=2))


    def main(self):
        self.init_ui()

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_plots)
        self.timer.start(UPDATE_PERIOD)  # 10Hz


        self.window.show()
        self.app.exec()


    def update_plots(self):
        while not self.queue.empty():
            t, x = self.queue.get()
            self.t_data.append(t)
            self.x_data.append(x)

        # Ogranicz do 200 ostatnich próbek
        self.t_data = self.t_data[-self.buffer_size:]
        self.x_data = self.x_data[-self.buffer_size:]


        self.plot.setData(self.t_data, self.x_data)
        self.plot.setYRange(-0.2, 0.2)
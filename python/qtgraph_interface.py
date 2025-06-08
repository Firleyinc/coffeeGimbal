from queue import Queue
from PyQt6 import QtWidgets, QtCore
import pyqtgraph as pg

UPDATE_PERIOD = 50     # ms
BUF_SIZE = 300  # samples

class QtGraph:
    def __init__(self, queue):
        self.queue = queue

        self.plotWidget = None
        self.plot_s = None

        self.t_data = []
        self.data = []

    def init_ui(self):
        self.plotWidget = pg.PlotWidget()
        self.plotWidget.setWindowTitle("Plots")

        self.plotWidget.setLabel("bottom", "time t[s]")
        self.plotWidget.setLabel("left", "displacement s[m]")
        self.plotWidget.setYRange(-0.25, 0.25)

        self.plot_s = self.plotWidget.plot([], [], pen=pg.mkPen('r', width=2))

        self.plotWidget.show()

    def main(self):
        app = QtWidgets.QApplication([])
        self.init_ui()

        timer = QtCore.QTimer()
        timer.timeout.connect(self.update_plots)
        timer.start(UPDATE_PERIOD)  # 10Hz

        app.exec()


    def update_plots(self):
        while not self.queue.empty():
            t, x = self.queue.get()
            self.t_data.append(t)
            self.data.append(x)

        self.t_data = self.t_data[-BUF_SIZE:]
        self.data = self.data[-BUF_SIZE:]

        self.plot_s.setData(self.t_data, self.data)


if __name__ == '__main__':
    qt = QtGraph(Queue())
    qt.main()
from queue import Queue
from PyQt6 import QtWidgets, QtCore
import pyqtgraph as pg
import numpy as np

UPDATE_PERIOD = 50     # ms
BUF_SIZE = 300  # samples

class QtGraph:
    def __init__(self, queue):
        self.queue = queue

        # self.plotLabels = ['displacement s[m]', 'acceleration a[m/s^2]']
        self.window = None

        self.plots = None
        self.plots2 = None
        self.plots_a = None

        self.curves = None
        self.curves2 = None

        self.data = np.empty((0, 6))
        self.t_data = []

    # def init_ui(self):
    #     self.plotWidgets = pg.PlotWidget()
    #
    #     self.plotWidgets.setWindowTitle("Plots")
    #
    #     self.plotWidgets.setLabel("bottom", "time t[s]")
    #     self.plotWidgets.setLabel("left", "displacement s[m]")
    #     self.plotWidgets.setYRange(-0.25, 0.25)
    #
    #     # self.plotWidgets_a.setLabel("bottom", "time t[s]")
    #     # self.plotWidgets_a.setLabel("left", "acceleration a[m/s^2]")
    #     # self.plotWidgets_a.setYRange(-0.25, 0.25)
    #
    #     self.plots = self.plotWidgets.plot([], [], pen=pg.mkPen('r', name='s_x'))
    #     self.plots2 = self.plotWidgets.plot([], [], pen=pg.mkPen('g', name='s_y'))
    #
    #     self.plots_a = self.plotWidgets.plot([], [], pen=pg.mkPen('r', width=2))
    #
    #     self.plotWidgets.show()
    #     self.plotWidgets_a.show()

    def init_ui(self):
        self.window = pg.GraphicsLayoutWidget(title="Plots")

        p1 = self.window.addPlot(title="Displacement")
        self.window.nextRow()
        p2 = self.window.addPlot(title="Acceleration")

        self.plots = p1.plot(pen='r')
        self.plots_a = p2.plot(pen='r')

        p1.setYRange(-0.25, 0.25)
        p2.setYRange(-1, 1)

        self.window.show()

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
            self.data = np.vstack([self.data, x])

        self.t_data = self.t_data[-BUF_SIZE:]
        self.data = self.data[-BUF_SIZE:, :]

        self.plots.setData(self.t_data, self.data[:, 0])
        self.plots_a.setData(self.t_data, self.data[:, 2])


if __name__ == '__main__':
    qt = QtGraph(Queue())
    qt.main()
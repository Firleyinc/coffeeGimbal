from queue import Queue
from PyQt6 import QtWidgets, QtCore
import pyqtgraph as pg

UPDATE_PERIOD = 100     # ms
BUF_SIZE = 1000  # samples

class QtGraph:
    def __init__(self, queue):
        self.queue = queue

        self.plotWidget = None
        self.plot = None
        
        self.buffer_size = BUF_SIZE
        self.t_data = []
        self.x_data = []

    def init_ui(self):
        self.plotWidget = pg.PlotWidget()
        self.plotWidget.setWindowTitle("Plots")

        self.plot = self.plotWidget.plot([], [], pen=pg.mkPen('b', width=2))

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
            self.x_data.append(x)

        # Ogranicz do 200 ostatnich próbek
        self.t_data = self.t_data[-self.buffer_size:]
        self.x_data = self.x_data[-self.buffer_size:]


        self.plot.setData(self.t_data, self.x_data)


if __name__ == '__main__':
    qt = QtGraph(Queue())
    qt.main()
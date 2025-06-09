from queue import Queue
from PyQt6 import QtWidgets, QtCore
import pyqtgraph as pg
import numpy as np
from dataclasses import dataclass, field
from typing import Any, List

from pyqtgraph import plots

UPDATE_PERIOD = 20     # ms
BUF_SIZE = 300  # samples

@dataclass
class Curve:
    plot: Any = None
    data: List[int] = field(default_factory=list)
    penColor: str = 'r'
    yRangeMin: float = None
    yRangeMax: float = 1.

    def create_curve(self, parent):
        if self.yRangeMin is None:
            self.yRangeMin = -self.yRangeMax
        self.plot = parent.plot(pen=self.penColor)
        parent.setYRange(self.yRangeMin, self.yRangeMax)

    def set_data(self, time):
        if self.plot is None:
            return
        self.plot.setData(time, self.data)

    def clip_horizon(self, horizon):
        self.data = self.data[-horizon:]



class QtGraph:
    def __init__(self, queue):
        self.queue = queue

        self.window = None

        self.curves = {
            's_x': Curve(yRangeMax=0.2),
            's_y': Curve(yRangeMax=0.2, penColor='g'),
            'a_x': Curve(),
            'a_y': Curve(penColor='g'),
            # 'a_x_dot': Curve(yRangeMax=10)
        }

        self.t_data = []

    def init_ui(self):
        def create_plot(title, leftLabel, bottomLabel = 't[s]'):
            plot = self.window.addPlot(title=title)
            plot.setLabel('left', leftLabel)
            plot.setLabel('bottom', bottomLabel)
            return plot

        self.window = pg.GraphicsLayoutWidget(title="Plots")

        plot_s = create_plot("Displacement", 's[m]')
        self.window.nextRow()
        plot_a = create_plot("Acceleration", 'a[m/s^2]')
        self.window.nextRow()
        # plot_a_dot = create_plot("Jerk", 'a_dot[m/s^3]')

        self.curves['s_y'].create_curve(plot_s)
        self.curves['s_x'].create_curve(plot_s)
        self.curves['a_y'].create_curve(plot_a)
        self.curves['a_x'].create_curve(plot_a)

        self.window.show()

    def main(self):
        app = QtWidgets.QApplication([])
        self.init_ui()

        timer = QtCore.QTimer()
        timer.timeout.connect(self.update_plots)
        timer.start(UPDATE_PERIOD)

        app.exec()


    def update_plots(self):
        while not self.queue.empty():
            t, x = self.queue.get()
            self.t_data.append(t)
            for name in self.curves:
                self.curves[name].data.append(x[name])

        self.t_data = self.t_data[-BUF_SIZE:]

        for name in self.curves:
            self.curves[name].clip_horizon(BUF_SIZE)
            self.curves[name].set_data(self.t_data)


if __name__ == '__main__':
    qt = QtGraph(Queue())
    qt.main()
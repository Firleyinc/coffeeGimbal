from queue import Queue
from PyQt6.QtWidgets import QVBoxLayout, QCheckBox, QLabel
from PyQt6 import QtWidgets, QtCore
import pyqtgraph as pg
import numpy as np
from dataclasses import dataclass, field
from typing import Any, List

UPDATE_PERIOD = 10     # ms
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
    def __init__(self, sim2ui_queue, ui2sim_queue):
        self.sim2ui_queue = sim2ui_queue
        self.ui2sim_queue = ui2sim_queue

        self.window = None
        self.checkboxes = []

        self.curves = {
            's_x': Curve(yRangeMax=0.2),
            's_y': Curve(yRangeMax=0.2, penColor='g'),
            'a_x': Curve(yRangeMax=10.),
            'a_y': Curve(yRangeMax=10., penColor='g'),
            'a_x_dot': Curve(yRangeMax=10.),
            'a_y_dot': Curve(yRangeMax=10., penColor='g'),
            'theta_x': Curve(penColor='m'),
            'theta_y': Curve(penColor='c'),
        }

        self.t_data = []

    def init_ui(self):
        def create_plot(title, leftLabel, bottomLabel = 't[s]'):
            plot = plotLayout.addPlot(title=title)
            plot.setLabel('left', leftLabel)
            plot.setLabel('bottom', bottomLabel)
            return plot

        self.window = QtWidgets.QWidget()
        self.window.setWindowTitle("Coffee Gimbal Diagnostics")
        self.window.resize(1200, 800)
        mainLayout = QtWidgets.QVBoxLayout(self.window)

        plotLayout = pg.GraphicsLayoutWidget()
        checkboxLayout = QtWidgets.QHBoxLayout()

        self.checkboxes.append(QtWidgets.QCheckBox("Sine generator"))
        self.checkboxes[0].stateChanged.connect(self.update_checkbox)

        for checkbox in self.checkboxes:
            checkboxLayout.addWidget(checkbox)

        plot_s = create_plot("Displacement", 's[m]')
        plotLayout.nextRow()
        plot_a = create_plot("Acceleration", 'a[m/s^2]')
        plotLayout.nextRow()
        plot_a_dot = create_plot("Jerk", 'a_dot[m/s^3]')
        plotLayout.nextRow()
        plot_theta = create_plot("Mug rotation", 'theta[rad]')
        plotLayout.nextRow()


        self.curves['s_y'].create_curve(plot_s)
        self.curves['s_x'].create_curve(plot_s)
        self.curves['a_y'].create_curve(plot_a)
        self.curves['a_x'].create_curve(plot_a)
        self.curves['a_y_dot'].create_curve(plot_a_dot)
        self.curves['a_x_dot'].create_curve(plot_a_dot)
        self.curves['theta_x'].create_curve(plot_theta)
        self.curves['theta_y'].create_curve(plot_theta)

        mainLayout.addLayout(checkboxLayout)
        mainLayout.addWidget(plotLayout)

        self.window.show()

    def main(self):
        app = QtWidgets.QApplication([])
        self.init_ui()

        timer = QtCore.QTimer()
        timer.timeout.connect(self.update_plots)
        timer.start(UPDATE_PERIOD)

        app.exec()


    def update_plots(self):
        while not self.sim2ui_queue.empty():
            t, x = self.sim2ui_queue.get()
            self.t_data.append(t)
            for name in self.curves:
                self.curves[name].data.append(x[name])

        self.t_data = self.t_data[-BUF_SIZE:]

        for name in self.curves:
            self.curves[name].clip_horizon(BUF_SIZE)
            self.curves[name].set_data(self.t_data)

    def update_checkbox(self, state):
        self.ui2sim_queue.put({'x_traj_gen': state})


if __name__ == '__main__':
    qt = QtGraph(Queue())
    qt.main()
import numpy as np
import matplotlib.pyplot as plt
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtWidgets
import threading

import mujoco_interface
import controller

MODEL_PATH = '../simulation/gimbal_simplified.xml'
PLOT_HORIZON = 200
TP = 0.001

class Gimbal:
    def __init__(self):

        self.sim = mujoco_interface.MujocoSimulator(MODEL_PATH)
        self.x_control = controller.Controller()





        # self.app = pg.mkQApp()
        # self.window = pg.GraphicsLayoutWidget(title="Real-time Plot")
        # self.window.show()
        # plot = self.window.addPlot(title="Real-time Plot")
        # self.curve = plot.plot()

        # self.gui_thread = threading.Thread(target=self.start_gui, daemon=True)
        # self.gui_thread.start()

        # Bufor danych
        self.x_data = []
        self.t_data = []
        self.t = 0.


    def main(self):
        def integrate(var, lastVar, dt):
            return (var - lastVar) / dt

        a_x_last = a_y_last = 0.

        while self.sim.is_running():

            varNames = ['move_x', 'move_y', 'rot_x', 'rot_y']
            s_x, s_y, tau_x, tau_y = [self.sim.get_pos(name) for name in varNames]  # prędkości
            a_x, a_y = [self.sim.get_acc(name) for name in varNames[:2]]  # przyspieszenia
            [a_x_dot, a_y_dot] = integrate(np.array([a_x, a_y]), np.array([a_x_last, a_y_last]), TP)  # zrywy
            a_x_last, a_y_last = a_x, a_y

            self.t_data.append(self.t)
            self.x_data.append(s_x)
            if len(self.t_data) > PLOT_HORIZON:
                self.t_data.pop(0)
                self.x_data.pop(0)

            # self.curve.setData(self.t_data, self.x_data)

            self.sim.step()
            self.t += TP

    def start_gui(self):
        app = pg.mkQApp()

        window = pg.GraphicsLayoutWidget(title="Real-time Plot")
        window.show()

        plot = window.addPlot(title="s_x over time")
        curve = plot.plot(pen='y')

        timer = pg.QtCore.QTimer()

        def update_plot():
            with self.lock:
                curve.setData(list(self.t_buffer), list(self.x_buffer))

        timer.timeout.connect(update_plot)
        timer.start(250)  # aktualizacja co 50 ms

        self.app.exec()





if __name__ == '__main__':
    gimbal = Gimbal()
    gimbal.main()

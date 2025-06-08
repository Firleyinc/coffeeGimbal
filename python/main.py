import numpy as np
from multiprocessing import Process, Queue


import mujoco_interface
import controller
from qtgraph_interface import QtGraph

MODEL_PATH = '../simulation/gimbal_simplified.xml'  # ścieżka do modelu MuJoCo
TP = 0.001
PLOT_TP = 0.1   # okres dodawania sampli do plotu

class Gimbal:
    def __init__(self, queue):

        self.sim = mujoco_interface.MujocoSimulator(MODEL_PATH)
        self.x_control = controller.Controller()
        self.queue = queue


    def main(self, queue):
        def integrate(var, lastVar, dt):
            return (var - lastVar) / dt

        a_x_last = a_y_last = 0.
        t = t_last = 0.

        self.sim.start()
        while self.sim.is_running():

            varNames = ['move_x', 'move_y', 'rot_x', 'rot_y']
            s_x, s_y, tau_x, tau_y = [self.sim.get_pos(name) for name in varNames]  # prędkości
            a_x, a_y = [self.sim.get_acc(name) for name in varNames[:2]]  # przyspieszenia
            [a_x_dot, a_y_dot] = integrate(np.array([a_x, a_y]), np.array([a_x_last, a_y_last]), TP)  # zrywy
            a_x_last, a_y_last = a_x, a_y

            if t - t_last > PLOT_TP:
                self.queue.put((t, s_x))
                t_last = t

            self.sim.step()
            t += TP








if __name__ == '__main__':
    q = Queue()

    gimbal = Gimbal(q)
    plotter = QtGraph(q)

    procGimbal = Process(target=gimbal.main, args=(q,))
    procPlotter = Process(target=plotter.main)

    procGimbal.start()
    procPlotter.start()

    procGimbal.join()
    procPlotter.join()

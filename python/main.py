import time

import numpy as np
from multiprocessing import Process, Queue


import mujoco_interface
import controller
import traj_gen
from qtgraph_interface import QtGraph

MODEL_PATH = '../simulation/gimbal_simplified.xml'  # ścieżka do modelu MuJoCo
TP = 0.01
PLOT_TP = 0.005   # okres dodawania sampli do plotu

class Gimbal:
    def __init__(self, queue):
        self.queue = queue
        self.time = 0.

        self.sim = mujoco_interface.MujocoSimulator(MODEL_PATH)
        self.x_control = controller.Controller()
        self.x_traj = traj_gen.TrajGen(amplitude=0.2)



    def main(self):
        def integrate(var, lastVar, dt):
            return (var - lastVar) / dt

        a_x_last = a_y_last = 0.
        t_last = 0.

        self.sim.start()
        while self.sim.is_running():
            startTime = time.time()

            varNames = ['move_x', 'move_y', 'rot_x', 'rot_y']
            s_x, s_y, theta_x, theta_y = [self.sim.get_pos(name) for name in varNames]  # przemieszczenia
            a_x, a_y = [self.sim.get_acc(name) for name in varNames[:2]]  # przyspieszenia
            [a_x_dot, a_y_dot] = integrate(np.array([a_x, a_y]), np.array([a_x_last, a_y_last]), TP)  # zrywy
            a_x_last, a_y_last = a_x, a_y

            if self.time - t_last > PLOT_TP:
                self.queue.put((self.time, {'s_x': s_x,
                                    's_y': s_y,
                                    'a_x': a_x,
                                    'a_y': a_y,
                                    'a_x_dot': a_x_dot,
                                    'a_y_dot': a_y_dot}))
                t_last = self.time

            s_x = self.x_traj.sine(self.time)
            s_y = self.x_traj.sine(self.time, phase=np.deg2rad(90))

            self.sim.set_pos(varNames[0], s_x)
            self.sim.set_pos(varNames[1], s_y)

            self.sim.step()
            time.sleep(max(0, TP - (time.time() - startTime)))
            self.time += TP








if __name__ == '__main__':
    q = Queue()

    gimbal = Gimbal(q)
    plotter = QtGraph(q)

    procGimbal = Process(target=gimbal.main)
    procPlotter = Process(target=plotter.main)

    procGimbal.start()
    procPlotter.start()

    while procGimbal.is_alive() and procPlotter.is_alive():
        time.sleep(0.1)

    procGimbal.terminate()
    procPlotter.terminate()

    procGimbal.join()
    procPlotter.join()

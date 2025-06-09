import time

import numpy as np
from multiprocessing import Process, Queue
import mujoco

import mujoco_interface
import controller
import traj_gen
from qtgraph_interface import QtGraph

MODEL_PATH = '../simulation/gimbal_simplified.xml'  # ścieżka do modelu MuJoCo
TP = 0.01
PLOT_TP = 0.005   # okres dodawania sampli do plotu

class Gimbal:
    def __init__(self, sim2ui_queue, ui2sim_queue):
        self.sim2ui_queue = sim2ui_queue
        self.ui2sim_queue = ui2sim_queue
        self.t = 0.

        self.sim = mujoco_interface.MujocoSimulator(MODEL_PATH)
        self.x_control = controller.Controller()
        self.x_traj = traj_gen.TrajGen(amplitude=0.2)

        self.enable_traj_gen = False

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

            if self.t - t_last > PLOT_TP:
                self.sim2ui_queue.put((self.t, {'s_x': s_x,
                                    's_y': s_y,
                                    'a_x': a_x,
                                    'a_y': a_y,
                                    'a_x_dot': a_x_dot,
                                    'a_y_dot': a_y_dot}))
                t_last = self.t

            self.read_queue()

            if self.enable_traj_gen:
                s_x = self.x_traj.sine(self.t, freq = 0.2)
                # s_y = self.x_traj.sine(self.t, phase=np.deg2rad(90))

            self.sim.set_pos(varNames[0], s_x)
            self.sim.set_pos(varNames[1], s_y)

            self.sim.step()
            time.sleep(max(0, TP - (time.time() - startTime)))
            self.t += TP

    def read_queue(self):
        while not self.ui2sim_queue.empty():
            state = self.ui2sim_queue.get()
            if 'checkbox' in state:
                self.enable_traj_gen = True if state['checkbox'] > 0 else False







if __name__ == '__main__':
    q_sim2ui = Queue()
    q_ui2sim = Queue()

    gimbal = Gimbal(q_sim2ui, q_ui2sim)
    plotter = QtGraph(q_sim2ui, q_ui2sim)

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

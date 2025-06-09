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
# PLOT_TP = 0.005   # okres dodawania sampli do plotu

class Gimbal:
    def __init__(self, sim2ui_queue, ui2sim_queue):
        self.sim2ui_queue = sim2ui_queue
        self.ui2sim_queue = ui2sim_queue
        self.t = 0.

        self.sim = mujoco_interface.MujocoSimulator(MODEL_PATH)
        self.x_control = controller.Controller()
        self.x_traj = traj_gen.TrajGen(amplitude=1.57, freq=0.5)

        self.enable_traj_gen = False

        self.checkboxes = {}
        self.inputParameters = {}
        self.outputParameters = {}

    def main(self):

        self.sim.start()
        while self.sim.is_running():
            stepStartTime = time.time()
            self.outputParameters.clear()

            self.get_sim_params()

            self.set_sim2ui_queue()

            self.get_ui2sim_queue()

            self.traj_gen_step()

            self.set_sim_params()

            self.sim.step()

            time.sleep(max(0, TP - (time.time() - stepStartTime)))
            self.t += TP

    def integrate(self, var, lastVar, dt):
        return (var - lastVar) / dt

    def get_sim_params(self, a_last=[(0, 0)]):
        varNames = ['move_x', 'move_y', 'rot_x', 'rot_y']
        s_x, s_y, theta_x, theta_y = [self.sim.get_pos(name) for name in varNames]  # przemieszczenia
        a_x, a_y = [self.sim.get_acc(name) for name in varNames[:2]]  # przyspieszenia
        [a_x_dot, a_y_dot] = self.integrate(np.array([a_x, a_y]), np.array(a_last[0]), TP)  # zrywy
        a_last[0] = (a_x, a_y)

        self.inputParameters = {'s_x': s_x,
                           's_y': s_y,
                           'theta_x': theta_x,
                           'theta_y': theta_y,
                           'a_x': a_x,
                           'a_y': a_y,
                           'a_x_dot': a_x_dot,
                           'a_y_dot': a_y_dot}

    def set_sim_params(self):
        varNames = {'s_x': 'move_x',
                    's_y': 'move_y',
                    'theta_x': 'rot_x',
                    'theta_y': 'rot_y'}
        for name in self.outputParameters:
            if name in varNames:
                self.sim.set_pos(varNames[name], self.outputParameters[name])

    def set_sim2ui_queue(self): # , t_last=[0]):
        # if self.t - t_last[0] > PLOT_TP:
        self.sim2ui_queue.put((self.t, self.inputParameters))
        # t_last[0] = self.t  # pseudo static

    def get_ui2sim_queue(self):
        while not self.ui2sim_queue.empty():
            state = self.ui2sim_queue.get()
            self.checkboxes.update(state)

    def traj_gen_step(self):
        if 'x_traj_gen' in self.checkboxes:
            if self.checkboxes['x_traj_gen']:
                self.outputParameters['s_x'] = self.x_traj.sine(self.t)
            else:
                self.x_traj.sync(self.t)
            # s_y = self.x_traj.sine(self.t, phase=np.deg2rad(90))









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

import time

import numpy as np
from multiprocessing import Process, Queue
import mujoco
import os

import mujoco_interface
import controller
import traj_gen
from qtgraph_interface import QtGraph

from Controllers.Gimbal_Controller import FullGimbalController

MODEL_PATH = '../simulation/gimbal_simplified.xml'  # ścieżka do modelu MuJoCo
TP = 0.002
PLOT_TP = 0.01   # okres dodawania sampli do plotu

class Gimbal:
    def __init__(self, sim2ui_queue, ui2sim_queue):
        self.sim2ui_queue = sim2ui_queue
        self.ui2sim_queue = ui2sim_queue
        self.t = 0.

        cwd = os.path.dirname(os.path.abspath(__file__))
        self.sim = mujoco_interface.MujocoSimulator(os.path.join(cwd, MODEL_PATH))
        self.x_control = controller.Controller(TP)
        self.x_traj = traj_gen.TrajGen(amplitude=0.2, freq=0.5)

        self.enable_traj_gen = False

        self.checkboxes = {}
        self.inputParameters = {}
        self.outputParameters = {}

        self.checkboxes['controllers'] = 1  # workaround, żeby sterownik był domyślnie odpalony

    def main(self):

        theta_prev = 0.

        self.sim.start()
        while self.sim.is_running():
            # przygotowanie kroku symulacji
            stepStartTime = time.time()
            self.outputParameters.clear()

            # pobranie wektora parametrów z ostatniego kroku symylatora
            self.get_sim_params()

            # wymiana danych pomiędzy symulatorem, a procesem od generowania wykresów
            self.set_sim2ui_queue()
            self.get_ui2sim_queue()

            #krok generatora trajektorii (o ile jest włączony)
            self.traj_gen_step()

            # Tutaj znaj    duje się implementacja kontrolera. Wektor parametrów wejściowych jest w self.inputParameters, a wyjścia
            # należy zapisywać do self.outputParameters. Oba z tych obiektów są słownikami.

            #theta = self.x_control.step(self.inputParameters['a_x'], self.inputParameters['a_x_dot'])

            controller_params = (0.3, 0.005, 0.05)
            acceleration = self.inputParameters['a_x']
            jerk = self.inputParameters['a_x_dot']

            self.x_control = FullGimbalController(controller_params=controller_params, acceleration=acceleration, jerk=jerk)
            theta = self.x_control.update()

            if self.checkboxes.get('controllers'):
                self.outputParameters['theta_x'] = theta

            # przesłanie wektora sterowań do symulatora
            self.set_sim_params()

            # krok symulacji MuJoCo
            self.sim.step()

            # kontrola czasu trwania kroku TP
            time.sleep(max(0, TP - (time.time() - stepStartTime)))
            self.t += TP

    def derive(self, var, lastVar, dt):
        return (var - lastVar) / dt



    def get_sim_params(self, a_last=[(0, 0)]):
        varNames = ['move_x', 'move_y', 'rot_x', 'rot_y']
        s_x, s_y, theta_x, theta_y = [self.sim.get_pos(name) for name in varNames]  # przemieszczenia
        a_x, a_y = [self.sim.get_acc(name) for name in varNames[:2]]  # przyspieszenia
        [a_x_dot, a_y_dot] = self.derive(np.array([a_x, a_y]), np.array(a_last[0]), TP)  # zrywy
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

    def set_sim2ui_queue(self, t_last=[0]):
        if self.t - t_last[0] > PLOT_TP:
            self.sim2ui_queue.put((self.t, self.inputParameters))
            t_last[0] = self.t  # pseudo static

    def get_ui2sim_queue(self):
        while not self.ui2sim_queue.empty():
            state = self.ui2sim_queue.get()
            self.checkboxes.update(state)

    def traj_gen_step(self):
            if self.checkboxes.get('sine_generator'):
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

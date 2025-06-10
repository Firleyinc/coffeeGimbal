import mujoco
import mujoco.viewer

class MujocoSimulator:
    def __init__(self, modelPath):
        self.model = mujoco.MjModel.from_xml_path(modelPath)
        self.data = mujoco.MjData(self.model)
        self.viewer = None

    def start(self):
        self.viewer = mujoco.viewer.launch_passive(self.model, self.data)

    def is_running(self):
        return self.viewer.is_running()

    def step(self):
        mujoco.mj_step(self.model, self.data)
        self.viewer.sync()

    def get_pos(self, name, objType=mujoco.mjtObj.mjOBJ_JOINT):
        return self.data.qpos[mujoco.mj_name2id(self.model, objType, name)]

    def get_acc(self, name, objType=mujoco.mjtObj.mjOBJ_JOINT):
        return self.data.qacc[mujoco.mj_name2id(self.model, objType, name)]

    def set_pos(self, name, value, objType=mujoco.mjtObj.mjOBJ_ACTUATOR):
        self.data.ctrl[mujoco.mj_name2id(self.model, objType, name)] = value



    # def set_variable(self, name, value, objType=mujoco.mjtObj.mjOBJ_JOINT):
    #     self.data[mujoco.mj_name2id(self.model, objType, name)] = value


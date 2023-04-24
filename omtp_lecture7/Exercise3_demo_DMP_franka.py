

import time
from threading import Thread

import glfw
import mujoco
import numpy as np
import matplotlib.pyplot as plt
from dmp import dmp_joint
import roboticstoolbox as rp
import spatialmath as sm
import pandas as pd
import time



class Demo:

    qpos0 = [0, -0.785, 0, -2.356, 0, 1.571, 0.785]
    print(qpos0)
    height, width = 600, 800  # Rendering window resolution.
    fps = 30  # Rendering framerate.

    def __init__(self) -> None:

        demo_filename = "Jtrajectory.csv"
        self.demo = pd.read_csv(demo_filename)
        self.qtrj = self.demo.to_numpy()
        self.dt = 1/100

        self.model = mujoco.MjModel.from_xml_path('franka_emika_panda/scene.xml')
        self.data = mujoco.MjData(self.model)
        self.cam = mujoco.MjvCamera()
        self.cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
        self.cam.fixedcamid = 0
        self.scene = mujoco.MjvScene(self.model, maxgeom=10000)
        for i in range(1, 8):
            self.data.joint(f"joint{i}").qpos = self.qpos0[i - 1]
        mujoco.mj_forward(self.model, self.data)

    def getState(self):
        ## State of the simulater robot 
        qState=[]    
        for i in range(1, 8):
            qState.append(float(self.data.joint(f"joint{i}").qpos)) 
        return qState
    

    def step(self) -> None:
        
        ## Move the robot 0.3 m downwards and send to robot 
        panda = rp.models.Panda() # load the robot kinematics from the robotics toolbox
        ## read current state of the robot in Mujoco
        q_cur =self.getState()
        ## set current state to the kinematics model
        panda.q = q_cur

        ## calculate transformation 
        Tep = sm.SE3.Trans(0, 0, -0.3)* panda.fkine(panda.q)
        ## define carthesian linear trajectory 
        Trj = rp.ctraj(panda.fkine(panda.q), Tep, 500)
       
        ## Do inverse kinematics 
        qik = panda.ikine_LM(Trj, q0=panda.q)

        ## send to simulator 
        for qs in qik.q:
            for i in range(1, 8):
                self.data.joint(f"joint{i}").qpos = qs[i - 1]
            mujoco.mj_step(self.model, self.data)
            time.sleep(self.dt)

        print('Done')

    def render(self) -> None:
        glfw.init()
        glfw.window_hint(glfw.SAMPLES, 8)
        window = glfw.create_window(self.width, self.height, "Demo", None, None)
        glfw.make_context_current(window)
        self.context = mujoco.MjrContext(
            self.model, mujoco.mjtFontScale.mjFONTSCALE_100
        )
        opt = mujoco.MjvOption()
        pert = mujoco.MjvPerturb()
        viewport = mujoco.MjrRect(0, 0, self.width, self.height)
        while not glfw.window_should_close(window):
            w, h = glfw.get_framebuffer_size(window)
            viewport.width = w
            viewport.height = h
            mujoco.mjv_updateScene(
                self.model,
                self.data,
                opt,
                pert,
                self.cam,
                mujoco.mjtCatBit.mjCAT_ALL,
                self.scene,
            )
            mujoco.mjr_render(viewport, self.scene, self.context)
            time.sleep(1.0 / self.fps)
            glfw.swap_buffers(window)
            glfw.poll_events()
        self.run = False
        glfw.terminate()

    def start(self) -> None:
        step_thread = Thread(target=self.step)
        step_thread.start()
        self.render()


if __name__ == "__main__":
    Demo().start()




"""
Single-link inverted pendulum simulation with PD and LQR control.

Author: Jaehyun Park
GitHub: https://github.com/blueed9696/Pendulum_Control
License: MIT
"""
import pybullet as p
import pybullet_data
import time
import math
from model import Model

# Connect
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.setRealTimeSimulation(0)

p.loadURDF("plane.urdf")

# Control/Simulation loop
dt = 1.0 / 48000

# Pendulum parameters
model = Model()
L = model.rod_length
ball_radius = model.ball_radius
ball_mass = model.ball_mass
rod_thickness = model.rod_thickness

# Collision for rod and ball
rod_collision = p.createCollisionShape(
    p.GEOM_CAPSULE,
    radius=rod_thickness,
    height=L,
    collisionFramePosition=[0, 0, -L/2]
)

ball_collision = p.createCollisionShape(
    p.GEOM_SPHERE,
    radius=ball_radius
)

# Base anchor (massless)
base_mass = 0
base_pos = [0, 0, L*2]   # pivot point
base_ori = [0, 0, 0, 1]

# Define two links:
#    Link 0: massless rod
#    Link 1: massive ball at the end of the rod
link_masses = [
    0.0,         # massless rod
    ball_mass
]

link_collision = [
    rod_collision,
    ball_collision
]

link_visual = [-1, -1]

link_positions = [
    [0, 0, 0], 
    [0, 0, -L]  
]

link_orientations = [
    [0, 0, 0, 1],
    [0, 0, 0, 1]
]

link_inertial_pos = [[0,0,0],[0,0,0]]
link_inertial_ori = [[0,0,0,1],[0,0,0,1]]

# Joint structure
link_parents = [0, 1]     

link_joint_types = [
    p.JOINT_REVOLUTE,  # base → rod
    p.JOINT_FIXED      # rod → ball
]

link_joint_axes = [
    [1, 0, 0],       
    [0, 0, 0]        
]

# Create multibody system
pendulum = p.createMultiBody(
    baseMass=base_mass,
    baseCollisionShapeIndex=-1,
    baseVisualShapeIndex=-1,
    basePosition=base_pos,
    baseOrientation=base_ori,
    linkMasses=link_masses,
    linkCollisionShapeIndices=link_collision,
    linkVisualShapeIndices=link_visual,
    linkPositions=link_positions,
    linkOrientations=link_orientations,
    linkInertialFramePositions=link_inertial_pos,
    linkInertialFrameOrientations=link_inertial_ori,
    linkParentIndices=link_parents,
    linkJointTypes=link_joint_types,
    linkJointAxis=link_joint_axes
)

# Disable motor → free pendulum
p.setJointMotorControl2(
    pendulum,
    0,                       
    p.VELOCITY_CONTROL,
    force=0
)

# Give initial angle
p.resetJointState(pendulum, 0, targetValue = math.pi-math.radians(30))

from controller import Controller
controller = Controller(dt)

text_pos = [0, 0, 2]

theta_text_id = p.addUserDebugText(
        "theta: 0.00 deg \n torque: 0.00 Nm",
        text_pos,
        textColorRGB=[1, 1, 1],
        textSize=1.5,
        lifeTime=0
    )

while True:
    p.stepSimulation()

    # Get joint state: joint angle = theta (from vertical)
    joint_state = p.getJointState(pendulum, 0)
    q = joint_state[0]
    q_dot = joint_state[1]

    theta = math.pi - q # radians

    # PD Controller
    # tau = controller.PD_control(theta, -q_dot)
    # p.setJointMotorControl2(
    #     pendulum,
    #     0,                       
    #     p.TORQUE_CONTROL,
    #     force = tau
    # )

    # LQR Controller
    # tau = controller.LQR(theta, -q_dot)
    # p.setJointMotorControl2(
    #     pendulum,
    #     0,                       
    #     p.TORQUE_CONTROL,
    #     force = -tau)

    # MPC Controller
    tau = controller.linear_MPC(theta, -q_dot)
    p.setJointMotorControl2(
        pendulum,
        0,
        p.TORQUE_CONTROL,
        force=-tau
    )

    p.addUserDebugText(
        f"theta: {theta*180/math.pi:5.2f} deg\ntorque: {tau:5.2f} Nm",
        text_pos,
        textColorRGB=[1, 1, 1],
        textSize=1.5,
        lifeTime=0,
        replaceItemUniqueId=theta_text_id
    )


    time.sleep(dt)

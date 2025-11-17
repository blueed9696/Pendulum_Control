import pybullet as p
import pybullet_data
import time
import math
from model import Model

# Model reference
# https://scaron.info/robotics/wheeled-inverted-pendulum-model.html

# 1. Connect
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.setRealTimeSimulation(0)

# 2. Ground
p.loadURDF("plane.urdf")

# 10. Simulation loop
dt = 1.0 / 2400

# 3. Pendulum parameters
model = Model()
L = model.rod_length
ball_radius = model.ball_radius
ball_mass = model.ball_mass
rod_thickness = model.rod_thickness

# 4. Collision for rod and ball
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

# 5. Base anchor (massless)
base_mass = 0
base_pos = [0, 0, L*2]   # pivot point
base_ori = [0, 0, 0, 1]

# 6. Define two links:
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

# 7. Create multibody system
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

# 8. Disable motor → free pendulum
p.setJointMotorControl2(
    pendulum,
    0,                       
    p.VELOCITY_CONTROL,
    force=0
)

# 9. Give initial angle
p.resetJointState(pendulum, 0, targetValue = math.pi-math.radians(30))

from controller import Controller
controller = Controller(dt)

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
    tau = controller.LQR(theta, -joint_state[1])
    p.setJointMotorControl2(
        pendulum,
        0,                       
        p.TORQUE_CONTROL,
        force = -tau)

    print(f"theta = {theta*180/math.pi:.2f} deg, tau = {tau:.2f}")
    time.sleep(dt)

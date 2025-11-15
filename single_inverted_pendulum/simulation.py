import pybullet as p
import pybullet_data
import time
import math

# 1. Connect
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.setRealTimeSimulation(0)

# 2. Ground
p.loadURDF("plane.urdf")

# 3. Pendulum parameters
L = 1.0           # rod length
ball_radius = 0.05 
ball_mass = 1.0 
rod_thickness = 0.01

# 4. Collision for rod and ball
rod_collision = p.createCollisionShape(
    p.GEOM_CAPSULE,
    radius=rod_thickness,
    height=L
)

ball_collision = p.createCollisionShape(
    p.GEOM_SPHERE,
    radius=ball_radius
)

# 5. Base anchor (massless)
base_mass = 0
base_pos = [0, 0, 2.0]   # pivot point
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

# IMPORTANT: rod origin is at its center → offset by -L/2
# so the top of the rod coincides with the base (pivot)
link_positions = [
    [0, 0, -L/2],   # rod centered below base
    [0, 0, -(L/2 + rod_thickness + ball_radius)]  
]

link_orientations = [
    [0, 0, 0, 1],
    [0, 0, 0, 1]
]

link_inertial_pos = [[0,0,0],[0,0,0]]
link_inertial_ori = [[0,0,0,1],[0,0,0,1]]

# Joint structure:
link_parents = [0, 1]     

link_joint_types = [
    p.JOINT_REVOLUTE,  # base → rod
    p.JOINT_FIXED      # rod → ball
]

link_joint_axes = [
    [0, 1, 0],         # swing in XZ plane
    [0, 0, 0]          # fixed joint → no motion
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
p.resetJointState(pendulum, 0, math.radians(170))

# 10. Simulation loop
dt = 1.0 / 24000
while True:
    p.stepSimulation()

    # Get joint state: joint angle = theta (from vertical)
    joint_state = p.getJointState(pendulum, 0)
    theta = joint_state[0] + math.pi  # radians

    x0, y0, z0 = base_pos   # pivot position
    print(f"theta = {theta*180/math.pi}")
    # Ball position:
    x_ball = x0 + L * math.cos(math.pi/2 - theta)
    y_ball = y0
    z_ball = L * math.sin(math.pi/2 - theta)

    time.sleep(dt)

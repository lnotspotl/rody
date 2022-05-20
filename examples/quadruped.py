import sympy
import numpy as np

import Geometry
import Robot
import Plucker

from numpy import pi as PI
from Geometry import rotX, rotY, rotZ, positionVector

def createRobot(name):
    robot = Robot.Robot(name = name,
            gravity = np.array([0,0,0,0,0,-9.81]))

    ###########################################################################
    #                                 WORLD                                   #
    ###########################################################################

    inertia_matrix_w = np.zeros(shape = (3,3))

    # predecessor frame 0 
    pf0_w = Geometry.Frame(
        name = "world",
        rotation_matrix = rotZ(0),
        position_vector = positionVector(0,0,0)
    )

    # list of predecessor frames
    pf_list_w = list()
    pf_list_w.append(pf0_w)

    # successor frame 0
    sf0_w = Geometry.Frame(
            name = "base_link_frame",
            rotation_matrix = rotZ(0),
            position_vector = positionVector(0,0,0)
    )

    # list of successor frames
    sf_list_w = list()
    sf_list_w.append(sf0_w)

    success = robot.addBody(
            name = "WORLD",
            mass = 0,
            inertia_matrix = inertia_matrix_w,
            predecessor_frames = pf_list_w,
            successor_frames = sf_list_w
    )

    if not success:
        exit()

    ###########################################################################
    #                               JOINT1                                    #
    ###########################################################################

    # JOINT 1 -> WORLD ^ BASE LINK
    success = robot.addJoint(
            name = "JOINT1",
            joint_type = "floating",
            frame = sf0_w
    )

    if not success:
        exit()

    ###########################################################################
    #                               BASE LINK                                 #
    ###########################################################################

    # predecessor frame 0 
    pf0_bl = Geometry.Frame(
        name = "base_link_predecessor",
        rotation_matrix = rotZ(0),
        position_vector = positionVector(0,0,0)
    )

    # list of predecessor frames
    pf_list_bl = list()
    pf_list_bl.append(pf0_bl)

    # successor frame 0
    sf_bl_FR = Geometry.Frame(
            name = "FR1",
            rotation_matrix = rotX(PI / 2) @ rotY(-PI / 2),
            position_vector = positionVector(+1,-1,0)
    )
    sf_bl_FL = Geometry.Frame(
            name = "FL1",
            rotation_matrix = rotX(PI / 2) @ rotY(-PI / 2),
            position_vector = positionVector(+1,+1,0)
    )
    sf_bl_RR = Geometry.Frame(
            name = "RR1",
            rotation_matrix = rotX(PI / 2) @ rotY(-PI / 2),
            position_vector = positionVector(-1,-1,0)
    )
    sf_bl_RL = Geometry.Frame(
            name = "RL1",
            rotation_matrix = rotX(PI / 2) @ rotY(-PI / 2),
            position_vector = positionVector(-1,+1,0)
    )

    # list of successor frames
    sf_list_bl = list()
    sf_list_bl.append(sf_bl_FR)
    sf_list_bl.append(sf_bl_FL)
    sf_list_bl.append(sf_bl_RR)
    sf_list_bl.append(sf_bl_RL)

    inertia_matrix_bl = np.eye(3)
    success = robot.addBody(
            name = "BASE LINK",
            mass = 12,
            inertia_matrix = inertia_matrix_bl,
            predecessor_frames = pf_list_bl,
            successor_frames = sf_list_bl
    )

    if not success:
        exit()

    ###########################################################################
    #                               JOINT2                                    #
    ###########################################################################

    # JOINT 2 -> BASE LINK ^ FR1
    success = robot.addJoint(
            name = "JOINT2",
            joint_type = "revolute",
            frame = sf_bl_FR
    )

    if not success:
        exit()

    ###########################################################################
    #                                BODY2                                    #
    ###########################################################################

    pf_b2 = Geometry.Frame(
        name = "FR1_joint",
        rotation_matrix = rotZ(0),
        position_vector = positionVector(-0.5,0,0)
    )

    pf_list_b2 = list()
    pf_list_b2.append(pf_b2)

    sf_b2 = Geometry.Frame(
            name = "FR2_joint",
            rotation_matrix = rotX(PI / 2) @ rotY(-PI / 2),
            position_vector = positionVector(0.5,0,0)
    )

    # list of successor frames
    sf_list_b2 = list()
    sf_list_b2.append(sf_b2)

    inertia_matrix_b2 = np.eye(3)
    success = robot.addBody(
            name = "FR1",
            mass = 1,
            inertia_matrix = inertia_matrix_b2,
            predecessor_frames = pf_list_b2,
            successor_frames = sf_list_b2
    )

    if not success:
        exit()

    ###########################################################################
    #                               JOINT3                                    #
    ###########################################################################

    # JOINT 2 -> FR1 ^ FR2
    success = robot.addJoint(
            name = "JOINT3",
            joint_type = "revolute",
            frame = sf_b2
    )

    if not success:
        exit()

    ###########################################################################
    #                                BODY3                                    #
    ###########################################################################

    pf_b3 = Geometry.Frame(
        name = "FR2_joint",
        rotation_matrix = rotZ(0),
        position_vector = positionVector(-0.5,0,0)
    )

    pf_list_b3 = list()
    pf_list_b3.append(pf_b3)

    sf_b3 = Geometry.Frame(
            name = "FR3_joint",
            rotation_matrix = rotZ(0),
            position_vector = positionVector(0.5,0,0)
    )

    # list of successor frames
    sf_list_b3 = list()
    sf_list_b3.append(sf_b3)

    inertia_matrix_b3 = np.eye(3)
    success = robot.addBody(
            name = "FR2",
            mass = 1,
            inertia_matrix = inertia_matrix_b3,
            predecessor_frames = pf_list_b3,
            successor_frames = sf_list_b3
    )

    if not success:
        exit()

    ###########################################################################
    #                               JOINT4                                    #
    ###########################################################################

    # JOINT 4 -> FR2 ^ FR3
    success = robot.addJoint(
            name = "JOINT4",
            joint_type = "revolute",
            frame = sf_b3
    )

    if not success:
        exit()

    ###########################################################################
    #                                BODY4                                    #
    ###########################################################################

    pf_b4 = Geometry.Frame(
        name = "FR3_joint",
        rotation_matrix = rotZ(0),
        position_vector = positionVector(-0.5,0,0)
    )

    pf_list_b4 = list()
    pf_list_b4.append(pf_b4)

    sf_b4 = Geometry.Frame(
            name = "DUMMY1_JOINT",
            rotation_matrix = rotZ(0),
            position_vector = positionVector(0.5,0,0)
    )

    # list of successor frames
    sf_list_b4 = list()
    sf_list_b4.append(sf_b4)

    inertia_matrix_b4 = np.eye(3)
    success = robot.addBody(
            name = "FR3",
            mass = 1,
            inertia_matrix = inertia_matrix_b4,
            predecessor_frames = pf_list_b4,
            successor_frames = sf_list_b4
    )

    if not success:
        exit()

    ###########################################################################
    #                               JOINT5                                    #
    ###########################################################################

    # JOINT 5 -> FR3 ^ DUMMY1
    success = robot.addJoint(
            name = "JOINT5",
            joint_type = "rigid",
            frame = sf_b4
    )

    if not success:
        exit()

    ###########################################################################
    #                                BODY5                                    #
    ###########################################################################

    pf_b5 = Geometry.Frame(
        name = "FR_RIGID_JOINT",
        rotation_matrix = rotZ(0),
        position_vector = positionVector(0,0,0)
    )

    pf_list_b5 = list()
    pf_list_b5.append(pf_b5)

    sf_b5 = Geometry.Frame(
            name = "FR4",
            rotation_matrix = rotZ(0),
            position_vector = positionVector(0,0,0)
    )

    # list of successor frames
    sf_list_b5 = list()
    sf_list_b5.append(sf_b5)

    inertia_matrix_b5 = np.zeros(shape = (3,3))
    success = robot.addBody(
            name = "DUMMY1",
            mass = 0,
            inertia_matrix = inertia_matrix_b5,
            predecessor_frames = pf_list_b5,
            successor_frames = sf_list_b5,
    )

    if not success:
        exit()

    ###########################################################################
    #                               JOINT6                                    #
    ###########################################################################

    # JOINT 6 -> BASE LINK ^ FL1
    success = robot.addJoint(
            name = "JOINT6",
            joint_type = "revolute",
            frame = sf_bl_FL
    )

    if not success:
        exit()

    ###########################################################################
    #                                BODY6                                    #
    ###########################################################################

    pf_b6 = Geometry.Frame(
        name = "FL1_joint",
        rotation_matrix = rotZ(0),
        position_vector = positionVector(0.5,0,0)
    )

    pf_list_b6 = list()
    pf_list_b6.append(pf_b6)

    sf_b6 = Geometry.Frame(
            name = "FL2_joint",
            rotation_matrix = rotX(PI / 2) @ rotY(-PI / 2),
            position_vector = positionVector(-0.5,0,0)
    )

    # list of successor frames
    sf_list_b6 = list()
    sf_list_b6.append(sf_b6)

    inertia_matrix_b6 = np.eye(3)
    success = robot.addBody(
            name = "FL1",
            mass = 1,
            inertia_matrix = inertia_matrix_b6,
            predecessor_frames = pf_list_b6,
            successor_frames = sf_list_b6,
            parent = "BASE LINK"
    )

    if not success:
        exit()

    ###########################################################################
    #                               JOINT7                                    #
    ###########################################################################

    # JOINT 7 -> FL1 ^ FL2
    success = robot.addJoint(
            name = "JOINT7",
            joint_type = "revolute",
            frame = sf_b6
    )

    if not success:
        exit()

    ###########################################################################
    #                                BODY7                                    #
    ###########################################################################

    pf_b7 = Geometry.Frame(
        name = "FL2_joint",
        rotation_matrix = rotZ(0),
        position_vector = positionVector(-0.5,0,0)
    )

    pf_list_b7 = list()
    pf_list_b7.append(pf_b7)

    sf_b7 = Geometry.Frame(
            name = "FL3_joint",
            rotation_matrix = rotZ(0),
            position_vector = positionVector(0.5,0,0)
    )

    # list of successor frames
    sf_list_b7 = list()
    sf_list_b7.append(sf_b7)

    inertia_matrix_b7 = np.eye(3)
    success = robot.addBody(
            name = "FL2",
            mass = 1,
            inertia_matrix = inertia_matrix_b7,
            predecessor_frames = pf_list_b7,
            successor_frames = sf_list_b7
    )

    if not success:
        exit()

    ###########################################################################
    #                               JOINT8                                    #
    ###########################################################################

    # JOINT 8 -> FL2 ^ FL3
    success = robot.addJoint(
            name = "JOINT8",
            joint_type = "revolute",
            frame = sf_b7
    )

    if not success:
        exit()

    ###########################################################################
    #                                BODY8                                    #
    ###########################################################################

    pf_b8 = Geometry.Frame(
        name = "FL3_joint",
        rotation_matrix = rotZ(0),
        position_vector = positionVector(-0.5,0,0)
    )

    pf_list_b8 = list()
    pf_list_b8.append(pf_b8)

    sf_b8 = Geometry.Frame(
            name = "Dummy2_joint",
            rotation_matrix = rotZ(0),
            position_vector = positionVector(0.5,0,0)
    )

    # list of successor frames
    sf_list_b8 = list()
    sf_list_b8.append(sf_b8)

    inertia_matrix_b8 = np.eye(3)
    success = robot.addBody(
            name = "FL3",
            mass = 1,
            inertia_matrix = inertia_matrix_b8,
            predecessor_frames = pf_list_b8,
            successor_frames = sf_list_b8
    )

    if not success:
        exit()

    ###########################################################################
    #                               JOINT9                                    #
    ###########################################################################

    # JOINT 9 -> FL3 ^ DUMMY2
    success = robot.addJoint(
            name = "JOINT9",
            joint_type = "rigid",
            frame = sf_b8
    )

    if not success:
        exit()

    ###########################################################################
    #                                BODY9                                    #
    ###########################################################################

    pf_b9 = Geometry.Frame(
        name = "FL_RIGID_JOINT",
        rotation_matrix = rotZ(0),
        position_vector = positionVector(0,0,0)
    )

    pf_list_b9 = list()
    pf_list_b9.append(pf_b9)

    sf_b9 = Geometry.Frame(
            name = "Dummy2_joint",
            rotation_matrix = rotZ(0),
            position_vector = positionVector(0,0,0)
    )

    # list of successor frames
    sf_list_b9 = list()
    sf_list_b9.append(sf_b9)

    inertia_matrix_b9 = np.zeros(shape = (3,3))
    success = robot.addBody(
            name = "DUMMY2",
            mass = 0,
            inertia_matrix = inertia_matrix_b9,
            predecessor_frames = pf_list_b9,
            successor_frames = sf_list_b9
    )

    if not success:
        exit()

    ###########################################################################
    #                               JOINT10                                   #
    ###########################################################################

    # JOINT 10 -> BASE LINK ^ RR1
    success = robot.addJoint(
            name = "JOINT10",
            joint_type = "revolute",
            frame = sf_bl_RR
    )

    if not success:
        exit()

    ###########################################################################
    #                                BODY10                                    #
    ###########################################################################

    pf_b10 = Geometry.Frame(
        name = "RR1_joint",
        rotation_matrix = rotZ(0),
        position_vector = positionVector(-0.5,0,0)
    )

    pf_list_b10 = list()
    pf_list_b10.append(pf_b10)

    sf_b10 = Geometry.Frame(
            name = "RR2_joint",
            rotation_matrix = rotX(PI / 2) @ rotY(-PI / 2),
            position_vector = positionVector(0.5,0,0)
    )

    # list of successor frames
    sf_list_b10 = list()
    sf_list_b10.append(sf_b10)

    inertia_matrix_b10 = np.eye(3)
    success = robot.addBody(
            name = "RR1",
            mass = 1,
            inertia_matrix = inertia_matrix_b10,
            predecessor_frames = pf_list_b10,
            successor_frames = sf_list_b10,
            parent = "BASE LINK"
    )

    if not success:
        exit()

    ###########################################################################
    #                               JOINT11                                   #
    ###########################################################################

    # JOINT 11 -> RR1 ^ RR2
    success = robot.addJoint(
            name = "JOINT11",
            joint_type = "revolute",
            frame = sf_b10
    )

    if not success:
        exit()

    ###########################################################################
    #                                BODY11                                   #
    ###########################################################################

    pf_b11 = Geometry.Frame(
        name = "RR2_joint",
        rotation_matrix = rotZ(0),
        position_vector = positionVector(-0.5,0,0)
    )

    pf_list_b11 = list()
    pf_list_b11.append(pf_b11)

    sf_b11 = Geometry.Frame(
            name = "RR3_joint",
            rotation_matrix = rotZ(0),
            position_vector = positionVector(0.5,0,0)
    )

    # list of successor frames
    sf_list_b11 = list()
    sf_list_b11.append(sf_b11)

    inertia_matrix_b11 = np.eye(3)
    success = robot.addBody(
            name = "RR2",
            mass = 1,
            inertia_matrix = inertia_matrix_b11,
            predecessor_frames = pf_list_b11,
            successor_frames = sf_list_b11
    )

    if not success:
        exit()

    ###########################################################################
    #                               JOINT12                                   #
    ###########################################################################

    # JOINT 12 -> RR2 ^ RR3
    success = robot.addJoint(
            name = "JOINT12",
            joint_type = "revolute",
            frame = sf_b11
    )

    if not success:
        exit()

    ###########################################################################
    #                                BODY12                                   #
    ###########################################################################

    pf_b12 = Geometry.Frame(
        name = "RR3_joint",
        rotation_matrix = rotZ(0),
        position_vector = positionVector(-0.5,0,0)
    )

    pf_list_b12 = list()
    pf_list_b12.append(pf_b12)

    sf_b12 = Geometry.Frame(
            name = "DUMMY3_JOINT",
            rotation_matrix = rotZ(0),
            position_vector = positionVector(0.5,0,0)
    )

    # list of successor frames
    sf_list_b12 = list()
    sf_list_b12.append(sf_b12)

    inertia_matrix_b12 = np.eye(3)
    success = robot.addBody(
            name = "RR3",
            mass = 1,
            inertia_matrix = inertia_matrix_b12,
            predecessor_frames = pf_list_b12,
            successor_frames = sf_list_b12
    )

    if not success:
        exit()

    ###########################################################################
    #                               JOINT13                                   #
    ###########################################################################

    # JOINT 13 -> RR3 ^ DUMMY3
    success = robot.addJoint(
            name = "JOINT13",
            joint_type = "rigid",
            frame = sf_b12
    )

    if not success:
        exit()

    ###########################################################################
    #                                BODY13                                   #
    ###########################################################################

    pf_b13 = Geometry.Frame(
        name = "RR_RIGID_JOINT",
        rotation_matrix = rotZ(0),
        position_vector = positionVector(0,0,0)
    )

    pf_list_b13 = list()
    pf_list_b13.append(pf_b13)

    sf_b13 = Geometry.Frame(
            name = "RR4",
            rotation_matrix = rotZ(0),
            position_vector = positionVector(0,0,0)
    )

    # list of successor frames
    sf_list_b13 = list()
    sf_list_b13.append(sf_b13)

    inertia_matrix_b13 = np.zeros(shape = (3,3))
    success = robot.addBody(
            name = "DUMMY3",
            mass = 0,
            inertia_matrix = inertia_matrix_b13,
            predecessor_frames = pf_list_b13,
            successor_frames = sf_list_b13,
    )

    if not success:
        exit()

    ###########################################################################
    #                               JOINT14                                   #
    ###########################################################################

    # JOINT 14 -> BASE LINK ^ RL1
    success = robot.addJoint(
            name = "JOINT14",
            joint_type = "revolute",
            frame = sf_bl_RL
    )

    if not success:
        exit()

    ###########################################################################
    #                                BODY14                                    #
    ###########################################################################

    pf_b14 = Geometry.Frame(
        name = "RL1_joint",
        rotation_matrix = rotZ(0),
        position_vector = positionVector(0.5,0,0)
    )

    pf_list_b14 = list()
    pf_list_b14.append(pf_b14)

    sf_b14 = Geometry.Frame(
            name = "RL2_joint",
            rotation_matrix = rotX(PI / 2) @ rotY(-PI / 2),
            position_vector = positionVector(-0.5,0,0)
    )

    # list of successor frames
    sf_list_b14 = list()
    sf_list_b14.append(sf_b14)

    inertia_matrix_b14 = np.eye(3)
    success = robot.addBody(
            name = "RL1",
            mass = 1,
            inertia_matrix = inertia_matrix_b14,
            predecessor_frames = pf_list_b14,
            successor_frames = sf_list_b14,
            parent = "BASE LINK"
    )

    if not success:
        exit()

    ###########################################################################
    #                               JOINT15                                   #
    ###########################################################################

    # JOINT 15 -> RL1 ^ RL2
    success = robot.addJoint(
            name = "JOINT15",
            joint_type = "revolute",
            frame = sf_b14
    )

    if not success:
        exit()

    ###########################################################################
    #                                BODY15                                   #
    ###########################################################################

    pf_b15 = Geometry.Frame(
        name = "RL2_joint",
        rotation_matrix = rotZ(0),
        position_vector = positionVector(-0.5,0,0)
    )

    pf_list_b15 = list()
    pf_list_b15.append(pf_b15)

    sf_b15 = Geometry.Frame(
            name = "RL3_joint",
            rotation_matrix = rotZ(0),
            position_vector = positionVector(0.5,0,0)
    )

    # list of successor frames
    sf_list_b15 = list()
    sf_list_b15.append(sf_b15)

    inertia_matrix_b15 = np.eye(3)
    success = robot.addBody(
            name = "RL2",
            mass = 1,
            inertia_matrix = inertia_matrix_b15,
            predecessor_frames = pf_list_b15,
            successor_frames = sf_list_b15
    )

    if not success:
        exit()

    ###########################################################################
    #                               JOINT16                                   #
    ###########################################################################

    # JOINT 16 -> RL2 ^ RL3
    success = robot.addJoint(
            name = "JOINT16",
            joint_type = "revolute",
            frame = sf_b15
    )

    if not success:
        exit()

    ###########################################################################
    #                                BODY16                                    #
    ###########################################################################

    pf_b16 = Geometry.Frame(
        name = "RL3_joint",
        rotation_matrix = rotZ(0),
        position_vector = positionVector(-0.5,0,0)
    )

    pf_list_b16 = list()
    pf_list_b16.append(pf_b16)

    sf_b16 = Geometry.Frame(
            name = "Dummy4_joint",
            rotation_matrix = rotZ(0),
            position_vector = positionVector(0.5,0,0)
    )

    # list of successor frames
    sf_list_b16 = list()
    sf_list_b16.append(sf_b16)

    inertia_matrix_b16 = np.eye(3)
    success = robot.addBody(
            name = "RL3",
            mass = 1,
            inertia_matrix = inertia_matrix_b16,
            predecessor_frames = pf_list_b16,
            successor_frames = sf_list_b16
    )

    if not success:
        exit()

    ###########################################################################
    #                               JOINT17                                   #
    ###########################################################################

    # JOINT 17 -> RL3 ^ DUMMY4
    success = robot.addJoint(
            name = "JOINT17",
            joint_type = "rigid",
            frame = sf_b16
    )

    if not success:
        exit()

    ###########################################################################
    #                                BODY17                                   #
    ###########################################################################

    pf_b17 = Geometry.Frame(
        name = "RL_RIGID_JOINT",
        rotation_matrix = rotZ(0),
        position_vector = positionVector(0,0,0)
    )

    pf_list_b17 = list()
    pf_list_b17.append(pf_b17)

    sf_b17 = Geometry.Frame(
            name = "Dummy4_joint",
            rotation_matrix = rotZ(0),
            position_vector = positionVector(0,0,0)
    )

    # list of successor frames
    sf_list_b17 = list()
    sf_list_b17.append(sf_b17)

    inertia_matrix_b17 = np.zeros(shape = (3,3))
    success = robot.addBody(
            name = "DUMMY4",
            mass = 0,
            inertia_matrix = inertia_matrix_b17,
            predecessor_frames = pf_list_b17,
            successor_frames = sf_list_b17
    )

    if not success:
        exit()

    return robot

def main():
    robot = createRobot(name = "quadruped")

    # create a simple graph of the robot
    robot.generateGraph("robot_quadruped.png")
    robot.summary()

    joint_variables = dict()
    joint_variables["JOINT1"] = {"q" : [0,0,0,0,0,0], 
                                 "q_dot" : [0,0,0,0,0,0],
                                 "q_ddot" : [0,0,0,0,0,0]}
    joint_variables["JOINT2"] = {"q" : [0], 
                                 "q_dot" : [0],
                                 "q_ddot" : [0]}
    joint_variables["JOINT3"] = {"q" : [0], 
                                 "q_dot" : [0],
                                 "q_ddot" : [0]}
    joint_variables["JOINT4"] = {"q" : [0], 
                                 "q_dot" : [0],
                                 "q_ddot" : [0]}
    joint_variables["JOINT6"] = {"q" : [0], 
                                 "q_dot" : [0],
                                 "q_ddot" : [0]}
    joint_variables["JOINT7"] = {"q" : [0], 
                                 "q_dot" : [0],
                                 "q_ddot" : [0]}
    joint_variables["JOINT8"] = {"q" : [0], 
                                 "q_dot" : [0],
                                 "q_ddot" : [0]}
    joint_variables["JOINT10"] = {"q" : [0], 
                                 "q_dot" : [0],
                                 "q_ddot" : [0]}
    joint_variables["JOINT11"] = {"q" : [0], 
                                 "q_dot" : [0],
                                 "q_ddot" : [0]}
    joint_variables["JOINT12"] = {"q" : [0], 
                                 "q_dot" : [0],
                                 "q_ddot" : [0]}
    joint_variables["JOINT14"] = {"q" : [0], 
                                 "q_dot" : [0],
                                 "q_ddot" : [0]}
    joint_variables["JOINT15"] = {"q" : [0], 
                                 "q_dot" : [0],
                                 "q_ddot" : [0]}
    joint_variables["JOINT16"] = {"q" : [0], 
                                 "q_dot" : [0],
                                 "q_ddot" : [0]}
    robot.setJointVariables(joint_variables)

    # BIAS FORCE VECTOR
    bias_force_vector = robot.BiasForceVector(set_to_symbolic = False)
    print(f"bias force vector: {bias_force_vector}")
    print()

    # MASS MATRIX
    mass_matrix = robot.MassMatrix(set_to_symbolic = True)
    for row in mass_matrix:
        print(row)
    print(mass_matrix)

if __name__ == "__main__":
    main()

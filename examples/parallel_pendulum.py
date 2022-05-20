import sympy
import numpy as np

import Geometry
import Robot
import Plucker

from Geometry import rotX, rotY, rotZ, positionVector

def createRobot(name):
    robot = Robot.Robot(name = name)

    ###########################################################################
    #                               BASE LINK                                 #
    ###########################################################################
    inertia_matrix_bl = np.zeros(shape = (3,3))

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
    sf0_bl = Geometry.Frame(
            name = "joint1_frame",
            rotation_matrix = rotZ(0),
            position_vector = positionVector(0,0,0)
    )

    # list of successor frames
    sf_list_bl = list()
    sf_list_bl.append(sf0_bl)

    success = robot.addBody(
            name = "BASE LINK",
            mass = 0,
            inertia_matrix = inertia_matrix_bl,
            predecessor_frames = pf_list_bl,
            successor_frames = sf_list_bl
    )

    if not success:
        exit()

    ###########################################################################
    #                               JOINT1                                    #
    ###########################################################################

    # JOINT 1 -> BASE LINK ^ BODY 1
    success = robot.addJoint(
            name = "JOINT1",
            joint_type = "revolute",
            frame = sf0_bl
    )

    if not success:
        exit()

    ###########################################################################
    #                                BODY1                                    #
    ###########################################################################

    ### BODY 1
    inertia_matrix_b1 = np.zeros(shape = (3,3))

    # predecessor frame 0 
    pf0_b1 = Geometry.Frame(
        name = "body1_predecessor",
        rotation_matrix = rotZ(0),
        position_vector = positionVector(-1.0,0,0)
    )

    # list of predecessor frames
    pf_list_b1 = list()
    pf_list_b1.append(pf0_b1)

    # successor frame 0
    sf0_b1 = Geometry.Frame(
            name = "joint2_frame",
            rotation_matrix = rotZ(0),
            position_vector = positionVector(0,0,0)
    )

    # list of successor frames
    sf_list_b1 = list()
    sf_list_b1.append(sf0_b1)

    success = robot.addBody(
            name = "BODY1",
            mass = 1,
            inertia_matrix = inertia_matrix_b1,
            predecessor_frames = pf_list_b1,
            successor_frames = sf_list_b1
    )

    if not success:
        exit()

    ###########################################################################
    #                               JOINT2                                    #
    ###########################################################################

    ### JOINT 2 -> body1 ^ body2
    success = robot.addJoint(
            name = "JOINT2",
            joint_type = "revolute",
            frame = sf0_b1
    )

    if not success:
        exit()

    ###########################################################################
    #                                BODY2                                    #
    ###########################################################################

    ### BODY 2
    inertia_matrix_b2 = np.zeros(shape = (3,3))

    # predecessor frame 0 
    pf0_b2 = Geometry.Frame(
        name = "body2_predecessor",
        rotation_matrix = rotZ(0),
        position_vector = positionVector(-1.0,0,0)
    )

    # list of predecessor frames
    pf_list_b2 = list()
    pf_list_b2.append(pf0_b2)

    # successor frame 0
    sf0_b2 = Geometry.Frame(
            name = "joint3_frame",
            rotation_matrix = rotZ(0),
            position_vector = positionVector(0,0,0)
    )

    # list of successor frames
    sf_list_b2 = list()
    sf_list_b2.append(sf0_b2)

    success = robot.addBody(
            name = "BODY2",
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

    ### JOINT 3 -> body2 ^ dummy3
    success = robot.addJoint(
            name = "JOINT3",
            joint_type = "rigid",
            frame = sf0_b2
    )

    if not success:
        exit()

    ###########################################################################
    #                                DUMMY1                                   #
    ###########################################################################

    inertia_matrix_d1 = np.zeros(shape = (3,3))

    # predecessor frame 0 
    pf0_d1 = Geometry.Frame(
        name = "dummy1_predecessor",
        rotation_matrix = rotZ(0),
        position_vector = positionVector(-1.0,0,0)
    )

    # list of predecessor frames
    pf_list_d1 = list()
    pf_list_d1.append(pf0_d1)

    # successor frame 0
    sf0_d1 = Geometry.Frame(
            name = "end_effector_frame",
            rotation_matrix = rotZ(0),
            position_vector = positionVector(0,0,0)
    )

    # list of successor frames
    sf_list_d1 = list()
    sf_list_d1.append(sf0_d1)

    success = robot.addBody(
            name = "DUMMY1",
            mass = 0,
            inertia_matrix = inertia_matrix_d1,
            predecessor_frames = pf_list_d1,
            successor_frames = sf_list_d1
    )

    if not success:
        exit()


    ###########################################################################
    #                               JOINT4                                    #
    ###########################################################################

    # JOINT 1 -> BASE LINK ^ BODY 4
    success = robot.addJoint(
            name = "JOINT4",
            joint_type = "revolute",
            frame = sf0_bl
    )

    if not success:
        exit()

    ###########################################################################
    #                                BODY4                                    #
    ###########################################################################

    inertia_matrix_b4 = np.zeros(shape = (3,3))

    # predecessor frame 0 
    pf0_b4 = Geometry.Frame(
        name = "body4_predecessor",
        rotation_matrix = rotZ(0),
        position_vector = positionVector(-1.0,0,0)
    )

    # list of predecessor frames
    pf_list_b4 = list()
    pf_list_b4.append(pf0_b4)

    # successor frame 0
    sf0_b4 = Geometry.Frame(
            name = "joint5_frame",
            rotation_matrix = rotZ(0),
            position_vector = positionVector(0,0,0)
    )

    # list of successor frames
    sf_list_b4 = list()
    sf_list_b4.append(sf0_b4)

    success = robot.addBody(
            name = "BODY4",
            mass = 1,
            inertia_matrix = inertia_matrix_b4,
            predecessor_frames = pf_list_b4,
            successor_frames = sf_list_b4,
            parent = "BASE LINK"
    )

    if not success:
        exit()

    ###########################################################################
    #                               JOINT5                                    #
    ###########################################################################

    # JOINT 1 -> BODY 4 ^ BODY 5
    success = robot.addJoint(
            name = "JOINT5",
            joint_type = "revolute",
            frame = sf0_b4
    )

    if not success:
        exit()

    ###########################################################################
    #                                BODY5                                    #
    ###########################################################################
    inertia_matrix_b5 = np.zeros(shape = (3,3))

    # predecessor frame 0 
    pf0_b5 = Geometry.Frame(
        name = "body1_predecessor",
        rotation_matrix = rotZ(0),
        position_vector = positionVector(-1.0,0,0)
    )

    # list of predecessor frames
    pf_list_b5 = list()
    pf_list_b5.append(pf0_b5)

    # successor frame 0
    sf0_b5 = Geometry.Frame(
            name = "joint6_frame",
            rotation_matrix = rotZ(0),
            position_vector = positionVector(0,0,0)
    )

    # list of successor frames
    sf_list_b5 = list()
    sf_list_b5.append(sf0_b5)

    success = robot.addBody(
            name = "BODY5",
            mass = 1,
            inertia_matrix = inertia_matrix_b5,
            predecessor_frames = pf_list_b5,
            successor_frames = sf_list_b5
    )

    ###########################################################################
    #                               JOINT6                                    #
    ###########################################################################

    ### JOINT 6 -> BODY5 ^ DUMMY2
    success = robot.addJoint(
            name = "JOINT6",
            joint_type = "rigid",
            frame = sf0_b5
    )

    if not success:
        exit()

    ###########################################################################
    #                                DUMMY2                                   #
    ###########################################################################

    inertia_matrix_d2 = np.zeros(shape = (3,3))

    # predecessor frame 0 
    pf0_d2 = Geometry.Frame(
        name = "dummy1_predecessor",
        rotation_matrix = rotZ(0),
        position_vector = positionVector(-1.0,0,0)
    )

    # list of predecessor frames
    pf_list_d2 = list()
    pf_list_d2.append(pf0_d2)

    # successor frame 0
    sf0_d2 = Geometry.Frame(
            name = "end_effector_frame",
            rotation_matrix = rotZ(0),
            position_vector = positionVector(0,0,0)
    )

    # list of successor frames
    sf_list_d2 = list()
    sf_list_d2.append(sf0_d2)

    success = robot.addBody(
            name = "DUMMY2",
            mass = 0,
            inertia_matrix = inertia_matrix_d2,
            predecessor_frames = pf_list_d2,
            successor_frames = sf_list_d2
    )

    if not success:
        exit()


    return robot

def main():
    robot = createRobot(name = "parallel_pendulum")

    robot.generateGraph("robot_parallel_pendulum.png")

    # print a short summary of the robot's structure, joints and bodies
    robot.summary()

    joint_variables = dict()
    joint_variables["JOINT1"] = {"q" : [0], 
                                 "q_dot" : [0],
                                 "q_ddot" : [0]}

    joint_variables["JOINT2"] = {"q" : [0], 
                                 "q_dot" : [0],
                                 "q_ddot" : [0]}

    joint_variables["JOINT4"] = {"q" : [0], 
                                 "q_dot" : [0],
                                 "q_ddot" : [0]}

    joint_variables["JOINT5"] = {"q" : [0], 
                                 "q_dot" : [0],
                                 "q_ddot" : [0]}

    robot.setJointVariables(joint_variables)
    # compute robot's bias force vector
    bias_force_vector = robot.BiasForceVector(set_to_symbolic = False)
    print(f"bias force vector: {bias_force_vector}")
    print()

    # compute robot's mass matrix
    mass_matrix = robot.MassMatrix(set_to_symbolic = False)
    print(mass_matrix)

if __name__ == "__main__":
    main()

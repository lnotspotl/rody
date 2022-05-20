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
            joint_type = "floating",
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
        position_vector = positionVector(0,0,0)
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

    return robot

def main():
    # create robot object
    robot = createRobot(name = "floating_base")

    # generate simple graph that shows the structure of the robot
    # robot.generateGraph("floating_mass_point.png")

    # print a short summary of the robot's structure, joints and bodies
    robot.summary()

    joint_variables = dict()
    joint_variables["JOINT1"] = {"q"      : [0,0,0,0,0,0], 
                                 "q_dot"  : [0,0,0,0,0,0],
                                 "q_ddot" : [0,0,0,0,0,0]}

    robot.setJointVariables(joint_variables)

    # compute robot's mass matrix
    mass_matrix = robot.MassMatrix(set_to_symbolic = False)
    for i in range(len(mass_matrix)):
        for j in range(len(mass_matrix)):
            mass_matrix[i][j] = int(mass_matrix[i][j])
    print("mass matrix:")
    for row in mass_matrix:
        print(row)

    # compute robot's bias force vector
    bias_force_vector = robot.BiasForceVector(set_to_symbolic = False)
    for i in range(len(bias_force_vector)):
        bias_force_vector[i] = sympy.simplify(bias_force_vector[i])
    print(f"bias force vector: {bias_force_vector}")
    print()

    # compute inverse dynamics
    generalized_forces = robot.newtonEulerID()
    print(f"joint generalized forces: {generalized_forces}")

if __name__ == "__main__":
    main()

import sympy
import numpy as np

import Joints
import Geometry

from Geometry import rotZ, positionVector

###############################################################################

def test_Joint_1():
    pf = Geometry.Frame(
        name = "frame",
        rotation_matrix = rotZ(0),
        position_vector = positionVector(0,0,0)
    )

    joint = Joints.Joint(
            name = "test_joint",
            frame = pf,
            reset_counter = True)
    assert joint.joint_number() == 1

def test_Joint_2():
    pf = Geometry.Frame(
        name = "frame",
        rotation_matrix = rotZ(0),
        position_vector = positionVector(0,0,0)
    )

    joint = Joints.Joint(
            name = "test_joint",
            frame = pf,
            reset_counter = True)
    for i in range(19):
        joint = Joints.Joint(
            name = "joint" + str(i + 1),
            frame = pf, 
            reset_counter = False
        )
    last_joint = Joints.Joint(
            name = "last_joint",
            frame = pf,
            reset_counter = False
        )
    assert last_joint.joint_number() == 21

###############################################################################

def test_RevoluteJoint():
    pf = Geometry.Frame(
        name = "frame",
        rotation_matrix = rotZ(0),
        position_vector = positionVector(0,0,0)
    )
    revolute_joint = Joints.RevoluteJoint(
            "rev_joint_test",
            frame = pf,
            reset_counter = True
        )

    assert revolute_joint.joint_type() == "revolute"

    # list of controllable parameters
    controllable_parameters = revolute_joint.getControllableParameters()
    assert len(controllable_parameters) == 1
    assert str(controllable_parameters[0]) == "q1"
    
    # relative velocity of body i + 1 with respect to body i
    v_j = revolute_joint.v_j()
    print(v_j)
    assert str(v_j[0]) == "0"
    assert str(v_j[1]) == "0"
    assert str(v_j[2]) == "q1_dot"
    assert str(v_j[3]) == "0"
    assert str(v_j[4]) == "0"
    assert str(v_j[5]) == "0"

    # set specific joint variable value
    revolute_joint.setJointVariables(q_dot = [1])
    v_j = revolute_joint.v_j()
    assert str(v_j[0]) == "0"
    assert str(v_j[1]) == "0"
    assert str(v_j[2]) == "1"
    assert str(v_j[3]) == "0"
    assert str(v_j[4]) == "0"
    assert str(v_j[5]) == "0"

    # reset joint variables, set them to their symbolic representations
    revolute_joint.resetJointVariables()
    v_j = revolute_joint.v_j()
    assert str(v_j[0]) == "0"
    assert str(v_j[1]) == "0"
    assert str(v_j[2]) == "q1_dot"
    assert str(v_j[3]) == "0"
    assert str(v_j[4]) == "0"
    assert str(v_j[5]) == "0"

###############################################################################

def test_PrismaticJoint():
    pf = Geometry.Frame(
        name = "frame",
        rotation_matrix = rotZ(0),
        position_vector = positionVector(0,0,0)
    )
    prismatic_joint = Joints.PrismaticJoint(
            name = "prism_joint_test",
            frame = pf,
            reset_counter = True
        )

    assert prismatic_joint.joint_type() == "prismatic"

    # list of controllable parameters
    controllable_parameters = prismatic_joint.getControllableParameters()
    assert len(controllable_parameters) == 1
    assert str(controllable_parameters[0]) == "d1"
    
    # relative velocity of body i + 1 with respect to body i
    v_j = prismatic_joint.v_j()
    assert str(v_j[0]) == "0"
    assert str(v_j[1]) == "0"
    assert str(v_j[2]) == "0"
    assert str(v_j[3]) == "0"
    assert str(v_j[4]) == "0"
    assert str(v_j[5]) == "d1_dot"

    # set specific joint variable value
    prismatic_joint.setJointVariables(q_dot = [1])
    v_j = prismatic_joint.v_j()
    assert str(v_j[0]) == "0"
    assert str(v_j[1]) == "0"
    assert str(v_j[2]) == "0"
    assert str(v_j[3]) == "0"
    assert str(v_j[4]) == "0"
    assert str(v_j[5]) == "1"

    # reset joint variables, set them to their symbolic representations
    prismatic_joint.resetJointVariables()
    v_j = prismatic_joint.v_j()
    assert str(v_j[0]) == "0"
    assert str(v_j[1]) == "0"
    assert str(v_j[2]) == "0"
    assert str(v_j[3]) == "0"
    assert str(v_j[4]) == "0"
    assert str(v_j[5]) == "d1_dot"

###############################################################################

def test_FloatingJoint():
    pf = Geometry.Frame(
        name = "frame",
        rotation_matrix = rotZ(0),
        position_vector = positionVector(0,0,0)
    )

    floating_joint = Joints.FloatingJoint(
            name = "float_joint_test",
            frame = pf,
            reset_counter = True
        )

    assert floating_joint.joint_type() == "floating"

    # list of controllable parameters
    controllable_parameters = floating_joint.getControllableParameters()
    assert len(controllable_parameters) == 6

    assert str(controllable_parameters[0]) == "re1"
    assert str(controllable_parameters[1]) == "pe1"
    assert str(controllable_parameters[2]) == "ye1"
    assert str(controllable_parameters[3]) == "x1"
    assert str(controllable_parameters[4]) == "y1"
    assert str(controllable_parameters[5]) == "z1"
    
    # set specific joint variable value
    floating_joint.setJointVariables(q = [0,0,0,0,0,0], q_dot = [1,2,3,4,5,6])
    v_j = floating_joint.v_j()
    assert v_j[0] == 1
    assert v_j[1] == 2
    assert v_j[2] == 3
    assert v_j[3] == 4
    assert v_j[4] == 5
    assert v_j[5] == 6

###############################################################################

def test_RigidJoint():
    pf = Geometry.Frame(
        name = "frame",
        rotation_matrix = rotZ(0),
        position_vector = positionVector(0,0,0)
    )
    rigid_joint = Joints.RigidJoint(
            name = "rigid_joint_test",
            frame = pf,
            reset_counter = True
        )

    assert rigid_joint.joint_type() == "rigid"

    # list of controllable parameters
    controllable_parameters = rigid_joint.getControllableParameters()
    assert len(controllable_parameters) == 1
    assert str(controllable_parameters[0]) == 'None'

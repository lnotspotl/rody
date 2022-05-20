## Plucker transformation matrices

import Geometry 
import numpy as np

def pluckerMotionTransform(rotation_matrix, position_vector):
    """ Create a 6x6 plucker motion transformation matrix. """

    # final plucker transformation matrix
    plucker_transformation_matrix = np.full(shape = (6,6), fill_value = None)
    plucker_transformation_matrix[::] = 0

    # position cross product matrix
    pos_cpm = Geometry.crossProductMatrix(position_vector)

    # top left-hand side corner
    plucker_transformation_matrix[:3,:3] = rotation_matrix

    # bottom left-hand side corner
    plucker_transformation_matrix[3:,:3] = pos_cpm @ rotation_matrix

    # bottom right-hand side corner
    plucker_transformation_matrix[3:,3:] = rotation_matrix

    return plucker_transformation_matrix

def pluckerForceTransform(rotation_matrix, position_vector):
    """ Create a 6x6 plucker force transformation matrix. """

    # final plucker transformation matrix
    plucker_transformation_matrix = np.full(shape = (6,6), fill_value = None)
    plucker_transformation_matrix[::] = 0

    # position cross product matrix
    pos_cpm = Geometry.crossProductMatrix(position_vector)

    # top left-hand side corner
    plucker_transformation_matrix[:3,:3] = rotation_matrix

    # top right-hand side corner
    plucker_transformation_matrix[:3,3:] = pos_cpm @ rotation_matrix

    # bottom right-hand side corner
    plucker_transformation_matrix[3:,3:] = rotation_matrix

    return plucker_transformation_matrix

def pluckerMotionInverse(plucker_transformation_matrix):
    """ Create a 6x6 inverse motion plucker transformation matrix. """
    rot_matrix = plucker_transformation_matrix[:3,:3]
    rot_matrix_inv = Geometry.rotInverse(rot_matrix)

    # position vector cross product matrix
    pos_cpm = plucker_transformation_matrix[3:,:3] @ rot_matrix_inv

    plucker_transformation_matrix_inv= np.full(shape = (6,6), fill_value = None)
    plucker_transformation_matrix_inv[::] = 0

    # top left-hand side corner
    plucker_transformation_matrix_inv[:3,:3] = rot_matrix_inv

    # bottom left-hand side corner
    plucker_transformation_matrix_inv[3:,:3] = (-1) * (rot_matrix_inv @ \
            pos_cpm)

    # bottom right-hand side corner
    plucker_transformation_matrix_inv[3:,3:] = rot_matrix_inv

    return plucker_transformation_matrix_inv

def pluckerForceInverse(plucker_transformation_matrix):
    """ Create a 6x6 inverse motion plucker transformation matrix. """
    rot_matrix = plucker_transformation_matrix[:3,:3]
    rot_matrix_inv = Geometry.rotInverse(rot_matrix)

    # position vector cross product matrix
    pos_cpm = plucker_transformation_matrix[:3,3:] @ rot_matrix_inv

    plucker_transformation_matrix_inv = np.full(shape = (6,6), fill_value = None)
    plucker_transformation_matrix_inv[::] = 0

    # top left-hand side corner
    plucker_transformation_matrix_inv[:3,:3] = rot_matrix_inv

    # top right-hand side corner
    plucker_transformation_matrix_inv[:3,3:] = (-1) * (rot_matrix_inv @ \
            pos_cpm)

    # bottom right-hand side corner
    plucker_transformation_matrix_inv[3:,3:] = rot_matrix_inv

    return plucker_transformation_matrix_inv

def pluckerForceToMotion(plucker_transformation_matrix):
    """ Convert motion plucker tranformation matrix into its force equivalent. """
    plucker_transformation_matrix_inv = pluckerForceInverse(plucker_transformation_matrix)
    return plucker_transformation_matrix_inv.T

def pluckerMotionToForce(plucker_transformation_matrix):
    """ Convert force plucker tranformation matrix into its motion equivalent. """
    plucker_transformation_matrix_inv = pluckerMotionInverse(plucker_transformation_matrix)
    return plucker_transformation_matrix_inv.T

def pluckerMotionExtractPositionVector(plucker_transformation_matrix):
    """ Extract position vector from plucker motion transformation matrix."""

    # GET ROTATION MATRIX
    rot_matrix = plucker_transformation_matrix[:3,:3]
    rot_matrix_inv = Geometry.rotInverse(rot_matrix)

    # GET POSITION VECTOR CROSS PRODUCT MATRIX
    pos_cpm = plucker_transformation_matrix[3:,:3] @ rot_matrix_inv

    # EXTRACT COORDINATES
    x = pos_cpm[2][1]
    y = pos_cpm[0][2]
    z = pos_cpm[1][0]

    return np.array([x,y,z])

def motionCrossProductMatrix(motion_vector):
    """ Return the motion cross product matrix of a spatial vector. """

    mcp = np.full(shape = (6,6), fill_value = None)
    mcp[::] = 0

    mcp[0, 1] = -motion_vector[2]
    mcp[0, 2] = motion_vector[1]
    mcp[1, 0] = motion_vector[2]
    mcp[1, 2] = -motion_vector[0]
    mcp[2, 0] = -motion_vector[1]
    mcp[2, 1] = motion_vector[0]

    mcp[3, 4] = -motion_vector[2]
    mcp[3, 5] = motion_vector[1]
    mcp[4, 3] = motion_vector[2]
    mcp[4, 5] = -motion_vector[0]
    mcp[5, 3] = -motion_vector[1]
    mcp[5, 4] = motion_vector[0]

    mcp[3, 1] = -motion_vector[5]
    mcp[3, 2] = motion_vector[4]
    mcp[4, 0] = motion_vector[5]
    mcp[4, 2] = -motion_vector[3]
    mcp[5, 0] = -motion_vector[4]
    mcp[5, 1] = motion_vector[3]

    return mcp

def forceCrossProductMatrix(fv):
    """ Return the force cross product matrix of a spatial vector. """
    return -motionCrossProductMatrix(fv).T

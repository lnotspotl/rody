import numpy as np
import sympy

## Rotation matrices
def rotX(alpha):
    """ Generate a 3x3 rotation matrix around the x axis. """
    alpha_type = type(alpha)
    if alpha_type != sympy.core.symbol.Symbol:
        cos = np.cos(alpha)
        sin = np.sin(alpha)
    else:
        cos = sympy.cos(alpha)
        sin = sympy.sin(alpha)

    rotation_matrix = np.array([
        [1,   0,  0  ],
        [0, cos, -sin],
        [0, sin,  cos]
    ])
    return rotation_matrix

def rotY(beta):
    """ Generate a 3x3 rotation matrix around the y axis. """
    beta_type = type(beta)
    if beta_type != sympy.core.symbol.Symbol:
        cos = np.cos(beta)
        sin = np.sin(beta)
    else:
        cos = sympy.cos(beta)
        sin = sympy.sin(beta)

    rotation_matrix = np.array([
        [ cos, 0,  sin],
        [ 0,   1,    0],
        [-sin, 0,  cos]
    ])
    return rotation_matrix

def rotZ(gamma):
    """ Generate a 3x3 rotation matrix around the z axis. """
    gamma_type = type(gamma)
    if gamma_type != sympy.core.symbol.Symbol:
        cos = np.cos(gamma)
        sin = np.sin(gamma)
    else:
        cos = sympy.cos(gamma)
        sin = sympy.sin(gamma)

    rotation_matrix = np.array([
        [cos, -sin, 0],
        [sin,  cos, 0],
        [  0,    0, 1]
    ])
    return rotation_matrix

def rotInverse(rotation_matrix):
    """ Invert a 3x3 rotation matrix. """
    return rotation_matrix.T

def rotXYZ(alpha, beta, gamma):
    """ Create a 3x3 rotation matrix from XYZ Euler angles. """
    return rotZ(gamma) @ rotY(beta) @ rotX(alpha)

def rotXZY(alpha, gamma, beta):
    """ Create a 3x3 rotation matrix from XZY Euler angles. """
    return rotY(beta) @ rotZ(gamma) @ rotX(alpha)

def rotYXZ(beta, alpha, gamma):
    """ Create a 3x3 rotation matrix from YXZ Euler angles. """
    return rotZ(gamma) @ rotX(alpha) @ rotY(beta)

def rotYZX(beta, gamma, alpha):
    """ Create a 3x3 rotation matrix from YZX Euler angles. """
    return rotX(alpha) @ rotZ(gamma) @ rotY(beta)

def rotZXY(gamma, alpha, beta):
    """ Create a 3x3 rotation matrix from ZXY Euler angles. """
    return rotY(beta) @ rotX(alpha) @ rotZ(gamma)

def rotZYX(gamma, beta, alpha):
    """ Create a 3x3 rotation matrix from ZYX Euler angles. """
    return rotX(alpha) @ rotY(beta) @ rotZ(gamma)

def positionVector(x, y, z):
    """ Create a 3D position vector. """
    return np.array([x, y, z])

def crossProductMatrix(position_vector):
    """ Create a 3x3 cross product matrix from given position vector. """
    x = position_vector[0]
    y = position_vector[1]
    z = position_vector[2]

    cross_product_matrix = np.array([
        [ 0, -z,  y],
        [ z,  0, -x],
        [-y,  x,  0]
    ])
    return cross_product_matrix

class Frame(object):
    def __init__(self, name, rotation_matrix, position_vector):
        self.name = name
        self.rotation_matrix = rotation_matrix
        self.position_vector = position_vector

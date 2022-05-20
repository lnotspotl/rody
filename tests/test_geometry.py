import sympy
import numpy as np

import Geometry

###############################################################################

def test_rotX_symbolic():
    """ Generation of X rotation matrix with symbols. """
    alpha = sympy.Symbol("alpha")
    rotmat_x = Geometry.rotX(alpha)

    # [1, 0,           0         ]
    # [0, cos(alpha), -sin(alpha)]
    # [0, sin(alpha),  cos(alpha)]

    assert str(rotmat_x[0][0]) == "1"
    assert str(rotmat_x[0][1]) == "0"
    assert str(rotmat_x[0][2]) == "0"

    assert str(rotmat_x[1][0]) == "0"
    assert str(rotmat_x[1][1]) == "cos(alpha)"
    assert str(rotmat_x[1][2]) == "-sin(alpha)"

    assert str(rotmat_x[2][0]) == "0"
    assert str(rotmat_x[2][1]) == "sin(alpha)"
    assert str(rotmat_x[2][2]) == "cos(alpha)"

def test_rotX_numeric():
    """ Generation of X rotation matrix with numerical values. """

    alpha = np.pi / 2    # 90 deg.
    rotmat_x = Geometry.rotX(alpha)

    # [1, 0,  0]
    # [0, 0, -1]
    # [0, 1,  0]

    allowed_error = 1e-10

    assert abs(rotmat_x[0][0] - 1) <= allowed_error
    assert abs(rotmat_x[0][1] - 0) <= allowed_error
    assert abs(rotmat_x[0][2] - 0) <= allowed_error

    assert abs(rotmat_x[1][0] - 0) <= allowed_error
    assert abs(rotmat_x[1][1] - 0) <= allowed_error
    assert abs(rotmat_x[1][2] + 1) <= allowed_error

    assert abs(rotmat_x[2][0] - 0) <= allowed_error
    assert abs(rotmat_x[2][1] - 1) <= allowed_error
    assert abs(rotmat_x[2][2] - 0) <= allowed_error

###############################################################################

def test_rotY_symbolic():
    """ Generation of Y rotation matrix with symbols. """
    beta = sympy.Symbol("beta")
    rotmat_y = Geometry.rotY(beta)

    # [ cos(beta), 0, sin(beta)]
    # [ 0,         1, 0        ]
    # [-sin(beta), 0  cos(beta)]

    assert str(rotmat_y[0][0]) == "cos(beta)"
    assert str(rotmat_y[0][1]) == "0"
    assert str(rotmat_y[0][2]) == "sin(beta)"

    assert str(rotmat_y[1][0]) == "0"
    assert str(rotmat_y[1][1]) == "1"
    assert str(rotmat_y[1][2]) == "0"

    assert str(rotmat_y[2][0]) == "-sin(beta)"
    assert str(rotmat_y[2][1]) == "0"
    assert str(rotmat_y[2][2]) == "cos(beta)"

def test_rotY_numeric():
    """ Generation of X rotation matrix with numerical values. """

    beta = np.pi / 2    # 90 deg.
    rotmat_y = Geometry.rotY(beta)

    # [ 0, 0,  1]
    # [ 0, 1,  0]
    # [-1, 0,  0]

    allowed_error = 1e-10

    assert abs(rotmat_y[0][0] - 0) <= allowed_error
    assert abs(rotmat_y[0][1] - 0) <= allowed_error
    assert abs(rotmat_y[0][2] - 1) <= allowed_error

    assert abs(rotmat_y[1][0] - 0) <= allowed_error
    assert abs(rotmat_y[1][1] - 1) <= allowed_error
    assert abs(rotmat_y[1][2] - 0) <= allowed_error

    assert abs(rotmat_y[2][0] + 1) <= allowed_error
    assert abs(rotmat_y[2][1] - 0) <= allowed_error
    assert abs(rotmat_y[2][2] - 0) <= allowed_error

###############################################################################

def test_rotZ_symbolic():
    """ Generation of Y rotation matrix with symbols. """
    gamma = sympy.Symbol("gamma")
    rotmat_z = Geometry.rotZ(gamma)

    # [cos(gamma), -sin(gamma), 0]
    # [sin(gamma),  cos(gamma), 0]
    # [0,           0,          1]

    assert str(rotmat_z[0][0]) == "cos(gamma)"
    assert str(rotmat_z[0][1]) == "-sin(gamma)"
    assert str(rotmat_z[0][2]) == "0"

    assert str(rotmat_z[1][0]) == "sin(gamma)"
    assert str(rotmat_z[1][1]) == "cos(gamma)"
    assert str(rotmat_z[1][2]) == "0"

    assert str(rotmat_z[2][0]) == "0"
    assert str(rotmat_z[2][1]) == "0"
    assert str(rotmat_z[2][2]) == "1"

def test_rotZ_numeric():
    """ Generation of X rotation matrix with numerical values. """

    gamma = np.pi / 2    # 90 deg.
    rotmat_z = Geometry.rotZ(gamma)

    # [0, -1,  0]
    # [1,  0,  0]
    # [0,  0,  1]

    allowed_error = 1e-10

    assert abs(rotmat_z[0][0] - 0) <= allowed_error
    assert abs(rotmat_z[0][1] + 1) <= allowed_error
    assert abs(rotmat_z[0][2] - 0) <= allowed_error

    assert abs(rotmat_z[1][0] - 1) <= allowed_error
    assert abs(rotmat_z[1][1] - 0) <= allowed_error
    assert abs(rotmat_z[1][2] - 0) <= allowed_error

    assert abs(rotmat_z[2][0] - 0) <= allowed_error
    assert abs(rotmat_z[2][1] - 0) <= allowed_error
    assert abs(rotmat_z[2][2] - 1) <= allowed_error

###############################################################################

def test_rotInverse():
    """ Generation of an inverse matrix for a rotation matrix. """
    alpha = sympy.Symbol("alpha")
    beta = sympy.Symbol("beta")
    gamma = sympy.Symbol("gamma")

    rot_matrix = Geometry.rotYXZ(beta, alpha, gamma)
    rot_matrix_inv = Geometry.rotInverse(rot_matrix)

    I1 = rot_matrix @ rot_matrix_inv
    I2 = rot_matrix_inv @ rot_matrix

    for i in range(3):
        for j in range(3):
            I1[i][j] = sympy.simplify(I1[i][j])
            I2[i][j] = sympy.simplify(I2[i][j])

    assert str(I1[0][0]) == "1"
    assert str(I1[0][1]) == "0"
    assert str(I1[0][2]) == "0"

    assert str(I1[1][0]) == "0"
    assert str(I1[1][1]) == "1"
    assert str(I1[1][2]) == "0"

    assert str(I1[2][0]) == "0"
    assert str(I1[2][1]) == "0"
    assert str(I1[2][2]) == "1"

    assert str(I2[0][0]) == "1"
    assert str(I2[0][1]) == "0"
    assert str(I2[0][2]) == "0"

    assert str(I2[1][0]) == "0"
    assert str(I2[1][1]) == "1"
    assert str(I2[1][2]) == "0"

    assert str(I2[2][0]) == "0"
    assert str(I2[2][1]) == "0"
    assert str(I2[2][2]) == "1"

###############################################################################

def test_positionVector_symbolic():
    """ Generation of position vectors with symbolic values. """
    x = sympy.Symbol("x")
    y = sympy.Symbol("y")
    z = sympy.Symbol("z")

    pos_vector = Geometry.positionVector(x, y, z)

    assert str(pos_vector[0]) == "x"
    assert str(pos_vector[1]) == "y"
    assert str(pos_vector[2]) == "z"

###############################################################################

def test_crossProductMatrix_symbolic():
    """ Generation of cross product matrix with symbolic values. """
    x = sympy.Symbol("x")
    y = sympy.Symbol("y")
    z = sympy.Symbol("z")

    pos_vector = Geometry.positionVector(x, y, z)
    cross_product_matrix = Geometry.crossProductMatrix(pos_vector)

    #[ 0, -z,  y],
    #[ z,  0, -x],
    #[-y,  x,  0]

    assert str(cross_product_matrix[0][0]) == "0"
    assert str(cross_product_matrix[0][1]) == "-z"
    assert str(cross_product_matrix[0][2]) == "y"

    assert str(cross_product_matrix[1][0]) == "z"
    assert str(cross_product_matrix[1][1]) == "0"
    assert str(cross_product_matrix[1][2]) == "-x"
    
    assert str(cross_product_matrix[2][0]) == "-y"
    assert str(cross_product_matrix[2][1]) == "x"
    assert str(cross_product_matrix[2][2]) == "0"

###############################################################################

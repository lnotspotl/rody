import numpy as np
from numpy import pi

import Bodies
import Geometry

###############################################################################

def test_Body():
    ALLOWED_ERROR = 0.001
    inertia_matrix_bl = np.eye(3)

    # predecessor frame 0 
    pf = Geometry.Frame(
        name = "base_link_predecessor",
        rotation_matrix = Geometry.rotZ(0),
        position_vector = Geometry.positionVector(0,0,0)
    )

    # list of predecessor frames
    pf_list = list()
    pf_list.append(pf)

    # successor frame 0
    sf = Geometry.Frame(
            name = "joint1_frame",
            rotation_matrix = Geometry.rotZ(0),
            position_vector = Geometry.positionVector(0,0,0)
    )

    # list of successor frames
    sf_list = list()
    sf_list.append(sf)

    test_body = Bodies.Body(
            name = "test_body",
            mass = 1,
            inertia_matrix = np.eye(3),
            predecessor_frames = pf_list,
            successor_frames = sf_list)

    assert test_body.name == "test_body"
    assert test_body.mass == 1
    assert test_body.body_number == 0

    plucker_inertia_matrix = test_body.getPluckerInertiaMatrix()
    assert plucker_inertia_matrix.shape == (6,6)

    body_inertia_matrix = test_body.getInertiaMatrix()
    assert body_inertia_matrix.shape == (3,3)
    assert body_inertia_matrix[0][0] - 1 < ALLOWED_ERROR
    assert body_inertia_matrix[1][1] - 1 < ALLOWED_ERROR
    assert body_inertia_matrix[2][2] - 1 < ALLOWED_ERROR


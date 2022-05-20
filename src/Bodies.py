import numpy as np
import logging

import Geometry

class Body(object):
    """
    Representation of a body in a robot kinematic chain.

    ...

    Atributes
    ---------
    name : string
        Body's name

    mass : float
        Body's mass

    body_number: int
        Body's number

    X_T : dict(string : np.array(shape = (6,6))
        Dictionary of transformation matrices to body's successors'
        coordinate frames.

    Methods
    -------
    getPluckerInertiaMatrix()
        Return body's Plucker inertia matrix.

    getInertiaMatrix()
        Return body's classical inertia matrix.

    """

    body_number = 0
    def __init__(self, name, mass, inertia_matrix, predecessor_frames,
            successor_frames):
        """
        Parameters
        ----------
        name : string
            Name of the body.

        mass : float
            Body's mass.

        inertia_matrix : np.array(shape = (3,3))
            Body's 3x3 inertia matrix

        predecessor_frames: list[Geometry.Frame]
            List of predecessor frames. As of now, only open-loop robots
            are supported, so it's expected that there's only on entry
            in the list.

        successor_frames : list[Geometry.Frame]
            List of successor frames.

        """
        self.name = name
        self.mass = mass
        self.body_number = Body.body_number
        Body.body_number += 1
        self.__inertia_matrix = inertia_matrix
        self.X_T = dict()

        ## Here, we compute X_T, which is a 6x6 Plucker transformation matrix
        ## from the predecessors frame to the successors frame

        if len(predecessor_frames) >= 2:
            logging.error("Only open-loop robots are supported!")
            exit()

        p_or = predecessor_frames[0].rotation_matrix
        p_pos = predecessor_frames[0].position_vector

        # Rotation matrix, P --> COM
        R_p_com = Geometry.rotInverse(p_or)

        for i in range(len(successor_frames)):
            s_or = successor_frames[i].rotation_matrix
            s_pos = successor_frames[i].position_vector
            s_name = successor_frames[i].name

            # Rotation matrix, P --> S
            R_p_s = R_p_com @ s_or

            # Position P --> COM in P
            p_p_com_p = (-1) * R_p_com.dot(p_pos)

            # Position COM --> S in P
            p_com_s_p = R_p_com.dot(s_pos)

            # Position P --> S in P
            p_p_s_p = p_p_com_p + p_com_s_p

            # p_p_s_p cross product matrix
            ppsp_cpm = Geometry.crossProductMatrix(p_p_s_p)

            # p_p_com_p cross product matrix
            ppcomp_cpm = Geometry.crossProductMatrix(p_p_com_p)

            # Body's motion plucker transformation matrix, P --> S
            X_T = np.zeros(shape=(6,6))

            # top left-hand side corner
            X_T[:3,:3] = R_p_s

            # top right-hand side corner
            #X_T[:3,3:] = 0

            # bottom left-hand side corner
            X_T[3:,:3] = ppsp_cpm @ R_p_s

            # bottom right-hand side corner
            X_T[3:,3:] = R_p_s

            self.X_T[s_name] = X_T

        ## Here, we compute the Plucker 6x6 inertia matrix that is 
        ## expressed in the predecessors coordinate frame

        # inertia matrix expressed in predecessors frame, R @ I @ R^-1
        self.__inertia_matrix = R_p_com @ self.__inertia_matrix @ p_or

        ## create plucker inertia matrix
        self.__plucker_inertia_matrix = np.zeros(shape = (6,6))

        # top left-hand side corner
        self.__plucker_inertia_matrix[:3,:3] = self.__inertia_matrix + \
                self.mass * (ppcomp_cpm @ ppcomp_cpm.T)

        # top right-hand side corner
        self.__plucker_inertia_matrix[:3,3:] = self.mass * \
                ppcomp_cpm

        # bottom left-hand side corner
        self.__plucker_inertia_matrix[3:,:3] = self.mass * \
                ppcomp_cpm.T

        # bottom right-hand side corner
        self.__plucker_inertia_matrix[3:,3:] = self.mass * np.eye(3)

    def getPluckerInertiaMatrix(self):
        """ 
        Return body's 6x6 Plucker inertia matrix. 

        Returns
        -------
        plucker_inertia_matrix: np.array(shape = (6,6))
            Body's Plucker inertia matrix.
        """
        return self.__plucker_inertia_matrix

    def getInertiaMatrix(self):
        """
        Return body's classical 3x3 inertia matrix.

        Returns
        -------
        inertia_matrix : np.array(shape = (3,3))
            Body's classical inertia matrix
        """
        return self.__inertia_matrix

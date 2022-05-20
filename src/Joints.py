import numpy as np
import sympy        # symbolic mathematics

import Geometry 
import Plucker 

###############################################################################

class Joint(object):
    """
    Joint base class.

    ...

    Atributes
    ---------
    name: string
        Joint's name.

    Methods
    -------
    joint_number()
        Return joint number.

    frame_name()
        Return frame name

    """
    __joint_number_ = 1

    def __init__(self, name, frame, reset_counter = False):
        """
        Joint's constructor.

        Parameters
        ----------
        name : string
            Joint's name.

        reset_counter : bool, default = False
            If True, joint counter will be reset.
        """
        if reset_counter:
            Joint.__joint_number_ = 1
        self.name = name
        self.__frame_name = frame.name

        self.__joint_number = Joint.__joint_number_
        Joint.__joint_number_ += 1

    def joint_number(self):
        """
        Return joint number.

        Returns
        -------
        joint_number : int
        """
        return self.__joint_number

    def frame_name(self):
        """
        Return frame name.

        Returns
        -------
        frame_name : string
        """
        return self.__frame_name


###############################################################################

class RevoluteJoint(Joint):
    """
    Revolute joint class.

    ...
    
    Atributes
    ---------
    name : string

    Methods
    -------
    resetJointVariables(q = True, q_dot = True, q_ddot = None)
        Reset joint variables and set them to their symbolic representations.

    setJointVariables(q = None, q_dot = None, q_ddot = None)
        Set joint variables, they will no longer be symbolic.

    getJointVariables()
        Return current joint variables as in a dictionary.

    getControllableParameters()
        Return controllable parameters.

    X_J()
        Compute joint Plucker transformation matrix.

    v_j()
        Compute successor's body velocity with respect to the predecessor.

    a_j()
        Compute successor's body acceleration with respect to the predecessor.

    c_j()
        Compute successor's body bias acceleration with respect to the predecessor.

    joint_type()
        Return name of the joint type.

    motion_subspace()
        Return matrix whose columns span joint's motion subspace.

    motion_subspace_ad()
        Return apparent derivative of the motion subspace matrix.

    getJointPosition()
        Return joint position.

    getJointVelocity()
        Return joint velocity.

    getJointAcceleration()
        Return joint acceleration.

    extractControllableParameter(generalized_force)
        Extract controllable parameter values from given generalized force 
        and return a dictionary with names of parameters as keys and the
        correspoding values as values.

    """

    def __init__(self, name, frame, q = None, q_dot = None, q_ddot = None,
            reset_counter = False):
        """
        Revolute joint's constructor.

        Parameters
        ----------
        name : string
            Joint's name

        q : float, default = None
            Joint's default position.

        q_dot : floatm default = None
            Joint's default velocity.

        q_ddot : float, default = None
            Joint's default acceleration.

        reset_counter : bool
            Joint counter should be reset.
        """
            
        super().__init__(name, frame, reset_counter)
        self.__controllable_parameters = ["q" + str(self.joint_number())]
        self.setJointVariables(q, q_dot, q_ddot)

    def resetJointVariables(self, q = True, q_dot = True, q_ddot = True):
        """
        Reset given joint variables and set them to their 
        symbolic representations.

        Parameters
        ----------
        q : bool, default = True
            Variable q should be reset.

        q_dot : bool, default = True
            Varibalbe q_dot should be reset.

        q_ddot: bool, default = True
            Variable q_ddot should be reset.
        """

        if q:
            self.q = [sympy.Symbol(self.__controllable_parameters[0])]
        if q_dot:
            self.q_dot = [sympy.Symbol(self.__controllable_parameters[0] + "_dot")]
        if q_ddot:
            self.q_ddot = [sympy.Symbol(self.__controllable_parameters[0] + "_ddot")]

    def setJointVariables(self, q = None, q_dot = None, q_ddot = None):
        """
        Set given joint variables. Unspecified joint variables are reset.

        Parameters
        ----------
        q : float
            New joint's position.

        q_dot : float
            New joint's velocity.

        q_ddot: float
            New joint's acceleration.
        """
        if q != None:
            assert len(q) == len(self.q)
            for i in range(len(q)):
                self.q[i] = q[i] if q[i] != None else \
                        sympy.Symbol(self.__controllable_parameters[i])

        else:
            self.resetJointVariables(q = True, q_dot = False, q_ddot = False)

        if q_dot != None:
            assert len(q_dot) == len(self.q_dot)
            for i in range(len(self.q_dot)):
                self.q_dot[i] = q_dot[i] if q_dot[i] != None else \
                        sympy.Symbol(self.__controllable_parameters[i] + "_dot")
        else:
            self.resetJointVariables(q = False, q_dot = True, q_ddot = False)

        if q_ddot != None:
            assert len(q_ddot) == len(self.q_ddot)
            for i in range(len(self.q_ddot)):
                self.q_ddot[i] = q_ddot[i] if q_ddot[i] != None else \
                        sympy.Symbol(self.__controllable_parameters[i] + "_ddot")
        else:
            self.resetJointVariables(q = False, q_dot = False, q_ddot = True)

    def getJointVariables(self):
        """
        Return current joint variables as in a dictionary.

        Returns
        -------
        joint_variables: dict[string : list[int]]
            Dictionary with joint variable names as keys and joint varible
            values as values.
            Value 'None' represents symbolic value.

             {
                "q" :     [None, None]
                "q_dot":  [None, 12],
                "q_ddot": [1,2]
              }

        """

        joint_variables = dict()
        joint_variables["q"] = list()
        joint_variables["q_dot"] = list()
        joint_variables["q_ddot"] = list()

        for q in self.q:
            if type(q) == sympy.core.symbol.Symbol:
                joint_variables["q"].append(None)
            else:
                joint_variables["q"].append(q)

        for q_dot in self.q_dot:
            if type(q_dot) == sympy.core.symbol.Symbol:
                joint_variables["q_dot"].append(None)
            else:
                joint_variables["q_dot"].append(q_dot)

        for q_ddot in self.q_ddot:
            if type(q_ddot) == sympy.core.symbol.Symbol:
                joint_variables["q_ddot"].append(None)
            else:
                joint_variables["q_ddot"].append(q_ddot)

        return joint_variables

    def getControllableParameters(self):
        """
        Return controllable parameters.

        Returns
        -------
        controllable_parameters : list[string]

        """
        return self.__controllable_parameters

    def X_J(self):
        """ 
        Compute joint transformation matrix.

        Returns
        -------
        joint_transformation_matrix : np.array(shape = (6,6))
            Joint's transformation matrix. It is a 6x6 Plucker
            motion matrix and it expresses the position and orientation
            of successor's body with respect to the predecessor's body.
        """
        rotation_matrix = Geometry.rotZ(self.q[0])
        position_vector = Geometry.positionVector(0, 0, 0)

        joint_transformation_matrix = Plucker.\
            pluckerMotionTransform(rotation_matrix, position_vector)
        return joint_transformation_matrix

    def v_j(self):
        """ 
        Compute relative velocity of body i with respect to its parent.

        Returns
        -------
        successors_velocity : np.array(shape = (6,))
            Successor's velocity relative to it's predecessor. The given
            vector is a Plucker motion vector.
        """
        successors_velocity = self.motion_subspace() @ self.getJointVelocity()
        return successors_velocity

    def a_j(self):
        """ 
        Compute relative acceleration of body i with respect to its parent.

        Returns
        -------
        successors_acceleration : np.array(shape = (6,))
            Successor's acceleration relative to it's predecessor. The given
            acceleration is a Plucker motion vector.
        """
        successors_acceleration = self.motion_subspace() @\
                self.getJointAcceleration()
        return successors_acceleration

    def c_j(self):
        """
        Compute successor's body bias acceleration with respect to the predecessor.
        
        Returns
        -------
        successors_bias_acceleration : np.array(shape = (6,))
        """
        successors_bias_acceleration = self.motion_subspace_ad() @\
                self.getJointVelocity()
        return successors_bias_acceleration

    def joint_type(self):
        """ 
        Return joint type.

        Returns
        -------
        joint_type : string
        """
        return "revolute"

    def motion_subspace(self):
        """ 
        Return matrix whose columns span joint's motion subspace.

        Returns
        -------
        m_matrix : np.array(shape = (6,6))
        """
        return np.array([
                [0,0,0,0,0,0],
                [0,0,0,0,0,0],
                [0,0,1,0,0,0],
                [0,0,0,0,0,0],
                [0,0,0,0,0,0],
                [0,0,0,0,0,0],
                ])

    def motion_subspace_ad(self):
        """
        Return apparent derivative of the motion subspace matrix.

        Returns
        -------
        m_matrix_d : np.array(shape = (6,6)):
            Apparent derivative of self.motion_subspace()
        """
        return np.zeros(shape = (6,6))

    def getJointPosition(self):
        """
        Return joint's position.

        Returns
        -------
        q_ddot : np.array(shape = (6,))

        """
        return np.array([0,0,self.q[0],0,0,0])

    def getJointVelocity(self):
        """
        Return joint's velocity.

        Returns
        -------
        q_ddot : np.array(shape = (6,))

        """
        return np.array([0,0,self.q_dot[0],0,0,0])

    def getJointAcceleration(self):
        """
        Return joint's acceleration.

        Returns
        -------
        q_ddot : np.array(shape = (6,))

        """
        return np.array([0,0,self.q_ddot[0],0,0,0])

    def extractControllableParameter(self, generalized_force):
        """
        Extract controllable parameter values from given generalized force 
        and return a dictionary with names of parameters as keys and the
        correspoding values as values.

        Parameters
        ----------
        generalized_force : np.array(shape = (6,))

        Returns
        -------
        extracted_values : dictionary[string : float]
        """
        extracted_values = dict()
        val = generalized_force[2]
        extracted_values["tau_" + self.__controllable_parameters[0]] = val

        return extracted_values

###############################################################################
        
class PrismaticJoint(Joint):
    """
    Prismatic joint class.

    ...
    
    Atributes
    ---------
    name : string

    Methods
    -------
    resetJointVariables(q = True, q_dot = True, q_ddot = True)
        Reset joint variables and set them to their symbolic representations.

    setJointVariables(q = None, q_dot = None, q_ddot = None)
        Set joint variables, they will no longer be symbolic.

    getJointVariables()
        Return current joint variables as in a dictionary.

    getControllableParameters()
        Return controllable parameters.

    X_J()
        Compute joint Plucker transformation matrix.

    v_j()
        Compute successor's body velocity with respect to the predecessor.

    a_j()
        Compute successor's body acceleration with respect to the predecessor.

    c_j()
        Compute successor's body bias acceleration with respect to the predecessor.

    joint_type()
        Return name of the joint type.

    motion_subspace()
        Return matrix whose columns span joint's motion subspace.

    motion_subspace_ad()
        Return apparent derivative of the motion subspace matrix.

    getJointPosition()
        Return joint position.

    getJointVelocity()
        Return joint velocity.

    getJointAcceleration()
        Return joint acceleration.

    extractControllableParameter(generalized_force)
        Extract controllable parameter values from given generalized force 
        and return a dictionary with names of parameters as keys and the
        correspoding values as values.
    """
    def __init__(self, name, frame, q = None, q_dot = None, q_ddot = None,
            reset_counter = False):
        """
        Prismatic joint's constructor.

        Parameters
        ----------
        name : string
            Joint's name

        q : float, default = None
            Joint's default position.

        q_dot : floatm default = None
            Joint's default velocity.

        q_ddot : float, default = None
            Joint's default acceleration.

        reset_counter : bool
            Joint counter should be reset.
        """
        super().__init__(name, frame, reset_counter)
        self.__controllable_parameters = ["d" + str(self.joint_number())]
        self.setJointVariables(q, q_dot, q_ddot)

    def resetJointVariables(self, q = True, q_dot = True, q_ddot = True):
        """
        Reset given joint variables and set them to their 
        symbolic representations.

        Parameters
        ----------
        q : bool, default = True
            Variable q should be reset.

        q_dot : bool, default = True
            Varibalbe q_dot should be reset.

        q_ddot: bool, default = True
            Variable q_ddot should be reset.
        """
        if q:
            self.q = [sympy.Symbol(self.__controllable_parameters[0])]
        if q_dot:
            self.q_dot = [sympy.Symbol(self.__controllable_parameters[0] + "_dot")]
        if q_ddot:
            self.q_ddot = [sympy.Symbol(self.__controllable_parameters[0] + "_ddot")]

    def setJointVariables(self, q = None, q_dot = None, q_ddot = None):
        """
        Set given joint variables. Unspecified joint variables are reset.

        Parameters
        ----------
        q : float
            New joint's position.

        q_dot : float
            New joint's velocity.

        q_ddot: float
            New joint's acceleration.
        """
        if q != None:
            assert len(q) == len(self.q)
            for i in range(len(q)):
                self.q[i] = q[i] if q[i] != None else \
                        sympy.Symbol(self.__controllable_parameters[i])
        else:
            self.resetJointVariables(q = True, q_dot = False, q_ddot = False)

        if q_dot != None:
            assert len(q_dot) == len(self.q_dot)
            for i in range(len(self.q_dot)):
                self.q_dot[i] = q_dot[i] if q_dot[i] != None else \
                        sympy.Symbol(self.__controllable_parameters[i] + "_dot")
        else:
            self.resetJointVariables(q = False, q_dot = True, q_ddot = False)

        if q_ddot != None:
            assert len(q_ddot) == len(self.q_ddot)
            for i in range(len(self.q_ddot)):
                self.q_ddot[i] = q_ddot[i] if q_ddot[i] != None else \
                        sympy.Symbol(self.__controllable_parameters[i] + "_ddot")
        else:
            self.resetJointVariables(q = False, q_dot = False, q_ddot = True)

    def getJointVariables(self):
        """
        Return current joint variables as in a dictionary.

        Returns
        -------
        joint_variables: dict[string : list[int]]
            Dictionary with joint variable names as keys and joint varible
            values as values.
            Value 'None' represents symbolic value.

             {
                "q" :     [None, None]
                "q_dot":  [None, 12],
                "q_ddot": [1,2]
              }

        """

        joint_variables = dict()
        joint_variables["q"] = list()
        joint_variables["q_dot"] = list()
        joint_variables["q_ddot"] = list()

        for q in self.q:
            if type(q) == sympy.core.symbol.Symbol:
                joint_variables["q"].append(None)
            else:
                joint_variables["q"].append(q)

        for q_dot in self.q_dot:
            if type(q_dot) == sympy.core.symbol.Symbol:
                joint_variables["q_dot"].append(None)
            else:
                joint_variables["q_dot"].append(q_dot)

        for q_ddot in self.q_ddot:
            if type(q_ddot) == sympy.core.symbol.Symbol:
                joint_variables["q_ddot"].append(None)
            else:
                joint_variables["q_ddot"].append(q_ddot)

        return joint_variables

    def getControllableParameters(self):
        """
        Return controllable parameters.

        Returns
        -------
        controllable_parameters : list[string]

        """
        return self.__controllable_parameters

    def X_J(self):
        """ 
        Compute joint transformation matrix.

        Returns
        -------
        joint_transformation_matrix : np.array(shape = (6,6))
            Joint's transformation matrix. It is a 6x6 Plucker
            motion matrix and it expresses the position and orientation
            of successor's body with respect to the predecessor's body.
        """
        rotation_matrix = Geometry.rotX(0)
        position_vector = Geometry.positionVector(0, 0, self.q[0])

        joint_transformation_matrix = Plucker.\
            pluckerMotionTransform(rotation_matrix, position_vector)
        return joint_transformation_matrix

    def v_j(self):
        """ 
        Compute relative velocity of body i with respect to its parent.

        Returns
        -------
        successors_velocity : np.array(shape = (6,))
            Successor's velocity relative to it's predecessor. The given
            vector is a Plucker motion vector.
        """
        successors_velocity = self.motion_subspace() @ self.getJointVelocity()
        return successors_velocity

    def a_j(self):
        """ 
        Compute relative acceleration of body i with respect to its parent.

        Returns
        -------
        successors_acceleration : np.array(shape = (6,))
            Successor's acceleration relative to it's predecessor. The given
            acceleration is a Plucker motion vector.
        """
        successors_acceleration = self.motion_subspace() @\
                self.getJointAcceleration()
        return successors_acceleration

    def c_j(self):
        """
        Compute successor's body bias acceleration with respect to the predecessor.
        
        Returns
        -------
        successors_bias_acceleration : np.array(shape = (6,))
        """
        successors_bias_acceleration = self.motion_subspace_ad() @\
                self.getJointVelocity()
        return successors_bias_acceleration

    def joint_type(self):
        """ 
        Return joint type.

        Returns
        -------
        joint_type : string
        """
        return "prismatic"

    def motion_subspace(self):
        """ 
        Return matrix whose columns span joint's motion subspace.

        Returns
        -------
        m_matrix : np.array(shape = (6,6))
        """
        return np.array([
                [0,0,0,0,0,0],
                [0,0,0,0,0,0],
                [0,0,0,0,0,0],
                [0,0,0,0,0,0],
                [0,0,0,0,0,0],
                [0,0,0,0,0,1],
            ])

    def motion_subspace_ad(self):
        """
        Return apparent derivative of the motion subspace matrix.

        Returns
        -------
        m_matrix_d : np.array(shape = (6,6)):
            Apparent derivative of self.motion_subspace()
        """
        return np.zeros(shape = (6,6))

    def getJointPosition(self):
        """
        Return joint's position.

        Returns
        -------
        q_ddot : np.array(shape = (6,))

        """
        return np.array([0,0,0,0,0,self.q[0]])

    def getJointVelocity(self):
        """
        Return joint's velocity.

        Returns
        -------
        q_ddot : np.array(shape = (6,))

        """
        return np.array([0,0,0,0,0,self.q_dot[0]])

    def getJointAcceleration(self):
        """
        Return joint's acceleration.

        Returns
        -------
        q_ddot : np.array(shape = (6,))

        """
        return np.array([0,0,0,0,0,self.q_ddot[0]])

    def extractControllableParameter(self, generalized_force):
        """
        Extract controllable parameter values from given generalized force 
        and return a dictionary with names of parameters as keys and the
        correspoding values as values.

        Parameters
        ----------
        generalized_force : np.array(shape = (6,))

        Returns
        -------
        extracted_values : dictionary[string : float]
        """
        extracted_values = dict()
        val = generalized_force[5]
        extracted_values["tau_" + self.__controllable_parameters[0]] = val

        return extracted_values
        
###############################################################################

class FloatingJoint(Joint):
    """
    Floating joint class.

    ...
    
    Atributes
    ---------
    name : string

    Methods
    -------
    resetJointVariables(q = True, q_dot = True, q_ddot = True)
        Reset joint variables and set them to their symbolic representations.

    setJointVariables(q = None, q_dot = None, q_ddot = None)
        Set joint variables, they will no longer be symbolic.

    getJointVariables()
        Return current joint variables as in a dictionary.

    getControllableParameters()
        Return controllable parameters.

    X_J()
        Compute joint Plucker transformation matrix.

    v_j()
        Compute successor's body velocity with respect to the predecessor.

    a_j()
        Compute successor's body acceleration with respect to the predecessor.

    c_j()
        Compute successor's body bias acceleration with respect to the predecessor.

    joint_type()
        Return name of the joint type.

    motion_subspace()
        Return matrix whose columns span joint's motion subspace.

    motion_subspace_ad()
        Return apparent derivative of the motion subspace matrix.

    getJointPosition()
        Return joint position.

    getJointVelocity()
        Return joint velocity.

    getJointAcceleration()
        Return joint acceleration.

    extractControllableParameter(generalized_force)
        Extract controllable parameter values from given generalized force 
        and return a dictionary with names of parameters as keys and the
        correspoding values as values.
    """
    def __init__(self, name, frame, q = None, q_dot = None, q_ddot = None,
            reset_counter = False):
        """
        Floating joint's constructor.

        Parameters
        ----------
        name : string
            Joint's name

        q : float, default = None
            Joint's default position.

        q_dot : floatm default = None
            Joint's default velocity.

        q_ddot : float, default = None
            Joint's default acceleration.

        reset_counter : bool
            Joint counter should be reset.
        """
        super().__init__(name, frame, reset_counter)
        # roll, pitch, yaw (Euler angles), x, y ,z

        self.__controllable_parameters = ["re", "pe", "ye", "x", "y", "z"]
        for i in range(len(self.__controllable_parameters)):
            self.__controllable_parameters[i] += str(self.joint_number())
        self.setJointVariables(q, q_dot, q_ddot)

    def resetJointVariables(self, q = True, q_dot = True, q_ddot = True):
        """
        Reset given joint variables and set them to their 
        symbolic representations.

        Parameters
        ----------
        q : bool, default = True
            Variable q should be reset.

        q_dot : bool, default = True
            Varibalbe q_dot should be reset.

        q_ddot: bool, default = True
            Variable q_ddot should be reset.
        """

        if q:
            self.q = [None for _ in range(len(self.__controllable_parameters))]
            for i in range(len(self.__controllable_parameters)):
                self.q[i] = sympy.Symbol(self.__controllable_parameters[i])
        if q_dot:
            self.q_dot = [None for _ in range(len(self.__controllable_parameters))]
            for i in range(len(self.__controllable_parameters)):
                self.q_dot[i] = sympy.Symbol(self.__controllable_parameters[i] + "_dot")

        if q_ddot:
            self.q_ddot = [None for _ in range(len(self.__controllable_parameters))]
            for i in range(len(self.__controllable_parameters)):
                self.q_ddot[i] = sympy.Symbol(self.__controllable_parameters[i] + "_ddot")

    def setJointVariables(self, q = None, q_dot = None, q_ddot = None):
        """
        Set given joint variables. Unspecified joint variables are reset.

        Parameters
        ----------
        q : list[float]
            New joint's positions.

        q_dot : list[float]
            New joint's velocities.

        q_ddot: list[float]
            New joint's accelerations.
        """
        if q != None:
            assert len(q) == len(self.q)
            for i in range(len(q)):
                self.q[i] = q[i] if q[i] != None else \
                        sympy.Symbol(self.__controllable_parameters[i])
        else:
            self.resetJointVariables(q = True, q_dot = False, q_ddot = False)

        if q_dot != None:
            assert len(q_dot) == len(self.q_dot)
            for i in range(len(self.q_dot)):
                self.q_dot[i] = q_dot[i] if q_dot[i] != None else \
                        sympy.Symbol(self.__controllable_parameters[i] + "_dot")
        else:
            self.resetJointVariables(q = False, q_dot = True, q_ddot = False)

        if q_ddot != None:
            assert len(q_ddot) == len(self.q_ddot)
            for i in range(len(self.q_ddot)):
                self.q_ddot[i] = q_ddot[i] if q_ddot[i] != None else \
                        sympy.Symbol(self.__controllable_parameters[i] + "_ddot")
        else:
            self.resetJointVariables(q = False, q_dot = False, q_ddot = True)

    def getJointVariables(self):
        """
        Return current joint variables as in a dictionary.

        Returns
        -------
        joint_variables: dict[string : list[int]]
            Dictionary with joint variable names as keys and joint varible
            values as values.
            Value 'None' represents symbolic value.

             {
                "q" :     [None, None]
                "q_dot":  [None, 12],
                "q_ddot": [1,2]
              }

        """

        joint_variables = dict()
        joint_variables["q"] = list()
        joint_variables["q_dot"] = list()
        joint_variables["q_ddot"] = list()

        for q in self.q:
            if type(q) == sympy.core.symbol.Symbol:
                joint_variables["q"].append(None)
            else:
                joint_variables["q"].append(q)

        for q_dot in self.q_dot:
            if type(q_dot) == sympy.core.symbol.Symbol:
                joint_variables["q_dot"].append(None)
            else:
                joint_variables["q_dot"].append(q_dot)

        for q_ddot in self.q_ddot:
            if type(q_ddot) == sympy.core.symbol.Symbol:
                joint_variables["q_ddot"].append(None)
            else:
                joint_variables["q_ddot"].append(q_ddot)

        return joint_variables

    def getControllableParameters(self):
        """
        Return controllable parameters.

        Returns
        -------
        controllable_parameters : list[string]

        """
        return self.__controllable_parameters

    def X_J(self):
        """ 
        Compute joint transformation matrix.

        Returns
        -------
        joint_transformation_matrix : np.array(shape = (6,6))
            Joint's transformation matrix. It is a 6x6 Plucker
            motion matrix and it expresses the position and orientation
            of successor's body with respect to the predecessor's body.
        """
        # Euler angles
        roll = self.q[0]
        pitch = self.q[1]
        yaw = self.q[2]
        # position vector
        x = self.q[3]
        y = self.q[4]
        z = self.q[5]

        # Euler XYZ 3x3 rotation matrix
        rotation_matrix = Geometry.rotXYZ(roll, pitch, yaw) 
        position_vector = Geometry.positionVector(x, y, z)
        
        joint_transformation_matrix = Plucker.pluckerMotionTransform(
                rotation_matrix, position_vector)

        return joint_transformation_matrix

    def v_j(self):
        """ 
        Compute relative velocity of body i with respect to its parent.

        Returns
        -------
        successors_velocity : np.array(shape = (6,))
            Successor's velocity relative to it's predecessor. The given
            vector is a Plucker motion vector.
        """
        successors_velocity = self.motion_subspace() @ self.getJointVelocity()
        return successors_velocity

    def a_j(self):
        """ 
        Compute relative acceleration of body i with respect to its parent.

        Returns
        -------
        successors_acceleration : np.array(shape = (6,))
            Successor's acceleration relative to it's predecessor. The given
            acceleration is a Plucker motion vector.
        """
        successors_acceleration = self.motion_subspace() @\
                self.getJointAcceleration()
        return successors_acceleration

    def c_j(self):
        """
        Compute successor's body bias acceleration with respect to the predecessor.
        
        Returns
        -------
        successors_bias_acceleration : np.array(shape = (6,))
        """
        successors_bias_acceleration = self.motion_subspace_ad() @\
                self.getJointVelocity()
        return successors_bias_acceleration

    def joint_type(self):
        """
        Return joint type.

        Returns
        -------
        joint_type : string
        """
        return "floating"

    def motion_subspace(self):
        """ 
        Return matrix whose columns span joint's motion subspace.

        Returns
        -------
        m_matrix : np.array(shape = (6,6))
        """
        p = self.q[1] # pitch
        y = self.q[2] # yaw

        if type(p) != sympy.core.symbol.Symbol:
            c_p = np.cos(p)
            s_p = np.sin(p)
        else:
            c_p = sympy.cos(p)
            s_p = sympy.sin(p)

        if type(y) != sympy.core.symbol.Symbol:
            c_y = np.cos(y)
            s_y = np.sin(y)
        else:
            c_y = sympy.cos(y)
            s_y = sympy.sin(y)

        return np.array([
                [c_y * c_p ,s_y,0,0,0,0],
                [-s_y * c_p,c_y,0,0,0,0],
                [s_p        ,0 ,1,0,0,0],
                [0          ,0 ,0,1,0,0],
                [0          ,0 ,0,0,1,0],
                [0          ,0 ,0,0,0,1],
            ])

    def motion_subspace_ad(self):
        """
        Return apparent derivative of the motion subspace matrix.

        Returns
        -------
        m_matrix_d : np.array(shape = (6,6)):
            Apparent derivative of self.motion_subspace()
        """
        p = self.q[1] # pitch
        y = self.q[2] # yaw

        pd = self.q_dot[1] # pitch_dot
        yd = self.q_dot[2] # yaw_dot

        if type(p) != sympy.core.symbol.Symbol:
            c_p = np.cos(p)
            s_p = np.sin(p)
        else:
            c_p = sympy.cos(p)
            s_p = sympy.sin(p)

        if type(y) != sympy.core.symbol.Symbol:
            c_y = np.cos(y)
            s_y = np.sin(y)
        else:
            c_y = sympy.cos(y)
            s_y = sympy.sin(y)

        return np.array([
                [-s_y * c_p * yd - c_y * s_p * pd,c_y * yd ,0,0,0,0],
                [-c_y * c_p * yd + s_y * s_p * pd,-s_y * yd,0,0,0,0],
                [c_p * pd                        ,0        ,0,0,0,0],
                [0                               ,0        ,0,0,0,0],
                [0                               ,0        ,0,0,0,0],
                [0                               ,0        ,0,0,0,0],
            ])

    def getJointPosition(self):
        """
        Return joint's position.

        Returns
        -------
        q_ddot : np.array(shape = (6,))

        """
        return np.array([self.q[0],
                         self.q[1],
                         self.q[2],
                         self.q[3],
                         self.q[4],
                         self.q[5]])

    def getJointVelocity(self):
        """
        Return joint's velocity.

        Returns
        -------
        q_ddot : np.array(shape = (6,))

        """
        return np.array([self.q_dot[0],
                         self.q_dot[1],
                         self.q_dot[2],
                         self.q_dot[3],
                         self.q_dot[4],
                         self.q_dot[5]])

    def getJointAcceleration(self):
        """
        Return joint's acceleration.

        Returns
        -------
        q_ddot : np.array(shape = (6,))

        """
        return np.array([self.q_ddot[0],
                         self.q_ddot[1],
                         self.q_ddot[2],
                         self.q_ddot[3],
                         self.q_ddot[4],
                         self.q_ddot[5]])

    def extractControllableParameter(self, generalized_force):
        """
        Extract controllable parameter values from given generalized force 
        and return a dictionary with names of parameters as keys and the
        correspoding values as values.

        Parameters
        ----------
        generalized_force : np.array(shape = (6,))

        Returns
        -------
        extracted_values : dictionary[string : float]
        """
        extracted_values = dict()
        roll = generalized_force[0]
        pitch = generalized_force[1]
        yaw = generalized_force[2]
        x = generalized_force[3]
        y = generalized_force[4]
        z = generalized_force[5]

        extracted_values["tau_" + self.__controllable_parameters[0]] = roll
        extracted_values["tau_" + self.__controllable_parameters[1]] = pitch
        extracted_values["tau_" + self.__controllable_parameters[2]] = yaw

        extracted_values["tau_" + self.__controllable_parameters[3]] = x
        extracted_values["tau_" + self.__controllable_parameters[4]] = y
        extracted_values["tau_" + self.__controllable_parameters[5]] = z

        return extracted_values

###############################################################################
    
class RigidJoint(Joint):
    """
    Rigid joint class.

    ...
    
    Atributes
    ---------
    name : string

    Methods
    -------
    getControllableParameters()
        Return controllable parameters.

    X_J()
        Compute joint Plucker transformation matrix.

    v_j()
        Compute successor's body velocity with respect to the predecessor.

    a_j()
        Compute successor's body acceleration with respect to the predecessor.

    c_j()
        Compute successor's body bias acceleration with respect to the predecessor.

    joint_type()
        Return name of the joint type.

    motion_subspace()
        Return matrix whose columns span joint's motion subspace.

    motion_subspace_ad()
        Return apparent derivative of the motion subspace matrix.

    getJointPosition()
        Return joint position.

    getJointVelocity()
        Return joint velocity.

    getJointAcceleration()
        Return joint acceleration.
    """

    def __init__(self, name, frame, reset_counter = False):
        """
        Rigid joint's constructor.

        Parameters
        ----------
        name : string
            Joint's name

        reset_counter : bool
            Joint counter should be reset.
        """
        super().__init__(name, frame, reset_counter)

    def getControllableParameters(self):
        """
        Return controllable parameters.

        Returns
        -------
        controllable_parameters : list[string]

        """
        return [None]

    def X_J(self):
        """ 
        Compute joint transformation matrix.

        Returns
        -------
        joint_transformation_matrix : np.array(shape = (6,6))
            Joint's transformation matrix. It is a 6x6 Plucker
            motion matrix and it expresses the position and orientation
            of successor's body with respect to the predecessor's body.
        """
        rotation_matrix = Geometry.rotX(0)
        position_vector = Geometry.positionVector(0, 0, 0)

        joint_transformation_matrix = Plucker.\
            pluckerMotionTransform(rotation_matrix, position_vector)

        return joint_transformation_matrix

    def v_j(self):
        """ 
        Compute relative velocity of body i with respect to its parent.

        Returns
        -------
        successors_velocity : np.array(shape = (6,))
            Successor's velocity relative to it's predecessor. The given
            vector is a Plucker motion vector.
        """
        successors_velocity = self.motion_subspace() @ self.getJointVelocity()
        return successors_velocity

    def a_j(self):
        """ 
        Compute relative acceleration of body i with respect to its parent.

        Returns
        -------
        successors_acceleration : np.array(shape = (6,))
            Successor's acceleration relative to it's predecessor. The given
            acceleration is a Plucker motion vector.
        """
        successors_acceleration = self.motion_subspace() @\
                self.getJointAcceleration()
        return successors_acceleration

    def c_j(self):
        """
        Compute successor's body bias acceleration with respect to the predecessor.
        
        Returns
        -------
        successors_bias_acceleration : np.array(shape = (6,))
        """
        successors_bias_acceleration = self.motion_subspace_ad() @\
                self.getJointVelocity()
        return successors_bias_acceleration

    def joint_type(self):
        """
        Return joint type.

        Returns
        -------
        joint_type : string
        """
        return "rigid"

    def motion_subspace(self):
        """ 
        Return matrix whose columns span joint's motion subspace.

        Returns
        -------
        m_matrix : np.array(shape = (6,6))
        """
        return np.zeros(shape = (6,6))

    def motion_subspace_ad(self):
        """
        Return apparent derivative of the motion subspace matrix.

        Returns
        -------
        m_matrix_d : np.array(shape = (6,6)):
            Apparent derivative of self.motion_subspace()
        """
        return np.zeros(shape = (6,6))

    def getJointPosition(self):
        """
        Return joint's position.

        Returns
        -------
        q_ddot : np.array(shape = (6,))

        """
        return np.array([0,0,0,0,0,0])

    def getJointVelocity(self):
        """
        Return joint's velocity.

        Returns
        -------
        q_ddot : np.array(shape = (6,))

        """
        return np.array([0,0,0,0,0,0])

    def getJointAcceleration(self):
        """
        Return joint's acceleration.

        Returns
        -------
        q_ddot : np.array(shape = (6,))

        """
        return np.array([0,0,0,0,0,0])

###############################################################################

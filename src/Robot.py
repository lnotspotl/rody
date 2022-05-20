import logging
import numpy as np
import pygraphviz

import Joints
import Bodies 
import Plucker 

class Robot(object):
    """
    Robot class.

    ...
    
    Atributes
    ---------
    name : string
        Robot's name, default = "^w^"

    parent_array : list[int]
        List of indexes of parents for bodies. 

    joint_array : list[Joints.Joint]
        List of joints between bodies.

    bodies : list[Bodies.Body]
        List of bodies.
        
    joint_dict : dictionary[string : int]
        Mapping of joint names to their indexes in joint_array.

    body_dict : dictionary[string : int]
        Mapping of body names to their indexes in bodies.

    gravity : np.array(shape = (6,)), default = np.array([0, 0, 0, -9.81, 0, 0])
        Gravity vector in Plucker coordinates

    Methods
    -------
    addJoint(name, joint_type, default_values = None)
        Add new joint that is attached to the end of the previously added body.

    addBody(name, mass, inertia_matrix, p_pos, p_or, s_por, s_or,
        parent = None)
        Add new body, whose parent is the previously added body, unless
        'parent' is set to real parent's number / name.

    pluckerTransformationMatrix(from_c, to_c)
        Compute Plucker motion transformation matrix, where 'from_c' indicates
        the 'from_c' body's coordinate frame and 'to' indicates 'to_c' body's 
        coordinate frame.

    setJointVariables(joint_variable_dict)
        Given a dictionary of joint names and the required joint variables
        in a dictionary, the requested joint variables are set.
        If some varibles are wished not to be changed, pass None as value.

    getJointVariables()
        Return a dictionary with current joint variables of all joints.
        The keys are joint names, the values are current joint variables.

    newtonEulerID(external_forces = {}, gravity_compensation = True)
        Compute robot's inverse dynamics given external forces acting on bodies.
        'external_forces' is a dictionary with names of bodies as keys
        and a tuple (force, local) as a value. If 'local' is set to True,
        the given force is expressed in the local frame.  Otherwise it's
        expressed in the global frame. 'force' is a Plucker force vector.

        If gravity_compensation is set to 'True', the resulting forces will 
        also try to compensate gravity.

    BiasForceVector(set_to_symbolic = True)
        Compute the C vector of bias forces. This vector is part of the 
        manipulator equation M * q_ddot + C = F, where F is a vector 
        of generalized forces.
        The vector C is computed using the Newton-Euler recursive inverse
        dynamics algorithm.

    MassMatrix()
        Compute robot's mass matrix via inverse dynamics.
        The mass matrix M is part of the manipulator equation M * q_ddot + C = F,
        where F is a vector of generalized forces.

    subtreeBodies(body_i)
        Return set of bodies supported by body 'body_i'

    generateGraph(output_file = "robot.png")
        Generate graph that shows robot's structure. The graph is saved
        into a file 'output_file'.png

    summary()
        Print out a short summary of the robot's structure, joints and bodies.
    """
    def __init__(self,name = "^w^", gravity = np.array([0, 0, 0, 0, -9.81, 0])):
        """
        Robot's constructor.

        Parameters
        ----------
        name : string
            Robot's name
        gravity : np.array(shape = (6,))
            Gravity vector, in Plucker coordinates
        """
        self.name = name

        # Number of bodies
        self.parent_array = list()
        self.joint_array = list()
        self.bodies = list()
        self.joint_dict = dict()
        self.body_dict = dict()

        self.gravity = gravity

    def addJoint(self, name, joint_type, frame,
                    default_values = [None, None, None]):
        """ 
        Add new joint that is attached to the end of the previously added body.

        Parameters
        ----------
        name : string
        
        joint_type : string
            Can be: 'revolute', 'prismatic', 'floating' or 'rigid'

        default_values : list[list[float]]
            Default numerical values, if value is None, nothing changes and
            the particular joint variable stays symbolic.
            If 'default_values' == None, all the joint variables stay symbolic.

            Setting 'default_values' for rigid joint has no effect.

        """
        if len(self.bodies) == 0:
            logging.error("You first need to add base link.")
            return False

        # Check whether given name has already been used
        if name in self.joint_dict:
            logging.error(f"Joint name '{name}' already exists.")
            return False
            
        # REVOLUTE JOINT
        if joint_type == "revolute":
            assert len(default_values) == 3
            joint = Joints.RevoluteJoint(name, frame, *default_values)
            self.joint_array.append(joint)
            self.joint_dict[name] = len(self.joint_array) - 1
            return True

        # PRISMATIC JOINT
        if joint_type == "prismatic":
            assert len(default_values) == 3
            joint = Joints.PrismaticJoint(name, frame, *default_values)
            self.joint_array.append(joint)
            self.joint_dict[name] = len(self.joint_array) - 1
            return True

        # FLOATING JOINT
        if joint_type == "floating":
            assert len(default_values) == 3
            joint = Joints.FloatingJoint(name, frame, *default_values)
            self.joint_array.append(joint)
            self.joint_dict[name] = len(self.joint_array) - 1
            return True

        # RIGID JOINT
        if joint_type == "rigid":
            joint = Joints.RigidJoint(name, frame)
            self.joint_array.append(joint)
            self.joint_dict[name] = len(self.joint_array) - 1
            return True

        logging.error(f"'joint_type' {joint_type} is not supported. No joint was added.")
        return False

    def addBody (self, name, mass, inertia_matrix,
            predecessor_frames, successor_frames, parent = None):
        """ 
        Add new body, whose parent is the previously added body, unless
        'parent' is set to real parent's number.

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

        parent : int / string, default = None
            Index / Name of the parent for the newly created body. If 'parent'
            is None, the body's parent is the last body.

        """

        if type(parent) == str:
            parent_index = self.body_dict[parent]
        else:
            if parent != None:
                parent_index = parent
            else:
                parent_index = None

        # Check whether given name has already been used
        if name in self.body_dict:
            logging.error(f"Body {name} already exists.")
            return False

        body = Bodies.Body(name, mass, inertia_matrix,
                predecessor_frames, successor_frames)

        if body.body_number != 0:
            if parent_index == None:
                parent_index = self.bodies[-1].body_number
            else:
                if not (parent_index >= 0 and parent_index < len(self.bodies)):
                    logging.error(f"Unknown index.")
                    return False

            # add parent_index to the end of the parent array and update
            # number of robot's bodies
            self.parent_array.append(parent_index)

        self.bodies.append(body)
        self.body_dict[name] = len(self.bodies) - 1
        return True

    def pluckerTransformationMatrix(self, from_c, to_c):
        """
        Compute Plucker motion transformation matrix, where 'from' indicates
        the 'from' body's coordinate frame and 'to' indicates 'to' body's 
        coordinate frame.

        Parameters
        ----------
        from_c : int / string
            int : 
                Index of body, whose coordinate system is used to compute the 
                plucker transformation matrix ('from_c')_T_('to_c')
            string :
                Name of the body, whose coordinate frame we want to use.

        to_c : int / string
            int:
                Index of body, whose coordinate system is used to compute the 
                plucker transformation matrix ('from_c')_T_('to_c')
            string:
                Name of the body, whose coordinate frame we want to use.

        Returns
        -------
        transformation_matrix : np.array(shape = (6,6))
            Plucker motion transfromation matrix, from 'from_c' to 'to_c'.
        """

        # check type, if string, choose the appropriate body number
        if type(from_c) == str:
            from_c = self.body_dict[from_c]
        if type(to_c) == str:
            to_c = self.body_dict[to_c]

        if not (from_c >= 0 and from_c < len(self.bodies)):
            logging.error(f"Body {from_c} does not exist.")
            return None

        if not (to_c >= 0 and to_c < len(self.bodies)):
            logging.error(f"Body {to_c} does not exist.")
            return None

        from_parents = []
        i = from_c 
        from_parents.append(i)

        # get all grandparents of body 'from_c'
        while True:
            if i == 0:
                break
            from_parents.append(self.parent_array[i-1])
            i = self.parent_array[i-1]
        to_parents = []
        i = to_c
        to_parents.append(i)

        # get all grandparents of body 'to_c'
        while True:
            if i == 0:
                break
            to_parents.append(self.parent_array[i-1])
            i = self.parent_array[i-1]
        grandparent = 0

        # find the first grandparent, that is common to both the bodies
        for i in range(1, min(len(to_parents), len(from_parents)) + 1):
            if to_parents[-i] == from_parents[-i]:
                grandparent = to_parents[-i]
                continue
            else:
                break
        
        # these variables are not needed anymore
        del to_parents
        del from_parents
    
        # Compute plucker transformation matrix from 'grandparent' to 'from_c'
        X_gp_f = np.eye(6)
        i = from_c
        while True:
            if i == grandparent:
                break
            X_J = self.joint_array[i - 1].X_J()
            joint_frame_name = self.joint_array[i-1].frame_name()
            X_T = self.bodies[self.parent_array[i-1]].X_T[joint_frame_name]
            X_gp_f = X_T @ (X_J @ X_gp_f)
            i = self.parent_array[i-1]

        # Compute plucker transformation matrix from 'grandparent' to 'to_c'
        X_gp_t = np.eye(6)
        i = to_c
        while True:
            if i == grandparent:
                break
            X_J = self.joint_array[i - 1].X_J()
            joint_frame_name = self.joint_array[i-1].frame_name()
            X_T = self.bodies[self.parent_array[i-1]].X_T[joint_frame_name]
            X_gp_t = X_T @ (X_J @ X_gp_t)
            i = self.parent_array[i-1]

        # ('from_c')_T_('gp') = (('gp')_T_('from_c'))^-1
        X_f_gp = Plucker.pluckerMotionInverse(X_gp_f)

        # ('from_c')_T_('to_c') = ('from_c')_T_('gp') @ ('gp')_T_('to_c')
        return X_f_gp @ X_gp_t

    def setJointVariables(self, joint_variable_dict):
        """
        Given a dictionary of joint names and the required joint variables
        in a dictionary, the requested joint variables are set.
        If some varibles are wished not to be changed, pass None as value.

        Parameters
        ----------
        joint_variable_dict: dict[string : dict[string : list[int]]]
            Dictionary of names of joints, whose joint variables are to
            be modified. The dictionary key is the name of the joint, the 
            value is o dictionary with three keys: "q", "q_dot", "q_ddot".
            The values of this second dictionary are lists, whose values
            are to be set. If some variables are wished not to be modified,
            set it to None.
            If none of the joint variables in a certain category are to be
            modified, pass None instead of list as value.

            Example: {
                      "FR1" : {
                                "q" :     None,
                                "q_dot":  [None, 12],
                                "q_ddot": [1,2]
                              }

                      "FR2" : {
                                "q" :     [1],
                                "q_dot":  [None],
                                "q_ddot": None
                              }
                     }
        """

        for joint_name in joint_variable_dict:

            # Check whether joint with name 'joint_name' exists
            if not joint_name in self.joint_dict:
                logging.error(f"Joint {joint_name} does not exist.")
                continue

            joint_index = self.joint_dict[joint_name]
            joint_variables = joint_variable_dict[joint_name]

            # Check whether q has been set
            if not "q" in joint_variables:
                logging.error(f"q for joint {joint_name} has not been set.")
                continue
            
            # Check whether q_dot has been set
            if not "q_dot" in joint_variables:
                logging.error(f"q_dot for joint {joint_name} has not been set.")
                continue

            # Check whether q_ddot has been set
            if not "q_ddot" in joint_variables:
                logging.error(f"q_ddot for joint {joint_name} has not been set.")
                continue

            self.joint_array[joint_index].setJointVariables(
                q = joint_variables["q"],
                q_dot = joint_variables["q_dot"],
                q_ddot = joint_variables["q_ddot"]
                )

    def getJointVariables(self):
        """
        Given a dictionary of joint names and the required joint variables
        in a dictionary, the requested joint variables are set.
        If some varibles are wished not to be changed, pass None as value.

        Returns
        -------
        joint_variable_dict: dict[string : dict[string : list[int]]]
            Dictionary of names of joints, whose joint variable.
            The dictionary key is the name of the joint, the 
            value is o dictionary with three keys: "q", "q_dot", "q_ddot".
            The values of this second dictionary are lists of current joint
            variable values. If some variables are symblic, the value in
            the list is set to None.

            Example: {
                      "FR1" : {
                                "q" :     [None, None]
                                "q_dot":  [None, 12],
                                "q_ddot": [1,2]
                              }

                      "FR2" : {
                                "q" :     [1],
                                "q_dot":  [None],
                                "q_ddot": [None]
                              }
                     }
        """
        joint_variable_dict = dict()

        for i in range(len(self.joint_array)):
            joint = self.joint_array[i]
            
            if(joint.joint_type() != "rigid"): 
                joint_name = joint.name
                joint_variables = joint.getJointVariables()
                joint_variable_dict[joint_name] = joint_variables

        return joint_variable_dict

    def newtonEulerID(self, external_forces = {}, gravity_compensation = True):
        """
        Compute robot's inverse dynamics given external forces acting on bodies.
        'external_forces' is a dictionary with names of bodies as keys
        and a tuple (force, local) as a value. If 'local' is set to True,
        the given force is expressed in the local frame.  Otherwise it's
        expressed in the global frame. 'force' is a Plucker force vector.

        If gravity_compensation is set to 'True', the resulting forces will 
        also try to compensate gravity.

        Parameters
        ----------
        external_forces : dict[string : tuple[np.array(shape = (6,)), bool]],
                          default = {}
            Dictionary, whose keys are names of bodies on which given external
            forces are exerted.
        gravity_compensation : bool, default = True
        """

        if gravity_compensation:
            # body acceleration, set to -gravity for gravity compensation
            a = [-self.gravity]
        else:
            a = [np.zeros(shape = (6,))]

        # body velocity
        v = [np.zeros(shape = (6,))]

        # forces acting on bodies
        f = list()

        for i in range(1,len(self.bodies)):
            joint = self.joint_array[i-1]

            a.append(np.zeros(shape = (6,)))
            v.append(np.zeros(shape = (6,)))

            # Joint Plucker transformation matrix
            X_J = joint.X_J()

            # velocity of body i with respect to its parent
            v_j = joint.v_j()

            # accelertaion of body i with respect to its parent
            a_j = joint.a_j()

            # Plucker transformation matrix, from child to its parent
            parent_index = self.parent_array[i - 1]
            X_i_pi = self.pluckerTransformationMatrix(i, parent_index)
            
            # overall velocity
            v[-1] = X_i_pi.dot(v[parent_index]) + v_j

            # motion and force cross product matrices
            v_cross_m = Plucker.motionCrossProductMatrix(v[-1])
            v_cross_f = Plucker.forceCrossProductMatrix(v[-1])

            # overall acceleration
            a[-1] = X_i_pi.dot(a[parent_index]) + joint.a_j() +\
                    joint.c_j() + v_cross_m.dot(v_j)

            plucker_inertia_matrix = self.bodies[i].getPluckerInertiaMatrix()

            generalized_force = plucker_inertia_matrix.dot(a[-1])
            generalized_force += v_cross_f @ (plucker_inertia_matrix.dot(v[-1]))

            body_name = self.bodies[i].name
            external_force_given = body_name in external_forces
            if external_force_given:
                external_force = external_forces[body_name][0]
                local = external_forces[body_name][1]

                if local:
                    X_i_0 = self.pluckerTransformationMatrix(i, 0)
                    X_i_0[3:,:3] = 0 # we only need orientation
                    X_force = Plucker.pluckerMotionToForce(X_i_0)
                else:
                    # X_i_i = self.pluckerTransformationMatrix(i, i)
                    # X_force = Plucker.pluckerMotionToForce(X_i_i)
                    X_force = np.eye(6)

                generalized_force -= X_force.dot(external_force)

            f.append(generalized_force)

        generalized_forces = {}
        for i in range(len(self.bodies)-1, 0, -1):
            F_i = f[i - 1]
            joint = self.joint_array[i-1]
            S_i = joint.motion_subspace()
            generalized_force = S_i @ F_i

            parent = self.parent_array[i-1]
            X_pi_i = self.pluckerTransformationMatrix(parent, i)
            X_pi_i_f = Plucker.pluckerMotionToForce(X_pi_i)
            if parent != 0:
                f[parent-1] = f[parent-1] + X_pi_i_f.dot(F_i)

            if joint.joint_type() != "rigid":
                joint_name = joint.name
                extracted_values = joint.extractControllableParameter(
                        generalized_force)
                generalized_forces[joint_name] = extracted_values

        return generalized_forces

    def BiasForceVector(self, set_to_symbolic = True):
        """
        Compute the C vector of bias forces. This vector is part of the 
        manipulator equation M * q_ddot + C = F, where F is a vector 
        of generalized forces.
        The vector C is computed using the Newton-Euler recursive inverse
        dynamics algorithm.

        Params
        ------
        set_to_symbolic : bool
            If 'True', all the values will be set to their symbolic representations.

        Returns
        -------
        bias_force_vector : np.array(shape(c,)), where c is number of controllable parameters
            Bias force vector.
        """

        # the current joint varibles will be modified, this will save a
        # copy of the current joint variables and at the end of this function
        # the joint variables will be set to their original values
        joint_variable_copy = self.getJointVariables()

        # Determine demension of the output vector
        c = 0

        current_pos = 0
        param_pos = dict()

        for i in range(len(self.joint_array)):
            joint = self.joint_array[i]
            if(joint.joint_type() != "rigid"):
                controllable_parameters = joint.getControllableParameters()
                c += len(controllable_parameters)

                if set_to_symbolic:
                    q = [None for _ in range(len(controllable_parameters))]
                    q_dot = [None for _ in range(len(controllable_parameters))]
                else:
                    joint_name = joint.name
                    q = joint_variable_copy[joint_name]["q"]
                    q_dot = joint_variable_copy[joint_name]["q_dot"]

                q_ddot = [0 for _ in range(len(controllable_parameters))]

                modified_joint_variables = dict()
                modified_joint_variables["q"] = q
                modified_joint_variables["q_dot"] = q_dot
                modified_joint_variables["q_ddot"] = q_ddot

                mjv_named = dict()
                mjv_named[joint.name] = modified_joint_variables

                # set joint variables to symbolic values
                self.setJointVariables(mjv_named)

                for j in range(len(controllable_parameters)):
                    controllable_parameter = "tau_" + controllable_parameters[j]
                    param_pos[controllable_parameter] = current_pos
                    current_pos += 1

        bias_force_vector = np.full(shape = (c,), fill_value = None)

        # no external forces, gravity compensation is turned on
        generalized_forces = self.newtonEulerID(gravity_compensation = True)

        for i in range(len(generalized_forces)):
            joint_name = list(generalized_forces.keys())[i]
            generalized_force_names = list(generalized_forces[joint_name].keys())

            for j in range(len(generalized_force_names)):
                position = param_pos[generalized_force_names[j]]
                bias_force_vector[position] = \
                        generalized_forces[joint_name][generalized_force_names[j]]

        # set joint variables to their original values
        self.setJointVariables(joint_variable_copy)

        return np.array(bias_force_vector)

    def MassMatrix(self, set_to_symbolic = True):
        """
        Compute robot's mass matrix via inverse dynamics.
        The mass matrix M is part of the manipulator equation M * q_ddot + C = F,
        where F is a vector of generalized forces.

        Params
        ------
        set_to_symbolic : bool, default = True
            If 'True', all the values will be set to their symbolic representations.

        Returns
        -------
        mass_matrix : np.array(shape = (m,m)), where m is the number of controllable
                                             parameters
        """

        # the current joint varibles will be modified, this will save a
        # copy of the current joint variables and at the end of this function
        # the joint variables will be set to their original values
        joint_variable_copy = self.getJointVariables()

        # mass matrix dimension
        m = 0

        current_pos = 0
        param_pos = dict()

        for i in range(len(self.joint_array)):
            joint = self.joint_array[i]
            if(joint.joint_type() != "rigid"):
                controllable_parameters = joint.getControllableParameters()
                joint_name = joint.name
                m += len(controllable_parameters)
                if set_to_symbolic:
                    q = [None for _ in range(len(controllable_parameters))]
                else:
                    q = joint_variable_copy[joint_name]["q"]
                q_dot = [0 for _ in range(len(controllable_parameters))]
                q_ddot = [0 for _ in range(len(controllable_parameters))]

                modified_joint_variables = dict()
                modified_joint_variables["q"] = q
                modified_joint_variables["q_dot"] = q_dot
                modified_joint_variables["q_ddot"] = q_ddot

                mjv_named = dict()
                mjv_named[joint.name] = modified_joint_variables

                # set joint variables to symbolic values
                self.setJointVariables(mjv_named)

                for j in range(len(controllable_parameters)):
                    controllable_parameter = "tau_" + controllable_parameters[j]
                    param_pos[controllable_parameter] = current_pos
                    current_pos += 1

        mass_matrix = np.full(shape = (m,m), fill_value = None)
        joint_variable_copy2 = self.getJointVariables()

        current_column = 0
        for i in range(len(self.joint_array)):
            joint = self.joint_array[i]
            if(joint.joint_type() != "rigid"):
                controllable_parameters = joint.getControllableParameters()
                joint_name = joint.name
                for j in range(len(controllable_parameters)):
                    joint_variable_copy2 = self.getJointVariables()

                    q = joint_variable_copy2[joint_name]["q"]
                    q_dot = joint_variable_copy2[joint_name]["q_dot"]
                    q_ddot = joint_variable_copy2[joint_name]["q_ddot"]
                    q_ddot[j] = 1.0

                    modified_joint_variables = dict()
                    modified_joint_variables["q"] = q
                    modified_joint_variables["q_dot"] = q_dot
                    modified_joint_variables["q_ddot"] = q_ddot
                    mjv_named = dict()
                    mjv_named[joint_name] = modified_joint_variables

                    # set joint accelerations to zero, except for joint 'i',
                    # whose joint acceleration is set to 1 to get the
                    # 'i'th column of the mass matrix
                    self.setJointVariables(mjv_named)

                    # bias force vector, that is gravity terms
                    bias_force_vector = self.BiasForceVector(set_to_symbolic = False)

                    # tau
                    generalized_forces = self.newtonEulerID()
                    tau = np.full(shape = (m,), fill_value = None)

                    for k in range(len(generalized_forces)):
                        joint_name2 = list(generalized_forces.keys())[k]
                        generalized_force_names = list(generalized_forces[joint_name2].keys())

                        for l in range(len(generalized_force_names)):
                            position = param_pos[generalized_force_names[l]]
                            tau[position] = \
                                    generalized_forces[joint_name2][generalized_force_names[l]]

                    mass_matrix[:,current_column] = tau - bias_force_vector
                    current_column += 1

                    # set joint acceleration back to zero
                    mjv_named[joint_name]["q_ddot"][j] = 0
                    self.setJointVariables(mjv_named)

        # set joint variables to the original values
        self.setJointVariables(joint_variable_copy)
        return mass_matrix

    def subtreeBodies(self, body_i):
        """
        Return set of bodies supported by body 'body_i'

        Parameters
        ----------
        body_i : int / string

        Returns
        -------
        body_set : set(int)
            Set of bodies, which are supported by body 'body_i'
        """
        if type(body_i) == str:
            body_i = self.body_dict[body_i]
        body_set = set()
        body_set |= {body_i}

        stack = [body_i]

        while len(stack) != 0:
            parent = stack.pop()
            for i in range(len(self.parent_array)):
                if self.parent_array[i] == parent:
                    if not i + 1 in body_set:
                        stack.append(i + 1)
                        body_set |= {i + 1}
        return body_set
        
    def generateGraph(self, output_file = "robot.png"):
        """ 
        Generate graph that shows robot's structure. The graph is saved
        into a file 'output_file'.

        Parameters
        ----------
        output_file : string, default = "robot.png"
            Name of the output file. The recommended output format is png.
        """

        # import needed library
        graph = pygraphviz.AGraph(directed = True, strict = False)

        for i in range(1, len(self.bodies)):
            # parent body index
            parent_body_i = self.parent_array[i - 1]
            parent_body_name = self.bodies[parent_body_i].name
            parent_body_name += " [{}]".format(self.bodies[parent_body_i].body_number)

            current_body_name = self.bodies[i].name
            current_body_name += " [{}]".format(self.bodies[i].body_number)
            current_joint = self.joint_array[i - 1]

            label = current_joint.name + " [{}]".format(current_joint.joint_number())
            label += ", " + str(current_joint.getControllableParameters())

            graph.add_edge(parent_body_name, current_body_name,
                    label = label)

        graph.layout(prog="dot")
        graph.draw(output_file)

    def summary(self):
        """ 
        Print out a short summary of the robot's structure, joints and bodies.
        """

        print("==============================================================")
        print("Robot name:", self.name)
        print()
        print("--------------------------------------------------------------")
        print()
        print("JOINTS:")
        for joint in self.joint_array:
            print(joint.name + ": ", joint.joint_type())
            print("Controllable parameters:", joint.getControllableParameters())
            print()
        print("--------------------------------------------------------------")
        print()
        print("BODIES")
        for body in self.bodies:
            print("Body name:", body.name)
        print()

        print("==============================================================")


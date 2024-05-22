'''In this exercise you need to implement forward kinematics for NAO robot

* Tasks:
    1. complete the kinematics chain definition (self.chains in class ForwardKinematicsAgent)
       The documentation from Aldebaran is here:
       http://doc.aldebaran.com/2-1/family/robots/bodyparts.html#effector-chain
    2. implement the calculation of local transformation for one joint in function
       ForwardKinematicsAgent.local_trans. The necessary documentation are:
       http://doc.aldebaran.com/2-1/family/nao_h21/joints_h21.html
       http://doc.aldebaran.com/2-1/family/nao_h21/links_h21.html
    3. complete function ForwardKinematicsAgent.forward_kinematics, save the transforms of all body parts in torso
       coordinate into self.transforms of class ForwardKinematicsAgent

* Hints:
    1. the local_trans has to consider different joint axes and link parameters for different joints
    2. Please use radians and meters as unit.
'''

# add PYTHONPATH
import os
import sys
import numpy as np
sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'joint_control'))

from numpy.matlib import matrix, identity, cos, sin

from recognize_posture import PostureRecognitionAgent


class ForwardKinematicsAgent(PostureRecognitionAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(ForwardKinematicsAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.transforms = {n: identity(4) for n in self.joint_names}

        # chains defines the name of chain and joints of the chain
        self.chains = {'Head': ['HeadYaw', 'HeadPitch'],
                       # YOUR CODE HERE
                        'LeftArm': ['LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll', 'LWristYaw'],
                        'RightArm': ['RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll', 'RWristYaw'],
                        'LeftLeg': ['LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch', 'LAnklePitch', 'LAnkleRoll'],
                        'RightLeg': ['RHipYawPitch', 'RHipRoll', 'RHipPitch', 'RKneePitch', 'RAnklePitch', 'RAnkleRoll']
        
                       }

    def think(self, perception):
        self.forward_kinematics(perception.joint)
        return super(ForwardKinematicsAgent, self).think(perception)

    def local_trans(self, joint_name, joint_angle):
        '''calculate local transformation of one joint

        :param str joint_name: the name of joint
        :param float joint_angle: the angle of joint in radians
        :return: transformation
        :rtype: 4x4 matrix
        '''
        T = identity(4)
        # YOUR CODE HERE

        # Define the axis of rotation for each joint and their link lengths
        if joint_name in ['HeadYaw', 'HeadPitch']:
            axis = 'z' if joint_name == 'HeadYaw' else 'y'
        elif joint_name in ['LShoulderPitch', 'RShoulderPitch']:
            axis = 'y'
        elif joint_name in ['LShoulderRoll', 'RShoulderRoll']:
            axis = 'x'
        elif joint_name in ['LElbowYaw', 'RElbowYaw', 'LElbowRoll', 'RElbowRoll', 'LWristYaw', 'RWristYaw']:
            axis = 'z' if 'Yaw' in joint_name else 'x'
        elif joint_name in ['LHipYawPitch', 'RHipYawPitch']:
            axis = 'z'
        elif joint_name in ['LHipRoll', 'RHipRoll']:
            axis = 'x'
        elif joint_name in ['LHipPitch', 'RHipPitch', 'LKneePitch', 'RKneePitch', 'LAnklePitch', 'RAnklePitch']:
            axis = 'y'
        elif joint_name in ['LAnkleRoll', 'RAnkleRoll']:
            axis = 'x'

        # Create rotation matrix based on the axis
        if axis == 'x':
            T[:3, :3] = [
                [1, 0, 0],
                [0, cos(joint_angle), -sin(joint_angle)],
                [0, sin(joint_angle), cos(joint_angle)]
            ]
        elif axis == 'y':
            T[:3, :3] = [
                [cos(joint_angle), 0, sin(joint_angle)],
                [0, 1, 0],
                [-sin(joint_angle), 0, cos(joint_angle)]
            ]
        elif axis == 'z':
            T[:3, :3] = [
                [cos(joint_angle), -sin(joint_angle), 0],
                [sin(joint_angle), cos(joint_angle), 0],
                [0, 0, 1]
            ]

        return T

    def forward_kinematics(self, joints):
        '''forward kinematics

        :param joints: {joint_name: joint_angle}
        '''
        for chain_joints in self.chains.values():
            T = identity(4)
            for joint in chain_joints:
                angle = joints[joint]
                Tl = self.local_trans(joint, angle)
                # YOUR CODE HERE
                T = T @ Tl  # Matrix multiplication
                self.transforms[joint] = T

if __name__ == '__main__':
    agent = ForwardKinematicsAgent()
    agent.run()

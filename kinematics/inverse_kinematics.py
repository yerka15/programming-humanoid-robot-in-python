'''In this exercise you need to implement inverse kinematics for NAO's legs

* Tasks:
    1. solve inverse kinematics for NAO's legs by using analytical or numerical method.
       You may need documentation of NAO's leg:
       http://doc.aldebaran.com/2-1/family/nao_h21/joints_h21.html
       http://doc.aldebaran.com/2-1/family/nao_h21/links_h21.html
    2. use the results of inverse kinematics to control NAO's legs (in InverseKinematicsAgent.set_transforms)
       and test your inverse kinematics implementation.
'''


from forward_kinematics import ForwardKinematicsAgent
from numpy.matlib import identity
import numpy as np


class InverseKinematicsAgent(ForwardKinematicsAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(InverseKinematicsAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
    
    def inverse_kinematics(self, effector_name, transform):
        '''solve the inverse kinematics

        :param str effector_name: name of end effector, e.g. LLeg, RLeg
        :param transform: 4x4 transform matrix
        :return: list of joint angles
        '''
        joint_angles = []
        # YOUR CODE HERE
        # Extract the desired position from the transform matrix
        target_position = transform[:3, 3]
        
        # Assuming the leg is represented by six joints: [HipYawPitch, HipRoll, HipPitch, KneePitch, AnklePitch, AnkleRoll]
        # Use the given links length
        L1 = 0.085  # Hip to knee length
        L2 = 0.090  # Knee to ankle length

        # Simplified inverse kinematics for the leg
        # Calculate KneePitch
        x, y, z = target_position
        D = (x**2 + y**2 + (z - L1)**2 - L1**2 - L2**2) / (2 * L1 * L2)
        if D > 1 or D < -1:
            raise ValueError("Target position is out of reach")
        
        KneePitch = np.arctan2(np.sqrt(1 - D**2), D)
        if np.isnan(KneePitch):
            KneePitch = 0.0
        
        # Calculate HipPitch
        L3 = np.sqrt(x**2 + y**2 + (z - L1)**2)
        alpha = np.arctan2(z - L1, np.sqrt(x**2 + y**2))
        beta = np.arctan2(L2 * np.sin(KneePitch), L1 + L2 * np.cos(KneePitch))
        HipPitch = alpha + beta
        
        # Calculate AnklePitch
        AnklePitch = np.arctan2(L1 * np.sin(HipPitch) - z, L1 * np.cos(HipPitch) + L2 * np.cos(KneePitch) - np.sqrt(x**2 + y**2))
        
        # Calculate HipYawPitch and HipRoll
        HipYawPitch = np.arctan2(y, x)
        HipRoll = 0.0  # Assuming no roll
        
        # Calculate AnkleRoll
        AnkleRoll = 0.0  # Assuming no roll
        
        joint_angles = [HipYawPitch, HipRoll, HipPitch, KneePitch, AnklePitch, AnkleRoll]

        return joint_angles

    def set_transforms(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
        joint_angles = self.inverse_kinematics(effector_name, transform)
        self.keyframes = ([], [], [])  # the result joint angles have to fill in
        self.keyframes = (
            ['LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch', 'LAnklePitch', 'LAnkleRoll'],
            [[0.0], [0.0], [0.0], [0.0], [0.0], [0.0]],  # assuming keyframes at time 0
            [[angle] for angle in joint_angles]
        )

if __name__ == '__main__':
    agent = InverseKinematicsAgent()
    # test inverse kinematics
    T = identity(4)
    T[-1, 1] = 0.05
    T[-1, 2] = -0.26
    agent.set_transforms('LLeg', T)
    agent.run()

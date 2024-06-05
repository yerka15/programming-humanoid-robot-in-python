'''In this file you need to implement remote procedure call (RPC) server

* There are different RPC libraries for python, such as xmlrpclib, json-rpc. You are free to choose.
* The following functions have to be implemented and exported:
 * get_angle
 * set_angle
 * get_posture
 * execute_keyframes
 * get_transform
 * set_transform
* You can test RPC server with ipython before implementing agent_client.py
'''

# add PYTHONPATH
import os
import sys
import xmlrpc.server
from threading import Thread
import time

sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'kinematics'))

from inverse_kinematics import InverseKinematicsAgent


class ServerAgent(InverseKinematicsAgent):
    '''ServerAgent provides RPC service
    '''
    # YOUR CODE HERE
    def __init__(self, simspark_ip='localhost', simspark_port=3100, teamname='DAInamite', player_id=0, sync_mode=True):
        super(ServerAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.rpc_server = xmlrpc.server.SimpleXMLRPCServer(('localhost', 8000), allow_none=True)
        self.rpc_server.register_instance(self)
        self.server_thread = Thread(target=self.rpc_server.serve_forever)
        self.server_thread.start()
    
    def get_angle(self, joint_name):
        '''get sensor value of given joint'''
        # YOUR CODE HERE
        if joint_name in self.joint_names:
            return self.perception.joint.get(joint_name, 0.0)
        return None
    
    def set_angle(self, joint_name, angle):
        '''set target angle of joint for PID controller
        '''
        # YOUR CODE HERE
        if joint_name in self.target_joints:
            self.target_joints[joint_name] = angle
            return True
        return False

    def get_posture(self):
        '''return current posture of robot'''
        # YOUR CODE HERE
        return self.posture

    def execute_keyframes(self, keyframes):
        '''excute keyframes, note this function is blocking call,
        e.g. return until keyframes are executed
        '''
        # YOUR CODE HERE
        self.keyframes = keyframes
        while not self.keyframes_executed():
            time.sleep(0.1)
        return True

    def get_transform(self, name):
        '''get transform with given name
        '''
        # YOUR CODE HERE
        return self.transforms.get(name, None) 

    def set_transform(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
        joint_angles = self.inverse_kinematics(effector_name, transform)
        self.set_joint_angles(effector_name, joint_angles)
        return True

if __name__ == '__main__':
    agent = ServerAgent()
    agent.run()


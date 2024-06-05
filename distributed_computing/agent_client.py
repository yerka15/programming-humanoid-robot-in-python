'''In this file you need to implement remote procedure call (RPC) client

* The agent_server.py has to be implemented first (at least one function is implemented and exported)
* Please implement functions in ClientAgent first, which should request remote call directly
* The PostHandler can be implement in the last step, it provides non-blocking functions, e.g. agent.post.execute_keyframes
 * Hints: [threading](https://docs.python.org/2/library/threading.html) may be needed for monitoring if the task is done
'''
import xmlrpc.client
import threading
import weakref

class PostHandler(object):
    '''the post hander wraps function to be excuted in paralle
    '''
    def __init__(self, obj):
        self.proxy = weakref.proxy(obj)

    def execute_keyframes(self, keyframes):
        '''non-blocking call of ClientAgent.execute_keyframes'''
        # YOUR CODE HERE
        threading.Thread(target=self.proxy.execute_keyframes, args=(keyframes,)).start()

    def set_transform(self, effector_name, transform):
        '''non-blocking call of ClientAgent.set_transform'''
        # YOUR CODE HERE
        threading.Thread(target=self.proxy.set_transform, args=(effector_name, transform,)).start()



class ClientAgent(object):
    '''ClientAgent request RPC service from remote server
    '''
    # YOUR CODE HERE
    def __init__(self):
        self.server = xmlrpc.client.ServerProxy('http://localhost:8000')
        self.post = PostHandler(self)
    
    def get_angle(self, joint_name):
        '''get sensor value of given joint'''
        # YOUR CODE HERE
        return self.server.get_angle(joint_name)
    
    def set_angle(self, joint_name, angle):
        '''set target angle of joint for PID controller
        '''
        # YOUR CODE HERE
        return self.server.set_angle(joint_name, angle)

    def get_posture(self):
        '''return current posture of robot'''
        # YOUR CODE HERE
        return self.server.get_posture()

    def execute_keyframes(self, keyframes):
        '''excute keyframes, note this function is blocking call,
        e.g. return until keyframes are executed
        '''
        # YOUR CODE HERE
        return self.server.execute_keyframes(keyframes)

    def get_transform(self, name):
        '''get transform with given name
        '''
        # YOUR CODE HERE
        return self.server.get_transform(name)

    def set_transform(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
        return self.server.set_transform(effector_name, transform)

if __name__ == '__main__':
    agent = ClientAgent()
    # TEST CODE HERE
    
    # Test get_angle
    joint_name = 'HeadYaw'
    print(f'Angle of {joint_name}: {agent.get_angle(joint_name)}')

    # Test set_angle
    print(f'Setting angle of {joint_name} to 0.5')
    agent.set_angle(joint_name, 0.5)

    # Test get_posture
    print(f'Current posture: {agent.get_posture()}')

    # Test execute_keyframes (blocking call)
    keyframes = (['HeadYaw'], [[0.0, 1.0]], [[0.0, 0.5]])
    print('Executing keyframes...')
    agent.execute_keyframes(keyframes)
    print('Keyframes executed.')

    # Test get_transform
    name = 'HeadYaw'
    print(f'Transform of {name}: {agent.get_transform(name)}')

    # Test set_transform (blocking call)
    from numpy.matlib import identity
    transform = identity(4)
    transform[-1, 1] = 0.05
    transform[-1, 2] = -0.26
    print(f'Setting transform of LLeg to {transform}')
    agent.set_transform('LLeg', transform)

    # Test non-blocking execute_keyframes
    print('Executing keyframes non-blocking...')
    agent.post.execute_keyframes(keyframes)
    print('Keyframes execution started (non-blocking).')

    # Test non-blocking set_transform
    print(f'Setting transform of LLeg to {transform} (non-blocking)')
    agent.post.set_transform('LLeg', transform)
    print('Transform set (non-blocking).')



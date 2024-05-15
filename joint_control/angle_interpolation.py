'''In this exercise you need to implement an angle interploation function which makes NAO executes keyframe motion

* Tasks:
    1. complete the code in `AngleInterpolationAgent.angle_interpolation`,
       you are free to use splines interploation or Bezier interploation,
       but the keyframes provided are for Bezier curves, you can simply ignore some data for splines interploation,
       please refer data format below for details.
    2. try different keyframes from `keyframes` folder

* Keyframe data format:
    keyframe := (names, times, keys)
    names := [str, ...]  # list of joint names
    times := [[float, float, ...], [float, float, ...], ...]
    # times is a matrix of floats: Each line corresponding to a joint, and column element to a key.
    keys := [[float, [int, float, float], [int, float, float]], ...]
    # keys is a list of angles in radians or an array of arrays each containing [float angle, Handle1, Handle2],
    # where Handle is [int InterpolationType, float dTime, float dAngle] describing the handle offsets relative
    # to the angle and time of the point. The first Bezier param describes the handle that controls the curve
    # preceding the point, the second describes the curve following the point.
'''


from pid import PIDAgent
from keyframes import hello
import numpy as np


class AngleInterpolationAgent(PIDAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(AngleInterpolationAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.keyframes = ([], [], [])

    def think(self, perception):
        target_joints = self.angle_interpolation(self.keyframes, perception)
        target_joints['RHipYawPitch'] = target_joints['LHipYawPitch'] # copy missing joint in keyframes
        self.target_joints.update(target_joints)
        return super(AngleInterpolationAgent, self).think(perception)

    def angle_interpolation(self, keyframes, perception):
        target_joints = {}
        # YOUR CODE HERE
    
        # Unpack keyframes data
        joints, times, keys = keyframes
    
        # Compute current time (adjust based on your initialization logic)
        current_time = perception.time - self.init_timestamp
    
        # Cubic Bezier Interpolation
        for joint_index in range(len(joints)):
            # Get correct keyframe data
            keyframe_angles = keys[joint_index]
            keyframe_times = times[joint_index]
            
            # Iterate through keyframes
            for i in range(len(keyframe_angles) - 1):
                if keyframe_times[i] <= current_time < keyframe_times[i + 1]:
                    P0 = keyframe_angles[i][0]
                    P3 = keyframe_angles[i + 1][0]
                    
                    # Compute control points
                    P1 = P0 + keyframe_angles[i][1][2]
                    P2 = P3 + keyframe_angles[i][2][2]
                    
                    # Calculate interpolation parameter
                    t = (current_time - keyframe_times[i]) / (keyframe_times[i + 1] - keyframe_times[i])
                    
                    # Perform cubic Bezier interpolation
                    target_joints[joints[joint_index]] = (1 - t) ** 3 * P0 + 3 * (1 - t) ** 2 * t * P1 + \
                                                        3 * (1 - t) * t ** 2 * P2 + t ** 3 * P3
                    break  # Exit loop if interpolation is done between two keyframes
            
        # Ensure symmetry
        if 'LHipYawPitch' in target_joints:
            target_joints['RHipYawPitch'] = target_joints['LHipYawPitch']
        

        return target_joints

if __name__ == '__main__':
    agent = AngleInterpolationAgent()
    agent.keyframes = hello()  # CHANGE DIFFERENT KEYFRAMES
    agent.run()

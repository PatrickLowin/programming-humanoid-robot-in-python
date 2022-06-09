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


from configparser import NoSectionError
from typing import KeysView
from pid import PIDAgent
from keyframes import leftBackToStand as hello
import numpy as np
from scipy.interpolate import UnivariateSpline
from scipy.special import comb


class AngleInterpolationAgent(PIDAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(AngleInterpolationAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.keyframes = ([], [], [])
        self.start = None

    def think(self, perception):
        target_joints = self.angle_interpolation(self.keyframes, perception)
        self.target_joints.update(target_joints)
        return super(AngleInterpolationAgent, self).think(perception)

    #Bezier Finnem
    def bezier(self, p0, p1,p2,p3, time):
        p0x,p0y = p0
        p1x,p1y = p1
        p2x,p2y = p2
        p3x,p3y = p3
        bezierMatrix = np.array([[1,0,0,0],[-3,3,0,0],[3,-6,3,0],[-1,3,-3,1]])
        x = np.array([p0x,p1x,p2x,p3x])
        y = np.array([p0y,p1y,p2y,p3y])
        
        #getting t value candidates (solutions for the polynomial) for current Time
        coefficientsX = np.dot(bezierMatrix, x)
        coefficientsX[0] -= time
        candidates = np.polynomial.polynomial.polyroots(coefficientsX)
        
        #finding correct candidate for t (t has to be in [0,1])
        candidates = [x.real for x in candidates if -(1e-4)<=x.real<=1+(1e-4) and x.imag == 0] #error margin uncertain
        
        t = candidates[0]
        
        t = np.clip(t,0,1)
            
        #getting y values
        coefficientsY = np.dot(bezierMatrix, y)
        result = np.dot(np.array([1, t, t**2, t**3]),coefficientsY)
        return result


    def angle_interpolation(self, keyframes, perception):

        if self.start is None:
            self.start = perception.time
        #split it up
        names, times, keys = keyframes

        if len(times)==0:
            return {}
        #get current value to interpolate
        time = perception.time - self.start
        #import ipdb;ipdb.set_trace()
        target_joints = {}
        for i,name in enumerate(names):
            #create arrays so we can use the keyframe values to interpolate
            time_s = np.asarray(times[i])
            #skip the one we already calculated
            if time_s[-1]< time or time_s[0]> time:
                continue
            #remove handles and create array,,,, Its super bad without the handles
            #angles = np.asarray([x[0] for x in keys[i]])
            #print(time_s.shape, angles.shape)
            #print(time, time_s)

            #if len(angles)<=3:
               # K=len(angles)-1
           # else:
              #  K=3
           # spls = UnivariateSpline(time_s,angles,k=K)
           
            #target_joints[name] = spls(time)
        
            idx = np.sum(np.where(time_s < time, 1, 0))
            if (idx-1) >= 0:
                current_key = keys[i][idx-1]
                current_time = time_s[idx-1]
            else:
                current_key = [0, [0, 0, 0.0], [0, 0, 0.0]]
                current_time = 0

            okey = keys[i][idx]
            p_0 = [current_time,current_key[0]]
            p_3 = [time_s[idx], okey[0]]
            #import ipdb;ipdb.set_trace()
            p_1 = [p_0[0]+current_key[2][1], p_0[1] + current_key[2][2]]
            p_2 = [p_3[0] + okey[1][1], p_3[1] + okey[1][2]]
            target_joints[name] = self.bezier(p_0,p_1,p_2,p_3, time)
            #print(self.bezier(p_0,p_1,p_2,p_3, time), self.bezier_curve([p_0, p_1,p_2,p_3]))
        


        #print(times)
        if time >= np.max([x[-1] for x in times])+self.start:  
            print(time, self.start,np.max([x[-1] for x in times]) )
            print('moin')
            self.start=None
        
        if 'LHipYawPitch' in target_joints:
            target_joints['RHipYawPitch'] = target_joints['LHipYawPitch']
        return target_joints


if __name__ == '__main__':
    agent = AngleInterpolationAgent()
    agent.keyframes = hello()  # CHANGE DIFFERENT KEYFRAMES
    agent.run()

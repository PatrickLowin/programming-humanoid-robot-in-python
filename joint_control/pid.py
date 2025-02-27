'''In this exercise you need to implement the PID controller for joints of robot.

* Task:
    1. complete the control function in PIDController with prediction
    2. adjust PID parameters for NAO in simulation

* Hints:
    1. the motor in simulation can simple modelled by angle(t) = angle(t-1) + speed * dt
    2. use self.y to buffer model prediction
'''

# add PYTHONPATH
import os
import sys
sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'software_installation'))

import numpy as np
from collections import deque
from spark_agent import SparkAgent, JOINT_CMD_NAMES


class PIDController(object):
    '''a discretized PID controller, it controls an array of servos,
       e.g. input is an array and output is also an array
    '''
    def __init__(self, dt, size):
        '''
        @param dt: step time
        @param size: number of control values
        @param delay: delay in number of steps
        '''
        self.dt = dt
        self.u = np.zeros(size)
        self.e1 = np.zeros(size)
        self.e2 = np.zeros(size)
        # ADJUST PARAMETERS BELOW
        delay = 0
        self.Kp = 40
        self.Ki = 0.5
        self.Kd = 0.1
        self.y = deque(np.zeros(size), maxlen=delay + 1)
        
    def set_delay(self, delay):
        '''
        @param delay: delay in number of steps
        '''
        self.y = deque(self.y, delay + 1)

    def control(self, target, sensor):
        #with prediction Folie 10
        '''apply PID control
        @param target: reference values
        @param sensor: current values from sensor
        @return control signal
        '''
        #prediction of next joint location
        
        no_delay_out = self.y[-1]
        if len(self.y) >= self.y.maxlen:
            y_hat = self.y.popleft()
        else:
            y_hat= no_delay_out
        
        #diff = y - y_hat
        y_diff =  y_hat - sensor
        #y~ + diff
        y_squiggly = y_diff + no_delay_out
        #r-y
        e = target - sensor #y_squiggly 
        #print(e,self.e1, self.e2)
        #Error Integration for I Term
        

        #PID
        #P = e *self.dt * self.Kp #Kp times current error
        #I = self.e2 *self.dt* self.Ki # Ki times error sum
        #D = (e-self.e1) * self.dt * self.Kd #(now - last error)*dt 
        #PID = P+I+D
        
        #PID = (self.Kp + self.Ki*self.dt + self.Kd/self.dt)*e-(self.Kp + 2*self.Kd/self.dt)*self.e1+ self.e2 * self.Kd/self.dt#P+I+D
        #formeln nach K's aufgelöst
        self.u +=self.Kp * (e - self.e1) + self.Ki * self.dt * e + self.Kd / self.dt * (e - 2*self.e1 + self.e2)
        
        speed = ((self.u - sensor) + (no_delay_out - sensor)) / (2*self.dt)
        self.y.append(self.u + speed*self.dt)
        #last error
        #self.e2 = self.e1
        self.e2 = self.e1
        self.e1 = e
    
        return self.u





class PIDAgent(SparkAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(PIDAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.joint_names = JOINT_CMD_NAMES.keys()
        number_of_joints = len(self.joint_names)
        self.joint_controller = PIDController(dt=0.01, size=number_of_joints)
        self.target_joints = {k: 0 for k in self.joint_names}

    def think(self, perception):
        action = super(PIDAgent, self).think(perception)
        '''calculate control vector (speeds) from
        perception.joint:   current joints' positions (dict: joint_id -> position (current))
        self.target_joints: target positions (dict: joint_id -> position (target)) '''
        joint_angles = np.asarray(
            [perception.joint[joint_id]  for joint_id in JOINT_CMD_NAMES])
        target_angles = np.asarray([self.target_joints.get(joint_id, 
            perception.joint[joint_id]) for joint_id in JOINT_CMD_NAMES])
        u = self.joint_controller.control(target_angles, joint_angles)
        #print(u)
        action.speed = dict(zip(JOINT_CMD_NAMES.keys(), u))  # dict: joint_id -> speed
        return action


if __name__ == '__main__':
    agent = PIDAgent()
    agent.target_joints['HeadYaw'] = 1.0
    agent.run()

'''In this exercise you need to implemente inverse kinematics for NAO's legs
* Tasks:
    1. solve inverse kinemtatics for NAO's legs by using analytical or numerical method.
       You may need documentation of NAO's leg:
       http://doc.aldebaran.com/2-1/family/nao_h25/joints_h25.html
       http://doc.aldebaran.com/2-1/family/nao_h25/links_h25.html
    2. use the results of inverse kinemtatics to control NAO's legs (in InverseKinematicsAgent.set_transforms)
       and test your inverse kinemtatics implementation.
'''


#robot falls over and simspark freezes

from forward_kinematics import ForwardKinematicsAgent
import numpy as np
from collections import deque
from scipy.spatial.transform import Rotation as R




class InverseKinematicsAgent(ForwardKinematicsAgent):

    def calc_pos(self, joint, transform):
        #get axis of rot
        rotationMatrix = transform[:-1, :-1]
        if joint in ['LElbowYaw', 'RElbowYaw', 'LHipRoll', 'LAnkleRoll', 'RHipRoll', 'RAnkleRoll']:
            axs = [1, 0, 0]
        if joint in  ['HeadPitch', 'LShoulderPitch', 'RShoulderPitch', 'LHipPitch', 'LKneePitch', 'LAnklePitch', 'RHipPitch', 'RKneePitch', 'RAnklePitch']:
            axs = [0, 1, 0]
        if joint in  ['HeadYaw', 'LShoulderRoll', 'LElbowRoll',  'RShoulderRoll', 'RElbowRoll']:
            axs = [0, 0, 1]
        if joint == 'LHipYawPitch':
            axs = [0, 0.707, 0.707]
        axs = np.dot(rotationMatrix, axs)
        coordinates = transform[:, -1][:-1]
        return (coordinates, axs)

    

    def inverse_kinematics(self, effector_name, transform):
        '''solve the inverse kinematics
        :param str effector_name: name of end effector, e.g. LLeg, RLeg
        :param transform: 4x4 transform matrix
        :return: list of joint angles
        '''
        lamb = 0.1
        max_transerror = 0.01
        max_angleerror = 1
        joint_angles = []  # thetas
        # YOUR CODE HERE
        counter = 0
        # getting current angles
        for joint in self.chains[effector_name]:
            joint_angles.append(self.perception.joint[joint])
        
        # target position
        target = self.calc_pos(self.chains[effector_name][-1], transform)
      
        target = np.concatenate((target[0], list(np.squeeze(R.from_matrix(transform[:3,:3]).as_euler('xyz',degrees=False)))))

        # starting inversed kinematics loop
        error=np.ones(6)*6
        while not (np.allclose(error[:-3], 0, 1, max_transerror) and np.allclose(error[-3:], 0, 1, max_angleerror)) and counter<10000:
            # running forward_kinematics
            trans = [np.identity(4)]
            posList = []
            for i, joint in enumerate(self.chains[effector_name]):
                print(i,joint)
                T = self.local_trans(joint, joint_angles[i])
                trans.append(np.dot(trans[-1], T))
                posList.append(self.calc_pos(joint, trans[-1]))
            print('forward finished')
            counter +=1
            end = posList[-1]

            #print(list(np.squeeze(R.from_matrix(transformations[-1][:3,:3]).as_euler('xyz',degrees=True))))
            end = np.concatenate((end[0], list(R.from_matrix(trans[-1][:3,:3]).as_euler('xyz',degrees=False))))

            error = target - end

            # jacobian matrix
            j_cols = []
            for x in posList:
                delta = np.cross(x[1], end[0] - x[0])
                j_cols.append(np.concatenate((delta, x[1])))

            Jacob = np.column_stack(j_cols)

            jacob_inv = np.linalg.pinv(Jacob)
            
            joint_angles += np.dot((lamb * jacob_inv), np.asarray(error).T) 
            joint_angles = joint_angles% 2*np.pi
        
        return joint_angles

    def set_transforms(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
        angles = self.inverse_kinematics(effector_name, transform)
        for i, joint in enumerate(self.chains[effector_name]):
            self.target_joints[joint] = angles[i]


if __name__ == '__main__':
    agent = InverseKinematicsAgent()
    # test inverse kinematics
    'robot falls over but why?'
    T = np.identity(4)
    T[1, -1] = 50.0
    T[2, -1] = -260.
    print(T)
    agent.set_transforms('LLeg', T)
    agent.run()

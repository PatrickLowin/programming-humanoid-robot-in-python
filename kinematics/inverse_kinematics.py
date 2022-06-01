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
from scipy.spatial.transform import Rotation as R

class InverseKinematicsAgent(ForwardKinematicsAgent):

    def getPos(self, joint_name, transform):  
        r_mat = transform[:-1,:-1]
        if joint_name in self.xJoints:
            axs = [1, 0, 0]
        if joint_name in self.yJoints:
            axs = [0, 1, 0]
        if joint_name in self.zJoints:
            axs = [0, 0, 1]
        if joint_name == 'LHipYawPitch':
            axs = [0, np.sin(np.pi/4), np.sin(np.pi/4)]
            axs = np.dot(r_mat, axs)

        coordinates = transform[:,-1][:-1]
        return (coordinates, axs)    

    def inverse_kinematics(self, effector_name, transform):
        '''solve the inverse kinematics

        :param str effector_name: name of end effector, e.g. LLeg, RLeg
        :param transform: 4x4 transform matrix
        :return: list of joint angles
        '''
        joint_angles = []
        #get angles
        for joint in self.chains[effector_name]:
            joint_angles.append(self.perception.joint[joint])

	    #target position
        target = self.getPos(self.chains[effector_name][-1], transform)
        target = np.r_[target[0], list(np.squeeze(R.from_matrix(transform).as_euler('xyz',degrees=True)))]
        
        #inversed kinematics loop
        while True:
            #forward_kinematics
            transformations = [np.identity(4)]
            pos_list = []
            for i, joint in enumerate(self.chains[effector_name]):
                    Tl = self.local_trans(joint, joint_angles[i])
                    transformations.append(np.dot(transformations[-1], Tl))
                    pos_list.append(self.getPos(joint, transformations[-1]))
                    
                    
            #en position
            last = pos_list[-1]
            if(effector_name == 'Lleg'):
                last[0] += np.dot(transformations[-1][:-1,:-1], [0,0,-self.mainLength['FootHeight']])
            last = np.r_[last[0], list(np.squeeze(R.from_matrix(transformations[-1]).as_euler('xyz',degrees=True)))]
         
            error = target - last
            #print s, p[-1]
            #stop if close enough
            if (np.allclose(error[:-3], 0, 1, 0.01) and np.allclose(error[-3:], 0, 1, 0.01)):
                break
            
            #calculating jacobian matrix
            jcolumns = []
            for el in pos_list:
                position, r_axs = el
                d_position = np.cross(r_axs, last[0] - position)
                jcolumns.append(np.r_[d_position, r_axs])
            
            J = np.column_stack(jcolumns)
 
            JI = np.linalg.pinv(J)
         
            #angle correction
            joint_angles += np.dot((0.1 * JI), np.asarray(error).T)
            joint_angles = np.mod(joint_angles, 2*np.pi)
      
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
    T = identity(4)
    T[-1, 1] = 0.05
    T[-1, 2] = -0.26
    agent.set_transforms('LLeg', T)
    agent.run()

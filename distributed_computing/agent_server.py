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
sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'kinematics'))

from inverse_kinematics import InverseKinematicsAgent


import threading
from xmlrpc.server import SimpleXMLRPCServer
from xmlrpc.server import SimpleXMLRPCRequestHandler


class RequestHandler(SimpleXMLRPCRequestHandler):
    rpc_paths = ('/RPC2',)


class ServerAgent(InverseKinematicsAgent):
    '''ServerAgent provides RPC service
    '''
    
    def get_angle(self, joint_name):
        '''get sensor value of given joint'''
        return self.target_joints.get(joint_name)
    
    def set_angle(self, joint_name, angle):
        '''set target angle of joint for PID controller
        '''
        self.target_joints[joint_name] = angle

    def get_posture(self):
        '''return current posture of robot'''
        return self.recognize_posture(self.perception)

    def execute_keyframes(self, keyframes):
        '''excute keyframes, note this function is blocking call,
        e.g. return until keyframes are executed
        '''
        return self.angle_interpolation(keyframes, self.perception)



    def get_transform(self, name):
        '''get transform with given name'''
        return self.transforms.get(name)

    def set_transform(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        self.set_transform(self, effector_name, transform)

if __name__ == '__main__':
    agent = ServerAgent()

    server = SimpleXMLRPCServer(("localhost", 8000), requestHandler=RequestHandler, allow_none=True)
    server.register_instance(agent)
    server.register_introspection_functions()
    server.register_multicall_functions()
    print('server')

    thread = threading.Thread(target=server.serve_forever)
    thread.start()
    print('finish thread')

    agent.run()


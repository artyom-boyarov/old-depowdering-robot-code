from trajectory_msgs.msg import *
from control_msgs.msg import *
import rospy
import actionlib
from sensor_msgs.msg import JointState
import numpy as np
 
JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

def get_inverse_kinematics(x, y, z, phi=0, theta=0, psi=np.pi/2):
    alpha0 = 0 # [rad]
    alpha1 = np.pi/2
    alpha2 = 0
    alpha4 = np.pi/2
    alpha5 = -np.pi/2

    a0 = 0 # [m]
    a1 = 0
    a2 = -0.425
    a3 = -0.3922
    a4 = 0
    a5 = 0

    d1 = 0.1625 # length from base joint shoulder joint in the z-direction
    #d1 = 0 # length from base joint shoulder joint in the z-direction
    d2 = 0
    d3 = 0
    d4 = 0.1333 # [m]
    d5 = 0.0997
    d6 = 0.0996 # [m]

    p_1 = np.array([[a0], [-np.sin(alpha0)*d1], [np.cos(alpha0)*d1]])
    p_2 = np.array([[a1], [-np.sin(alpha1)*d2], [np.cos(alpha1)*d2]])
    p_3 = np.array([[a2], [-np.sin(alpha2)*d3], [np.cos(alpha2)*d3]])
    p_5 = np.array([[a4], [-np.sin(alpha4)*d5], [np.cos(alpha4)*d5]])
    p_6 = np.array([[a5], [-np.sin(alpha5)*d6], [np.cos(alpha5)*d6]])

    q_0 = 0 # base joint angle
    # rotation matrix
    R_xyz = np.array([[np.cos(phi)*np.cos(theta), -np.sin(phi)*np.cos(psi)+np.cos(phi)*np.sin(theta)*np.sin(psi), np.sin(phi)*np.sin(psi)+np.cos(phi)*np.sin(theta)*np.cos(psi)],
                    [np.sin(phi)*np.cos(theta), np.cos(phi)*np.cos(psi)+np.sin(phi)*np.sin(theta)*np.sin(psi), -np.cos(phi)*np.sin(psi)+np.sin(phi)*np.sin(theta)*np.cos(psi)],
                    [-np.sin(theta), np.cos(theta)*np.sin(psi), np.cos(theta)*np.cos(psi)]])

    tol = 10**(-3) # tolerance for computations
    q = np.zeros((6,))
    p_60 = np.array([[x],[y],[z]])
    H_60 = np.concatenate((R_xyz,p_60), axis=1)
    homogenous_concat = np.concatenate((np.zeros((1,3)),np.ones((1,1))), axis=1)
    H_60 = np.concatenate((H_60, homogenous_concat),axis=0)

    # q_1 -- find the origin of frame 5 by moving d_6 in the negative z direction
    unit_Z_60 = H_60[0:3,2]
    p_50 = p_60.flatten() - d6*unit_Z_60.flatten()
    q[0] = np.arctan2(p_50[1],p_50[0]) + np.arccos( d4 / (np.sqrt(p_50[0]**2 + p_50[1]**2)) ) + np.pi/2 # [rad]

    R_10 = np.array([[np.cos(q[0]), -np.sin(q[0]), 0],
                    [np.sin(q[0])*np.cos(alpha0), np.cos(q[0])*np.cos(alpha0), -np.sin(alpha0)],
                    [np.sin(q[0])*np.sin(alpha0), np.cos(q[0])*np.sin(alpha0), np.cos(alpha0)]])
    T_10 = np.concatenate((R_10,p_1), axis=1)
    T_10 = np.concatenate((T_10, homogenous_concat),axis=0)

    # q_5
    val = np.round((p_60[0]*np.sin(q[0]) - p_60[1]*np.cos(q[0])-d4) / d6, 4)
    q[4] = np.arccos( val )

    R_54 = np.array([[np.cos(q[4]), -np.sin(q[4]), 0],
                    [np.sin(q[4])*np.cos(alpha4), np.cos(q[4])*np.cos(alpha4), -np.sin(alpha4)],
                    [np.sin(q[4])*np.sin(alpha4), np.cos(q[4])*np.sin(alpha4), np.cos(alpha4)]])
    T_54 = np.concatenate((R_54,p_5), axis=1)
    T_54 = np.concatenate((T_54, homogenous_concat),axis=0)

    # q_6 
    unit_X_06 = np.matmul(np.transpose(R_xyz),np.array([[1],[0],[0]]))
    unit_Y_06 = np.matmul(np.transpose(R_xyz),np.array([[0],[1],[0]]))

    if np.abs(np.sin(q[4])) <= tol:
        q[5] = 0
    else:
        q[5] = np.arctan2((-unit_X_06[1]*np.sin(q[0])+unit_Y_06[1]*np.cos(q[0]))/np.sin(q[4]), \
            (unit_X_06[0]*np.sin(q[0])-unit_Y_06[0]*np.cos(q[0])/np.sin(q[4])) )
    
    R_65 = np.array([[np.cos(q[5]), -np.sin(q[5]), 0],
                    [np.sin(q[5])*np.cos(alpha5), np.cos(q[5])*np.cos(alpha5), -np.sin(alpha5)],
                    [np.sin(q[5])*np.sin(alpha5), np.cos(q[5])*np.sin(alpha5), np.cos(alpha5)]])
    T_65 = np.concatenate((R_65,p_6), axis=1)
    T_65 = np.concatenate((T_65, homogenous_concat),axis=0)

    # q_3
    p_40 = p_50 - d5*np.array([0,0,-1])
    p_41 = p_40 - d1*np.array([0,0,1])

    test_len = np.sqrt((p_41[0].flatten())**2 + (p_41[2].flatten())**2)
    if test_len >= np.abs(a2-a3)+tol and test_len <= np.abs(a2+a3)+tol:
        q[2] = np.arccos(((p_41[0]**2 + p_41[2]**2) - a2**2 - a3**2) / (2*a2*a3))
    else:
        # print an error message and break the loop
        raise Exception("The position is not in the robots reachable space. Exiting...")

    # q_2
    q[1] = np.arctan2(-p_41[2],-p_41[0]) + np.arcsin( a3*np.sin(q[2]) / test_len)

    #q_4
    R_21 = np.array([[np.cos(q[1]), -np.sin(q[1]), 0],
                    [np.sin(q[1])*np.cos(alpha1), np.cos(q[1])*np.cos(alpha1), -np.sin(alpha1)],
                    [np.sin(q[1])*np.sin(alpha1), np.cos(q[1])*np.sin(alpha1), np.cos(alpha1)]])
    T_21 = np.concatenate((R_21,p_2), axis=1)
    T_21 = np.concatenate((T_21, homogenous_concat),axis=0)

    R_32 = np.array([[np.cos(q[2]), -np.sin(q[2]), 0],
                    [np.sin(q[2])*np.cos(alpha2), np.cos(q[2])*np.cos(alpha2), -np.sin(alpha2)],
                    [np.sin(q[2])*np.sin(alpha2), np.cos(q[2])*np.sin(alpha2), np.cos(alpha2)]])
    T_32 = np.concatenate((R_32,p_3), axis=1)
    T_32 = np.concatenate((T_32, homogenous_concat),axis=0)

    T_30 = np.matmul(T_10,np.matmul(T_21,T_32))
    T_64 = np.matmul(T_54,T_65)

    T_43 = np.matmul(np.linalg.inv(T_30),np.matmul(H_60,np.linalg.inv(T_64)))

    X_43 = T_43[:,0]

    q[3] = np.arctan2(X_43[1],X_43[0])

    return np.round(q,4)

def move():
          #create class
          goal = FollowJointTrajectoryGoal()
 
          #set trajectory
          goal.trajectory = JointTrajectory()
          #joint.names
          goal.trajectory.joint_names = JOINT_NAMES
 
          #get joint_state values
          joint_states = rospy.wait_for_message("joint_states",JointState)
          joints_pos = joint_states.position
 
          #give points valueW
          sept1 = get_inverse_kinematics(-0.5,0.20,0.3,0,0,np.pi)
          sept2 = get_inverse_kinematics(-0.5,-0.20,0.3,0,0,np.pi)
          sept3 = get_inverse_kinematics(-0.65,-0.20,0.3,0,0,np.pi)
          sept4 = get_inverse_kinematics(-0.65,0.20,0.3,0,0,np.pi)
          #sept1[0] -= 2*np.pi
          #sept4[0] -= 2*np.pi
          sept1 = sept1 - 2*np.pi*np.floor((sept1+np.pi)/(2*np.pi))
          sept4 = sept4 - 2*np.pi*np.floor((sept4+np.pi)/(2*np.pi))
          goal.trajectory.points=[0]*5
          goal.trajectory.points[0]=JointTrajectoryPoint(positions=joints_pos, velocities=[0]*6,time_from_start=rospy.Duration(0.0))
          goal.trajectory.points[1]=JointTrajectoryPoint(positions=sept1, velocities=[0]*6,time_from_start=rospy.Duration(1.0))
          goal.trajectory.points[2]=JointTrajectoryPoint(positions=sept2, velocities=[0]*6,time_from_start=rospy.Duration(2.0))
          goal.trajectory.points[3]=JointTrajectoryPoint(positions=sept3, velocities=[0]*6,time_from_start=rospy.Duration(3.0))
          goal.trajectory.points[4]=JointTrajectoryPoint(positions=sept4, velocities=[0]*6,time_from_start=rospy.Duration(4.0))
          #go for loop
          for i in range(5):
                
          #goal.trajectory.points[2]=JointTrajectoryPoint(positions=sept3, velocities=[0]*6,time_from_start=rospy.Duration(3.0))
          #goal.trajectory.points[2]=JointTrajectoryPoint(positions=sept4, velocities=[0]*6,time_from_start=rospy.Duration(4.0))  

          #goal.trajectory.points[2]=JointTrajectoryPoint(positions=[3.3432,  2.8806 , 0.3807 ,-0.1198 , 2.9398, -3.1416], velocities=[0]*6,time_from_start=rospy.Duration(2.0))
          #goal.trajectory.points[2]=JointTrajectoryPoint(positions=[1,0,-1,0,0,0], velocities=[0]*6,time_from_start=rospy.Duration(2.0))
          #goal.trajectory.points[3]=JointTrajectoryPoint(positions=[1.57,0,-1.57,0,0,0], velocities=[0]*6,time_from_start=rospy.Duration(3.0))
          
          #send goal
            client.send_goal(goal)
            client.wait_for_result()
 
def pub_test():
          global client
 
          #initialize ros node
          rospy.init_node("pub_action_test")
 
          #create action class
          client = actionlib.SimpleActionClient('pos_joint_traj_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
          print("Waiting for server...")
          #wait for server
          client.wait_for_server()
          print("Connect to server")
          #run move function
          move()
 
if __name__ == "__main__":
          pub_test()

#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, sys
from math import *
import numpy as np
import numpy.matlib
import moveit_commander
import scipy.io as io
import h5py

from geometry_msgs.msg import PoseStamped, Pose, Point
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Path
from std_msgs.msg import ColorRGBA

from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from moveit_msgs.msg import  PlanningScene, ObjectColor
from geometry_msgs.msg import PoseStamped, Pose

def visualization_stl(num,position,color):
    marker=Marker()
    marker.header.frame_id="/world"
    marker.type=Marker.MESH_RESOURCE
    marker.mesh_resource="package://paintingrobot_planning/script/stl_document/0016.STL"
    marker.action=Marker.ADD
    marker.pose.orientation.w=1.0
    marker.scale.x=1.0
    marker.scale.y=1.0
    marker.scale.z=1.0
    marker.ns='arrow'
    marker.id=num
    marker.lifetime = rospy.Duration()

    marker.color.r=color.r
    marker.color.g=color.g
    marker.color.b=color.b
    marker.color.a=color.a

    marker.pose.position.x=position.x
    marker.pose.position.y=position.y
    marker.pose.position.z=position.z
    return marker

class pose2mat():
    def __init__(self):
        self.a=3
    def rotx(self,q):
        cq=cos(q)
        sq=sin(q)
        R =[[1, 0, 0],
            [0, cq, -sq],
            [0, sq, cq]]
        return R
    def roty(self,q):
        cq = cos(q)
        sq = sin(q)
        R=[[cq, 0, sq],
           [0, 1, 0],
           [-sq,0,cq]]
        return  R
    def rotz(self,q):
        cq=cos(q)
        sq=sin(q)
        R=[[cq, -sq, 0],
           [sq,  cq, 0],
           [ 0,   0, 1]]
        return R
    def rpy2r(self,q):
        roll=q[0]
        pitch=q[1]
        yaw=q[2]
        Rx = self.rotx(roll)
        Ry = self.roty(pitch)
        Rz = self.rotz(yaw)
        R1=np.dot(Rx, Ry)
        R=np.dot(R1,Rz)
        return R
    def rpy2tr(self,q):
        T = np.matlib.identity(4, dtype=float)
        R=self.rpy2r(q)
        T[0:3, 0:3] = R
        return T
    def tran2r(self,p):
        r = np.array(p).reshape((3, 1))
        return r
    def tran2tr(self,p):
        T = np.matlib.identity(4, dtype=float)
        T[0:3, 3] = np.array(p).reshape((3, 1))
        return T
    def mat4x4(self,p,q):
        T = np.matlib.identity(4, dtype=float)
        rot = self.rpy2r(q)
        tran = self.tran2r(p)
        T[0:3, 0:3] = rot
        T[0:3, 3] = tran
        return T

class Aubo_kinematics():
    def __init__(self):
        self.a2 = 0.408
        self.a3 = 0.376
        self.d1 = 0.122
        self.d2 = 0.1215
        self.d5 = 0.1025
        self.d6 = 0.094
        self.ZERO_THRESH = 1e-4
        self.ARM_DOF = 6

    def degree_to_rad(self, q):
        temp = []
        for i in range(len(q)):
            temp.append(q[i] * pi / 180)
        return temp

    def antiSinCos(self, sA, cA):

        eps = 1e-8
        angle = 0
        if ((abs(sA) < eps) and (abs(cA) < eps)):
            return 0

        if (abs(cA) < eps):
            angle = pi / 2.0 * self.SIGN(sA)
        elif (abs(sA) < eps):

            if (self.SIGN(cA) == 1):
                angle = 0
            else:
                angle = pi

        else:

            angle = atan2(sA, cA)

        return angle

    def SIGN(self, x):
        if x>0:
            return 1
        else:
            return -1
        # return (x > 0)-(x < 0)

    def aubo_forward(self, q):
        q = self.degree_to_rad(q)
        T = []
        for i in range(16):
            T.append(0)
        # print q
        q1 = q[0]
        q2 = q[1]
        q3 = q[2]
        q4 = q[3]
        q5 = q[4]
        q6 = q[5]
        C1 = cos(q1)
        C2 = cos(q2)
        C4 = cos(q4)
        C5 = cos(q5)
        C6 = cos(q6)
        C23 = cos(q2 - q3)
        C234 = cos(q2 - q3 + q4)
        C2345 = cos(q2 - q3 + q4 - q5)
        C2345p = cos(q2 - q3 + q4 + q5)
        S1 = sin(q1)
        S2 = sin(q2)
        S4 = sin(q4)
        S5 = sin(q5)
        S6 = sin(q6)
        S23 = sin(q2 - q3)
        S234 = sin(q2 - q3 + q4)

        T[0] = -C6 * S1 * S5 + C1 * (C234 * C5 * C6 - S234 * S6)
        T[1] = S1 * S5 * S6 - C1 * (C4 * C6 * S23 + C23 * C6 * S4 + C234 * C5 * S6)
        T[2] = C5 * S1 + C1 * C234 * S5
        T[3] = (self.d2 + C5 * self.d6) * S1 - C1 * (
                    self.a2 * S2 + (self.a3 + C4 * self.d5) * S23 + C23 * self.d5 * S4 - C234 * self.d6 * S5)

        T[4] = C234 * C5 * C6 * S1 + C1 * C6 * S5 - S1 * S234 * S6
        T[5] = -C6 * S1 * S234 - (C234 * C5 * S1 + C1 * S5) * S6
        T[6] = -C1 * C5 + C234 * S1 * S5
        T[7] = -C1 * (self.d2 + C5 * self.d6) - S1 * (
                    self.a2 * S2 + (self.a3 + C4 * self.d5) * S23 + C23 * self.d5 * S4 - C234 * self.d6 * S5)

        T[8] = C5 * C6 * S234 + C234 * S6
        T[9] = C234 * C6 - C5 * S234 * S6
        T[10] = S234 * S5
        T[11] = self.d1 + self.a2 * C2 + self.a3 * C23 + self.d5 * C234 + self.d6 * C2345 / 2 - self.d6 * C2345p / 2
        T[12] = 0
        T[13] = 0
        T[14] = 0
        T[15] = 1
        return T

    def aubo_inverse(self, T):
        q_reslut_dic = {}
        q_reslut = []
        singularity = False

        num_sols = 0
        nx = T[0]
        ox = T[1]
        ax = T[2]
        px = T[3]

        ny = T[4]
        oy = T[5]
        ay = T[6]
        py = T[7]
        nz = T[8]
        oz = T[9]
        az = T[10]
        pz = T[11]

        # //////////////////////// shoulder rotate joint (q1) //////////////////////////////
        q1 = [0, 0]

        A1 = self.d6 * ay - py
        B1 = self.d6 * ax - px
        R1 = A1 * A1 + B1 * B1 - self.d2 * self.d2

        if R1 < 0.0:
            return num_sols
        else:
            R12 = sqrt(R1)
            q1[0] = self.antiSinCos(A1, B1) - self.antiSinCos(self.d2, R12)
            q1[1] = self.antiSinCos(A1, B1) - self.antiSinCos(self.d2, -R12)
            for i in range(len(q1)):

                while q1[i] > pi:
                    q1[i] -= 2 * pi
                while q1[i] < -pi:
                    q1[i] += 2 * pi

        # ////////////////////////////// wrist 2 joint (q5) //////////////////////////////
        q5 = [[0, 0], [0, 0]]

        for i in range(len(q5)):
            C1 = cos(q1[i])
            S1 = sin(q1[i])
            B5 = -ay * C1 + ax * S1
            M5 = (-ny * C1 + nx * S1)
            N5 = (-oy * C1 + ox * S1)
            R5 = sqrt(M5 * M5 + N5 * N5)

            q5[i][0] = self.antiSinCos(R5, B5)
            q5[i][1] = self.antiSinCos(-R5, B5)

        # ////////////////////////////////////////////////////////////////////////////////

        # ////////////////////////////// wrist 3 joint (q6) //////////////////////////////
        q6 = 0

        q3 = [0, 0]
        q2 = [0, 0]
        q4 = [0, 0]
        for i in range(len(q3)):

            for j in range(len(q3)):

                # // wrist 3 joint (q6) //
                C1 = cos(q1[i])
                S1 = sin(q1[i])
                S5 = sin(q5[i][j])

                A6 = (-oy * C1 + ox * S1)
                B6 = (ny * C1 - nx * S1)

                if fabs(S5) < self.ZERO_THRESH:  # //the condition is only dependent on q1

                    singularity = True
                    break
                else:
                    q6 = self.antiSinCos(A6 * S5, B6 * S5)

                # /////// joints (q3,q2,q4) //////
                C6 = cos(q6)
                S6 = sin(q6)

                pp1 = C1 * (ax * self.d6 - px + self.d5 * ox * C6 + self.d5 * nx * S6) + S1 * (
                            ay * self.d6 - py + self.d5 * oy * C6 + self.d5 * ny * S6)
                pp2 = -self.d1 - az * self.d6 + pz - self.d5 * oz * C6 - self.d5 * nz * S6
                B3 = (pp1 * pp1 + pp2 * pp2 - self.a2 * self.a2 - self.a3 * self.a3) / (2 * self.a2 * self.a3)

                if ((1 - B3 * B3) < self.ZERO_THRESH):
                    singularity = True
                    continue
                else:
                    Sin3 = sqrt(1 - B3 * B3)
                    q3[0] = self.antiSinCos(Sin3, B3)
                    q3[1] = self.antiSinCos(-Sin3, B3)

                for k in range(len(q3)):

                    C3 = cos(q3[k])
                    S3 = sin(q3[k])
                    A2 = pp1 * (self.a2 + self.a3 * C3) + pp2 * (self.a3 * S3)
                    B2 = pp2 * (self.a2 + self.a3 * C3) - pp1 * (self.a3 * S3)

                    q2[k] = self.antiSinCos(A2, B2)
                    C2 = cos(q2[k])
                    S2 = sin(q2[k])

                    A4 = -C1 * (ox * C6 + nx * S6) - S1 * (oy * C6 + ny * S6)
                    B4 = oz * C6 + nz * S6
                    A41 = pp1 - self.a2 * S2
                    B41 = pp2 - self.a2 * C2

                    q4[k] = self.antiSinCos(A4, B4) - self.antiSinCos(A41, B41)
                    while (q4[k] > pi):
                        q4[k] -= 2 * pi
                    while (q4[k] < -pi):
                        q4[k] += 2 * pi
                    q_reslut = [q1[i], q2[k], q3[k], q4[k], q5[i][j], q6]

                    q_reslut_dic.update({num_sols: q_reslut})
                    num_sols += 1

        return q_reslut_dic, num_sols
        """
        The Frobenius norm, sometimes also called the Euclidean norm (a term unfortunately also used for the vector L^2-norm), 
        is matrix norm of an m×n matrix A defined as the square root of the sum of the absolute squares of its elements,
        """

    def List_Frobenius_Norm(self, list_a, list_b):
        new_list = []
        if len(list_a) == len(list_b):
            for i in range(len(list_a)):
                new_list.append(abs(list_a[i] - list_b[i]) ** 2)
        else:
            print("please make sure the list has the same length")

        return sqrt(self.sum_list(new_list))

    def sum_list(self, list_data):
        sum_data = 0
        for i in range(len(list_data)):
            sum_data += list_data[i]
        return sum_data

    # //choose solution ;input all q_sols,and last q_old;
    # //out put the nearest q solution;
    # /**
    #  * @brief chooseIKonRefJoint , choose mode == 0;
    #  * @param q_sols rad
    #  * @param q_ref
    #  * @param q_choose
    #  * @return
    #  */
    def chooseIKonRefJoint(self, q_sols, q_ref):

        nn = len(q_sols)
        if (nn == 0):
            return False, []
        # print "nn---",nn
        sum_data = self.List_Frobenius_Norm(q_ref, q_sols[0])
        err = 0
        index = 0
        for i in range(len(q_sols)):

            err = self.List_Frobenius_Norm(q_ref, q_sols[i])
            # print "err",err
            if (err < sum_data):
                index = i
                sum_data = err

        # print sum_data
        q_choose = q_sols[index]

        return True, q_choose

    """aubo rotation joint:-175 degree/175 degree,here is rad I wanna take a break so use degree"""

    def selectIK(self, q_sols, AngleLimit):
        q_sols_selected = {}
        N = len(q_sols)
        # print "selectIK ---N---",N
        if (N == 0):
            return False, {}
        num = 0
        valid = True
        for i in range(N):

            valid = True;
            for j in range(self.ARM_DOF):
                # drop greater than offical degree
                if (q_sols[i][j] > AngleLimit[j][1] or q_sols[i][j] < AngleLimit[j][0]):
                    valid = False
                    break

            # delete the sols about joint angular increase 2*pi greater than the legal angular
            if (valid):
                temp = q_sols[i]
                temp_1 = q_sols[i]
                for j in range(self.ARM_DOF):
                    temp[j] += 2 * pi
                    temp_1[j] -= 2 * pi
                    if temp[j] > AngleLimit[j][1] or temp_1[j] < AngleLimit[j][0]:
                        valid = False
                        break
                if valid:
                    q_sols_selected.update({num: q_sols[i]})
                    num += 1

        num_sols = num;

        if (num > 0):
            return True, q_sols_selected
        else:
            return False, {}

    def GetInverseResult(self, T_target, q_ref):
        num_sols = 0
        maxq = 175.0 / 180.0 * pi
        AngleLimit = [(-maxq, maxq), (-maxq, maxq), (-maxq, maxq), (-maxq, maxq), (-maxq, maxq), (-maxq, maxq)]
        # inverse and remove zero list
        q_sols_all, num_sols = self.aubo_inverse(T_target)
        if (len(q_sols_all) != 0):
            for i in q_sols_all:
                pass
                # print("num:" + str(i) + ' ' + "sols", q_sols_all[i])
            # remove not in limited data
            ret2, q_sols_inlimit = self.selectIK(q_sols_all, AngleLimit)
            # print "q_sols_inlimit",q_sols_inlimit
            if ((len(q_sols_inlimit) != 0) and (True == ret2)):
                ret3, q_result = self.chooseIKonRefJoint(q_sols_inlimit, q_ref)
                if (True == ret3):

                    print(" find solution choose  ")
                    return q_result
                else:
                    print(" No solution choose  ")
            else:
                print("no valid sols ")
        else:
            print("inverse result num is 0")
            # return False


class Renovationrobot_motion():
    def __init__(self):
        self.parameterx=0.430725381079
        self.parametery=-0.00033063639818
        self.parameterz=1.32061628232

    def homing(self):
        # definition of three moveit groups
        mobileplatform = moveit_commander.MoveGroupCommander('mobileplatform')
        rodclimbing_robot= moveit_commander.MoveGroupCommander('rodclimbing_robot')
        arm = moveit_commander.MoveGroupCommander('aubo5')
        # these groups moves to initial poses
        mobileplatform.set_named_target('home1')
        mobileplatform.go()
        rospy.sleep(1)

        rodclimbing_robot.set_named_target('home2')
        rodclimbing_robot.go()
        rospy.sleep(1)

        arm.set_named_target('home3')
        arm.go()
        rospy.sleep(1)
    def motion(self,manipulatorbase_targetpose_onecell,manipulatorendeffector_targetpose_onecell):
        # definition of three moveit groups
        mobileplatform = moveit_commander.MoveGroupCommander('mobileplatform')
        rodclimbing_robot = moveit_commander.MoveGroupCommander('rodclimbing_robot')
        arm = moveit_commander.MoveGroupCommander('aubo5')

        # obtain the robot base pose at initial state
        manipulatorbase_pose = rodclimbing_robot.get_current_pose('aubo_baselink').pose
        print("manipulatorbase_pose=:",manipulatorbase_pose)
        # obtain the robot joints at initial state
        mobileplatform_joint = mobileplatform.get_current_joint_values()
        print('mobileplatform_joint=:',mobileplatform_joint)
        rodclimbing_robot_joints = rodclimbing_robot.get_current_joint_values()
        print('rodclimbing_robot_joints=:',rodclimbing_robot_joints)

        # firstly, the motion of mobile platform
        theta_z=manipulatorbase_targetpose_onecell[0][3]
        deltax=self.parameterx*cos(theta_z)-self.parametery*sin(theta_z)
        deltay=self.parameterx*sin(theta_z)+self.parametery*cos(theta_z)

        # mobileplatform_targetjoints=[manipulatorbase_targetpose_onecell[0][0]-deltax-mobileplatform_joint[0],-(manipulatorbase_targetpose_onecell[0][1]-deltay+mobileplatform_joint[1]),theta_z-mobileplatform_joint[2]]
        mobileplatform_targetjoints=[manipulatorbase_targetpose_onecell[0][0]-deltax,-(manipulatorbase_targetpose_onecell[0][1]-deltay),theta_z]
        print("mobileplatform_targetjoints=:",mobileplatform_targetjoints)
        mobileplatform.set_joint_value_target(mobileplatform_targetjoints)
        mobileplatform.go()
        rospy.sleep(1)

        # secondly, the motion of rod climbing robot
        rodclimbing_robot_targetjoints=[manipulatorbase_targetpose_onecell[0][2]-manipulatorbase_pose.position.z+rodclimbing_robot_joints[0],rodclimbing_robot_joints[1]]
        print("rodclimbing_robot_targetjoints=:",rodclimbing_robot_targetjoints)
        rodclimbing_robot.set_joint_value_target(rodclimbing_robot_targetjoints)
        rodclimbing_robot.go()
        rospy.sleep(1)

        manipulatorbase_pose1 = rodclimbing_robot.get_current_pose('aubo_baselink').pose
        print("manipulatorbase_pose1=:",manipulatorbase_pose1)

        # thirdly, the motion of manipulator, the target pose of manipulator end effector: xyz-rpy(roll-pitch-yaw,'xyz')
        print(manipulatorendeffector_targetpose_onecell)
        manipulatorendeffector_targetpose_onecell_new=np.zeros(manipulatorendeffector_targetpose_onecell.shape)
        for i in range(len(manipulatorendeffector_targetpose_onecell_new)/2):
            if i%2==0:
                for j in range(len(manipulatorendeffector_targetpose_onecell_new[i])):
                    manipulatorendeffector_targetpose_onecell_new[2*i][j]=manipulatorendeffector_targetpose_onecell[2*i][j]
                    manipulatorendeffector_targetpose_onecell_new[2*i+1][j]=manipulatorendeffector_targetpose_onecell[2*i+1][j]
            else:
                for j in range(len(manipulatorendeffector_targetpose_onecell_new[i])):
                    manipulatorendeffector_targetpose_onecell_new[2*i][j]=manipulatorendeffector_targetpose_onecell[2*i+1][j]
                    manipulatorendeffector_targetpose_onecell_new[2*i+1][j]=manipulatorendeffector_targetpose_onecell[2*i][j]
        print(manipulatorendeffector_targetpose_onecell_new)

        interval = 0.10
        previous_aubo_joints=np.array([0.7432146906113353, -0.6072259915236845, 1.387201205355398, 1.9944271968790823, 0.8275816361835613, 1.5707963267948966])
        for i in range(len(manipulatorendeffector_targetpose_onecell_new)-1):
            p1=np.array([manipulatorendeffector_targetpose_onecell_new[i][0],manipulatorendeffector_targetpose_onecell_new[i][1],manipulatorendeffector_targetpose_onecell_new[i][2]])
            p2=np.array([manipulatorendeffector_targetpose_onecell_new[i+1][0],manipulatorendeffector_targetpose_onecell_new[i+1][1],manipulatorendeffector_targetpose_onecell_new[i+1][2]])
            distance=sqrt((p1[0]-p2[0])**2+(p1[1]-p2[1])**2+(p1[2]-p2[2])**2)
            p=np.zeros(3)
            if i==len(manipulatorendeffector_targetpose_onecell_new)-2:
                num = int(distance / interval)+2
            else:
                num = int(distance / interval)+1
            for j in range(num):
                p[0] = p1[0] + (p2[0] - p1[0]) / distance * interval * j
                p[1] = p1[1] + (p2[1] - p1[1]) / distance * interval * j
                p[2] = p1[2] + (p2[2] - p1[2]) / distance * interval * j
                q=np.array([manipulatorendeffector_targetpose_onecell_new[i][3],manipulatorendeffector_targetpose_onecell_new[i][4],manipulatorendeffector_targetpose_onecell_new[i][5]])
                T_mat_generation=pose2mat()
                mat=T_mat_generation.mat4x4(p,q)
                mat1=np.ravel(mat)
                mat2=mat1.tolist()
                aubo_arm = Aubo_kinematics()
                aubo_joints=aubo_arm.GetInverseResult(mat2,previous_aubo_joints)
                arm.set_joint_value_target(aubo_joints)
                arm.go()
                # print(previous_aubo_joints)
                previous_aubo_joints=aubo_joints
                array1=np.array([p[0],p[1],p[2],q[0],q[1],q[2]])
                print(array1)
        rospy.sleep(1)

        # # obtian the robot pose after motion
        # arm.set_pose_reference_frame("base_link")
        # manipulatorendeffector_pose=arm.get_current_pose()
        # print("the manipulator end effector pose is shown as:", manipulatorendeffector_pose)
    def path_visualization(self,num,pose):
        marker = Marker()
        marker.header.frame_id = "/base_link"
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0

        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        marker.ns = 'path'
        marker.id = num
        marker.lifetime = rospy.Duration()

        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        marker.pose.position.x = pose[0]
        marker.pose.position.y = pose[1]
        marker.pose.position.z = pose[2]
        marker.pose.orientation.x = pose[3]
        marker.pose.orientation.y = pose[4]
        marker.pose.orientation.z = pose[5]
        marker.pose.orientation.w = pose[6]


if __name__ == "__main__":
    mat_path="/home/zy/catkin_ws/src/paintingrobot_related/paintingrobot_planning/script/data.mat"
    # mat_path="package://paintingrobot_planning/script/data.mat"
    data = io.loadmat(mat_path)
    manipulatorbase_targetpose=data['renovation_cells_manipulatorbase_positions']
    manipulatorendeffector_targetpose=data['manipulator_endeffector_positions']

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('paintingrobot_simulation', anonymous=True)
    Paintrobot = Renovationrobot_motion()
    try:
        # for i in range(len(manipulatorbase_targetpose[0])):
        #     for j in range(len(manipulatorbase_targetpose[0][i][0])):
        #         for k in range(len(manipulatorbase_targetpose[0][i][0][j][0])):
        for i in range(1):
            for j in range(1):
                for k in range(1):
                    manipulatorbase_targetpose_onecell = manipulatorbase_targetpose[0][i][0][j][0][k]
                    manipulatorendeffector_targetpose_onecell = manipulatorendeffector_targetpose[0][i][0][j][0][k]
                    for n1 in range(len(manipulatorbase_targetpose_onecell)):
                        for n2 in range(3):
                            manipulatorbase_targetpose_onecell[n1][n2] = manipulatorbase_targetpose_onecell[n1][n2] * 0.001
                    for m1 in range(len(manipulatorendeffector_targetpose_onecell)):
                        for m2 in range(3):
                            manipulatorendeffector_targetpose_onecell[m1][m2]=manipulatorendeffector_targetpose_onecell[m1][m2] * 0.001
                    print(manipulatorbase_targetpose_onecell)
                    print(manipulatorendeffector_targetpose_onecell)
                    # Paintrobot.show_obstacles()
                    Paintrobot.motion(manipulatorbase_targetpose_onecell,manipulatorendeffector_targetpose_onecell)
    except rospy.ROSInterruptException:
        pass
    moveit_commander.roscpp_shutdown()
    moveit_commander.os._exit(0)







    # def show_obstacles(self):
    #     # 初始化场景对象
    #     scene = PlanningSceneInterface()
    #     # 创建一个发布场景变化信息的发布者
    #     scene_pub = rospy.Publisher('planning_scene', PlanningScene, queue_size=5)
    #
    #     # 创建一个存储物体颜色的字典对象
    #     colors = dict()
    #     # 设置场景物体的名称
    #     table_id = 'table'
    #     box1_id = 'box1'
    #     box2_id = 'box2'
    #     # 设置桌面的高度
    #     table_ground = 0.25
    #     # 设置table、box1和box2的三维尺寸
    #     table_size = [0.2, 0.7, 0.01]
    #     box1_size = [0.1, 0.05, 0.05]
    #     box2_size = [0.05, 0.05, 0.15]
    #
    #     # 将三个物体加入场景当中
    #     table_pose = PoseStamped()
    #     table_pose.header.frame_id = 'base_link'
    #     table_pose.pose.position.x = 0.26
    #     table_pose.pose.position.y = 0.0
    #     table_pose.pose.position.z = table_ground + table_size[2] / 2.0
    #     table_pose.pose.orientation.w = 1.0
    #     scene.add_box(table_id, table_pose, table_size)
    #
    #     box1_pose = PoseStamped()
    #     box1_pose.header.frame_id = 'base_link'
    #     box1_pose.pose.position.x = 0.21
    #     box1_pose.pose.position.y = -0.1
    #     box1_pose.pose.position.z = table_ground + table_size[2] + box1_size[2] / 2.0
    #     box1_pose.pose.orientation.w = 1.0
    #     scene.add_box(box1_id, box1_pose, box1_size)
    #
    #     box2_pose = PoseStamped()
    #     box2_pose.header.frame_id = 'base_link'
    #     box2_pose.pose.position.x = 0.19
    #     box2_pose.pose.position.y = 0.15
    #     box2_pose.pose.position.z = table_ground + table_size[2] + box2_size[2] / 2.0
    #     box2_pose.pose.orientation.w = 1.0
    #     scene.add_box(box2_id, box2_pose, box2_size)
    #
    #     # 将桌子设置成红色，两个box设置成橙色
    #     colors[table_id]=self.setColor(0.8, 0, 0, 1.0)
    #     colors[box1_id]=self.setColor(0.8, 0.4, 0, 1.0)
    #     colors[box2_id]=self.setColor(0.8, 0.4, 0, 1.0)
    #     # 初始化规划场景对象
    #     p = PlanningScene()
    #     # 需要设置规划场景是否有差异
    #     p.is_diff = True
    #     # 从颜色字典中取出颜色设置
    #     for color in colors.values():
    #         p.object_colors.append(color)
    #     scene_pub.publish(p)
    #
    # # 设置场景物体的颜色
    # def setColor(self, name, r, g, b, a=0.9):
    #     color = ObjectColor()
    #     color.id = name
    #     color.color.r = r
    #     color.color.g = g
    #     color.color.b = b
    #     color.color.a = a
    #     return color

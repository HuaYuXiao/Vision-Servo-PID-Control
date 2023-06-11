#!/usr/bin/env python3
import sys
import rospy
import time
from kortex_driver.srv import *
from kortex_driver.msg import *
import cv2


class ExampleCartesianActionsWithNotifications:
    def __init__(self):
        try:
            rospy.init_node('example_cartesian_poses_with_notifications_python')

            self.HOME_ACTION_IDENTIFIER = 2

            self.action_topic_sub = None
            self.all_notifs_succeeded = True

            self.all_notifs_succeeded = True

            # Get node params
            self.robot_name = rospy.get_param('~robot_name', "my_gen3")

            rospy.loginfo("Using robot_name " + self.robot_name)

            # Init the action topic subscriber
            self.action_topic_sub = rospy.Subscriber("/" + self.robot_name + "/action_topic", ActionNotification, self.cb_action_topic)
            self.last_action_notif_type = None

            # Init the services
            clear_faults_full_name = '/' + self.robot_name + '/base/clear_faults'
            rospy.wait_for_service(clear_faults_full_name)
            self.clear_faults = rospy.ServiceProxy(clear_faults_full_name, Base_ClearFaults)

            read_action_full_name = '/' + self.robot_name + '/base/read_action'
            rospy.wait_for_service(read_action_full_name)
            self.read_action = rospy.ServiceProxy(read_action_full_name, ReadAction)

            execute_action_full_name = '/' + self.robot_name + '/base/execute_action'
            rospy.wait_for_service(execute_action_full_name)
            self.execute_action = rospy.ServiceProxy(execute_action_full_name, ExecuteAction)

            set_cartesian_reference_frame_full_name = '/' + self.robot_name + '/control_config/set_cartesian_reference_frame'
            rospy.wait_for_service(set_cartesian_reference_frame_full_name)
            self.set_cartesian_reference_frame = rospy.ServiceProxy(set_cartesian_reference_frame_full_name, SetCartesianReferenceFrame)

            activate_publishing_of_action_notification_full_name = '/' + self.robot_name + '/base/activate_publishing_of_action_topic'
            rospy.wait_for_service(activate_publishing_of_action_notification_full_name)
            self.activate_publishing_of_action_notification = rospy.ServiceProxy(activate_publishing_of_action_notification_full_name, OnNotificationActionTopic)
        
            #add
            self.is_gripper_present = rospy.get_param("/" + self.robot_name + "/is_gripper_present", False)
            send_gripper_command_full_name = '/' + self.robot_name + '/base/send_gripper_command'
            rospy.wait_for_service(send_gripper_command_full_name)
            self.send_gripper_command = rospy.ServiceProxy(send_gripper_command_full_name, SendGripperCommand)
        
        except:
            self.is_init_success = False
        else:
            self.is_init_success = True
            
    def example_send_gripper_command(self, value):
        # Initialize the request
        # Close the gripper
        req = SendGripperCommandRequest()
        finger = Finger()
        finger.finger_identifier = 0
        finger.value = value
        req.input.gripper.finger.append(finger)
        req.input.mode = GripperMode.GRIPPER_POSITION

        rospy.loginfo("Sending the gripper command...")

        # Call the service 
        try:
            self.send_gripper_command(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call SendGripperCommand")
            return False
        else:
            time.sleep(1)
            return True

    def cb_action_topic(self, notif):
        self.last_action_notif_type = notif.action_event

    def wait_for_action_end_or_abort(self):
        while not rospy.is_shutdown():
            if (self.last_action_notif_type == ActionEvent.ACTION_END):
                rospy.loginfo("Received ACTION_END notification")
                return True
            elif (self.last_action_notif_type == ActionEvent.ACTION_ABORT):
                rospy.loginfo("Received ACTION_ABORT notification")
                self.all_notifs_succeeded = False
                return False
            else:
                time.sleep(0.01)

    def example_clear_faults(self):
        try:
            self.clear_faults()
        except rospy.ServiceException:
            rospy.logerr("Failed to call ClearFaults")
            return False
        else:
            rospy.loginfo("Cleared the faults successfully")
            rospy.sleep(2.5)
            return True

    def example_home_the_robot(self):
        # The Home Action is used to home the robot. It cannot be deleted and is always ID #2:
        req = ReadActionRequest()
        req.input.identifier = self.HOME_ACTION_IDENTIFIER
        self.last_action_notif_type = None
        try:
            res = self.read_action(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call ReadAction")
            return False
        # Execute the HOME action if we could read it
        else:
            # What we just read is the input of the ExecuteAction service
            req = ExecuteActionRequest()
            req.input = res.output
            rospy.loginfo("Sending the robot home...")
            try:
                self.execute_action(req)
            except rospy.ServiceException:
                rospy.logerr("Failed to call ExecuteAction")
                return False
            else:
                return self.wait_for_action_end_or_abort()

    def example_set_cartesian_reference_frame(self):
        # Prepare the request with the frame we want to set
        req = SetCartesianReferenceFrameRequest()
        req.input.reference_frame = CartesianReferenceFrame.CARTESIAN_REFERENCE_FRAME_MIXED

        # Call the service
        try:
            self.set_cartesian_reference_frame()
        except rospy.ServiceException:
            rospy.logerr("Failed to call SetCartesianReferenceFrame")
            return False
        else:
            rospy.loginfo("Set the cartesian reference frame successfully")
            return True

        # Wait a bit
        rospy.sleep(0.25)

    def example_subscribe_to_a_robot_notification(self):
        # Activate the publishing of the ActionNotification
        req = OnNotificationActionTopicRequest()
        rospy.loginfo("Activating the action notifications...")
        try:
            self.activate_publishing_of_action_notification(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call OnNotificationActionTopic")
            return False
        else:
            rospy.loginfo("Successfully activated the Action Notifications!")

        rospy.sleep(1.0)

        return True

    def iniMain(self):
        # For testing purposes
        success = self.is_init_success
        try:
            rospy.delete_param("/kortex_examples_test_results/cartesian_poses_with_notifications_python")
        except:
            pass

        if success:
            # Make sure to clear the robot's faults else it won't move if it's already in fault
            success &= self.example_clear_faults()

            # Set the reference frame to "Mixed"
            success &= self.example_set_cartesian_reference_frame()

            # Subscribe to ActionNotification's from the robot to know when a cartesian pose is finished
            success &= self.example_subscribe_to_a_robot_notification()

            if self.is_gripper_present:
                success &= self.example_send_gripper_command(0.3)
            else:
                rospy.logwarn("No gripper is present on the arm.")
            
            # Prepare and send pose 1
            my_cartesian_speed = CartesianSpeed()
            my_cartesian_speed.translation = 0.1 # m/s
            my_cartesian_speed.orientation = 15  # deg/s

            my_constrained_pose = ConstrainedPose()
            my_constrained_pose.constraint.oneof_type.speed.append(my_cartesian_speed)
            
            #read local position
            feedback = rospy.wait_for_message("/" + self.robot_name + "/base_feedback", BaseCyclic_Feedback)
            
            my_constrained_pose.target_pose.x = 0.25
            my_constrained_pose.target_pose.y = -0.10
            my_constrained_pose.target_pose.z = 0.25
            my_constrained_pose.target_pose.theta_x = 0
            my_constrained_pose.target_pose.theta_y = -180
            my_constrained_pose.target_pose.theta_z = 85

            req = ExecuteActionRequest()
            req.input.oneof_action_parameters.reach_pose.append(my_constrained_pose)
            req.input.name = "pose1"
            req.input.handle.action_type = ActionType.REACH_POSE 
            req.input.handle.identifier = 1001

            rospy.loginfo("Sending pose 1...")
            self.last_action_notif_type = None
            try:
                self.execute_action(req)
            except rospy.ServiceException:
                rospy.logerr("Failed to send pose 1")
                success = False
            else:
                rospy.loginfo("Waiting for pose 1 to finish...")

            self.wait_for_action_end_or_abort()

            success &= self.all_notifs_succeeded

            success &= self.all_notifs_succeeded

        # For testing purposes
        rospy.set_param("/kortex_examples_test_results/cartesian_poses_with_notifications_python", success)

        if not success:
            rospy.logerr("The example encountered an error.")
            
    def move(self,errX,errY):
        # For testing purposes
        success = self.is_init_success
        try:
            rospy.delete_param("/kortex_examples_test_results/cartesian_poses_with_notifications_python")
        except:
            pass

        if success:
            # Prepare and send pose 1
            my_cartesian_speed = CartesianSpeed()
            my_cartesian_speed.translation = 0.3 # m/s
            my_cartesian_speed.orientation = 50  # deg/s

            my_constrained_pose = ConstrainedPose()
            my_constrained_pose.constraint.oneof_type.speed.append(my_cartesian_speed)
            
            #read local position
            feedback = rospy.wait_for_message("/" + self.robot_name + "/base_feedback", BaseCyclic_Feedback)
            P = 10000
         
            my_constrained_pose.target_pose.y = feedback.base.commanded_tool_pose_y - errX/P
        
            my_constrained_pose.target_pose.x = feedback.base.commanded_tool_pose_x - errY/P
            
            my_constrained_pose.target_pose.z = feedback.base.commanded_tool_pose_z
            my_constrained_pose.target_pose.theta_x = feedback.base.commanded_tool_pose_theta_x
            my_constrained_pose.target_pose.theta_y = feedback.base.commanded_tool_pose_theta_y
            my_constrained_pose.target_pose.theta_z = feedback.base.commanded_tool_pose_theta_z

            req = ExecuteActionRequest()
            req.input.oneof_action_parameters.reach_pose.append(my_constrained_pose)
            req.input.name = "pose1"
            req.input.handle.action_type = ActionType.REACH_POSE 
            req.input.handle.identifier = 1001

            rospy.loginfo("Sending pose 1...")
            self.last_action_notif_type = None
            try:
                self.execute_action(req)
            except rospy.ServiceException:
                rospy.logerr("Failed to send pose 1")
                success = False
            else:
                rospy.loginfo("Waiting for pose 1 to finish...")

            self.wait_for_action_end_or_abort()

            success &= self.all_notifs_succeeded

            success &= self.all_notifs_succeeded

        # For testing purposes
        rospy.set_param("/kortex_examples_test_results/cartesian_poses_with_notifications_python", success)

        if not success:
            rospy.logerr("The example encountered an error.")
            
    def spin(self):
        # For testing purposes
        success = self.is_init_success
        try:
            rospy.delete_param("/kortex_examples_test_results/cartesian_poses_with_notifications_python")
        except:
            pass

        if success:
            # Prepare and send pose 1
            my_cartesian_speed = CartesianSpeed()
            my_cartesian_speed.translation = 0.3 # m/s
            my_cartesian_speed.orientation = 50  # deg/s

            my_constrained_pose = ConstrainedPose()
            my_constrained_pose.constraint.oneof_type.speed.append(my_cartesian_speed)
            
            #read local position
            feedback = rospy.wait_for_message("/" + self.robot_name + "/base_feedback", BaseCyclic_Feedback)
            P = 10000

            my_constrained_pose.target_pose.y = feedback.base.commanded_tool_pose_y
            my_constrained_pose.target_pose.x = feedback.base.commanded_tool_pose_x
            my_constrained_pose.target_pose.z = feedback.base.commanded_tool_pose_z
            my_constrained_pose.target_pose.theta_x = feedback.base.commanded_tool_pose_theta_x
            my_constrained_pose.target_pose.theta_y = feedback.base.commanded_tool_pose_theta_y
            my_constrained_pose.target_pose.theta_z = feedback.base.commanded_tool_pose_theta_z - 10

            req = ExecuteActionRequest()
            req.input.oneof_action_parameters.reach_pose.append(my_constrained_pose)
            req.input.name = "pose1"
            req.input.handle.action_type = ActionType.REACH_POSE 
            req.input.handle.identifier = 1001

            rospy.loginfo("Sending pose 1...")
            self.last_action_notif_type = None
            try:
                self.execute_action(req)
            except rospy.ServiceException:
                rospy.logerr("Failed to send pose 1")
                success = False
            else:
                rospy.loginfo("Waiting for pose 1 to finish...")

            self.wait_for_action_end_or_abort()

            success &= self.all_notifs_succeeded

            success &= self.all_notifs_succeeded

        # For testing purposes
        rospy.set_param("/kortex_examples_test_results/cartesian_poses_with_notifications_python", success)

        if not success:
            rospy.logerr("The example encountered an error.")

    def down(self,z=0,grip=0):
        # For testing purposes
        global downFlag
        downFlag = 1
        success = self.is_init_success
        try:
            rospy.delete_param("/kortex_examples_test_results/cartesian_poses_with_notifications_python")
        except:
            pass

        if success:
            # Prepare and send pose 1
            my_cartesian_speed = CartesianSpeed()
            my_cartesian_speed.translation = 0.3 # m/s
            my_cartesian_speed.orientation = 50  # deg/s

            my_constrained_pose = ConstrainedPose()
            my_constrained_pose.constraint.oneof_type.speed.append(my_cartesian_speed)
            
            #read local position
            feedback = rospy.wait_for_message("/" + self.robot_name + "/base_feedback", BaseCyclic_Feedback)
            P = 10000
            
            my_constrained_pose.target_pose.y = feedback.base.commanded_tool_pose_y + 0.04
        
            my_constrained_pose.target_pose.x = feedback.base.commanded_tool_pose_x + 0.05
            
            my_constrained_pose.target_pose.z = feedback.base.commanded_tool_pose_z - 0.25 - z
            my_constrained_pose.target_pose.theta_x = feedback.base.commanded_tool_pose_theta_x
            my_constrained_pose.target_pose.theta_y = feedback.base.commanded_tool_pose_theta_y
            my_constrained_pose.target_pose.theta_z = feedback.base.commanded_tool_pose_theta_z

            req = ExecuteActionRequest()
            req.input.oneof_action_parameters.reach_pose.append(my_constrained_pose)
            req.input.name = "pose1"
            req.input.handle.action_type = ActionType.REACH_POSE 
            req.input.handle.identifier = 1001

            rospy.loginfo("Sending pose 1...")
            self.last_action_notif_type = None
            try:
                self.execute_action(req)
            except rospy.ServiceException:
                rospy.logerr("Failed to send pose 1")
                success = False
            else:
                rospy.loginfo("Waiting for pose 1 to finish...")

            self.wait_for_action_end_or_abort()
            
            if self.is_gripper_present:
                success &= self.example_send_gripper_command(0.6+grip)
            else:
                rospy.logwarn("No gripper is present on the arm.")   
            
             # Prepare and send pose 2
            req.input.handle.identifier = 1002
            req.input.name = "pose2"

            my_constrained_pose.target_pose.z = 0.35
            my_constrained_pose.target_pose.theta_x = 65
            my_constrained_pose.target_pose.theta_y = -165
            my_constrained_pose.target_pose.theta_z = 75
            my_constrained_pose.target_pose.x = 0.52
            my_constrained_pose.target_pose.y = -0.32
            
            req.input.oneof_action_parameters.reach_pose[0] = my_constrained_pose

            rospy.loginfo("Sending pose 2...")
            self.last_action_notif_type = None
            try:
                self.execute_action(req)
            except rospy.ServiceException:
                rospy.logerr("Failed to send pose 2")
                success = False
            else:
                rospy.loginfo("Waiting for pose 2 to finish...")

            self.wait_for_action_end_or_abort()
            
            success &= self.all_notifs_succeeded

            success &= self.all_notifs_succeeded
                        
        # For testing purposes
        rospy.set_param("/kortex_examples_test_results/cartesian_poses_with_notifications_python", success)

        if not success:
            rospy.logerr("The example encountered an error.")

if __name__ == "__main__":
    obj = input("please tell me what you need\n")
    thres = 0.6 # Threshold to detect object0.45

    cap = cv2.VideoCapture(6)
    cap.set(3,1280)
    cap.set(4,720)
    cap.set(10,-20)
    cap.set(6,cv2.VideoWriter_fourcc('M','J','P','G'))
    cap.set(38,1)

    ex = ExampleCartesianActionsWithNotifications()
    ex.iniMain()

    classNames= []
    classFile = '/home/ee368-7/Desktop/EE368_lab/project/Resources/coco.names'
    with open(classFile,'rt') as f:
        classNames = f.read().rstrip('\n').split('\n')
    print(classNames)
    configPath = '/home/ee368-7/Desktop/EE368_lab/project/Resources/ssd_mobilenet_v3_large_coco_2020_01_14.pbtxt'
    weightsPath = '/home/ee368-7/Desktop/EE368_lab/project/Resources/frozen_inference_graph.pb'

    net = cv2.dnn_DetectionModel(weightsPath,configPath)
    net.setInputSize(320,320)
    net.setInputScale(1.0/ 127.5)
    net.setInputMean((127.5, 127.5, 127.5))
    net.setInputSwapRB(True)

    apple = [0, [0,0,0,0],[0,0],[0,0]]
    banana = [0, [0,0,0,0],[0,0],[0,0]]

    global downFlag = 0
    
    if "apple" in obj:
        apple[0] = 1
    elif "banana" in obj:
        banana[0] = 1
        
    try:
        while True:
            success,img = cap.read()

            h, w, _ = img.shape
            classIds, confs, bbox = net.detect(img,confThreshold=thres)
            # print(classIds,bbox)

            if len(classIds) != 0:
                for classId, confidence,box in zip(classIds.flatten(),confs.flatten(),bbox):
                    # print(classId)
                    cv2.rectangle(img,box,color=(0,255,0),thickness=2) #
                    cv2.putText(img,
                                classNames[classId-1].upper(),
                                (box[0]+10,box[1]+30),
                                cv2.FONT_HERSHEY_COMPLEX,
                                1,
                                (0,255,0),
                                2)
                    cv2.putText(img,
                                str(round(confidence*100,2)),
                                (box[0]+200,box[1]+30),
                                cv2.FONT_HERSHEY_COMPLEX,
                                1,
                                (0,255,0),
                                2)
                    
                    if (classId - 1) == 51 and banana[0]==1:
                        banana[1][0], banana[1][1], banana[1][2], banana[1][3] = box[0], box[1], box[2], box[3]
                        banana[2][0] = banana[1][0] + banana[1][2]/2
                        banana[2][1] = banana[1][1] + banana[1][3]/2
                        banana[3][0] = banana[2][0] - w/2
                        banana[3][1] = banana[2][1] - h/2
                        print(banana)
                        if (abs(banana[3][0])>30 or abs(banana[3][1])>20) and downFlag==0:
                            ex.move(banana[3][0],banana[3][1])
                        elif downFlag == 0:
                            ex.down(z=0.01,grip=0.25)
                    
                    if (classId - 1) == 52 and apple[0]==1:
                        apple[1][0], apple[1][1], apple[1][2], apple[1][3] = box[0], box[1], box[2], box[3]
                        apple[2][0] = apple[1][0] + apple[1][2]/2
                        apple[2][1] = apple[1][1] + apple[1][3]/2
                        apple[3][0] = apple[2][0] - w/2
                        apple[3][1] = apple[2][1] - h/2
                        if (abs(apple[3][0])>30 or abs(apple[3][1])>20) and downFlag==0:
                            ex.move(apple[3][0],apple[3][1])
                        elif downFlag == 0:
                            ex.down()
                    cv2.imshow('Output',img)
                    cv2.waitKey(1)
    except KeyboardInterrupt:
        
        cv2.destroyAllWindows()
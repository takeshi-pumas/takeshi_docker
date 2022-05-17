#!/usr/bin/env python
from std_srvs.srv import Empty, Trigger, TriggerRequest
import smach
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped, Point , Quaternion
from actionlib_msgs.msg import GoalStatus
import moveit_commander
import moveit_msgs.msg
import tf2_ros
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from object_classification.srv import *
from sensor_msgs.msg import Image as ImageMsg
from cv_bridge import CvBridge, CvBridgeError

from utils_notebooks import *
from utils_takeshi import *
########## Functions for takeshi states ##########
class Proto_state(smach.State):###example of a state definition.
    def __init__(self):
        smach.State.__init__(self,outcomes=['succ','failed','tries'])
        self.tries=0
    def execute(self,userdata):
        rospy.loginfo('State : PROTO_STATE')

        if self.tries==3:
            self.tries=0 
            return'tries'
        if succ:
            return 'succ'
        else:
            return 'failed'
        global trans_hand
        self.tries+=1
        if self.tries==3:
            self.tries=0 
            return'tries'

####################################################################Functions in notebook ##################################################################

def classify_images(images):
    req=classify_client.request_class()
    for image in images:
        img_msg=bridge.cv2_to_imgmsg(image)
        req.in_.image_msgs.append(img_msg)
    resp1 = classify_client(req)
    class_resp= np.asarray(resp1.out.data)
    cont3=0
    class_labels=[]
    for cla in class_resp:
        
        if cont3==3:
            print '-----------------'
            cont3=0
        print (class_names [(int)(cla)])
        class_labels.append(class_names [(int)(cla)])
        cont3+=1  
    return class_resp  

def table_line_up(target_distance=0.5,target_link='Object0_Table_1'):
    ###Face towards Targetlink and get target distance close
    try:
        obj_pose,_ =  listener.lookupTransform('map',target_link,rospy.Time(0))
        print (obj_pose)
        new_pose= obj_pose
        new_pose[1]+=-target_distance
    except(tf.LookupException):
        print ('no  tf found')
        return False
    
    
    
    
    arm.set_named_target('go')
    arm.go()

    
    
    succ=move_base( new_pose[0],new_pose[1], 0.5*np.pi)
    return succ   








def find_category(dictionary, name):
    if name in dictionary:
        category=dictionary[name]

    else:  #go to Bin B
        category = 8

    return category

#To Do: 
#To Do: pack them in an utils file

############################################### NEW GRASP DETECTOR

def loop_hand():
    img_msg = rospy.wait_for_message('/hsrb/hand_camera/image_raw', ImageMsg, timeout=None)
    old_img = rospy.wait_for_message('/hsrb/hand_camera/image_raw', ImageMsg, timeout=None)
    while old_img == img_msg:
        #print("data is the same old data")
        img_msg = rospy.wait_for_message('/hsrb/hand_camera/image_raw', ImageMsg, timeout=None)
        rospy.sleep(0.2)
    old_img = img_msg
    return img_msg

"""def save_hand(i=0):
        img_msg = loop_hand()
        cv2_img = bridge.imgmsg_to_cv2(img_msg, "bgr8")
        img = np.asarray (cv2_img)
        #np.save('hand_camera_rgb_'+str(i)+'.npy',cv2_img)  
        #cv2.imwrite('hand_camera_rgb_'+str(i)+'.jpg', cv2_img) 
        print ('saving hand cam img',i) 
        return img  
    """    
def save_hand(k=0):
    
    img_msg = rospy.wait_for_message('/hsrb/hand_camera/image_raw', ImageMsg, timeout=None)
    cv2_img = bridge.imgmsg_to_cv2(img_msg, "bgr8")
    img = np.asarray (cv2_img)
    #np.save('hand_camera_rgb_'+str(i)+'.npy',cv2_img)  
    cv2.imwrite('hand_camera_rgb_'+str(k)+'.jpg', cv2_img) 
    
    return img  


def grasp_detector(img):

    Hmax, Smax, Vmax = 19, 153, 208
    Hmin, Smin, Vmin = 15, 121, 166
    cropped_image = img[75:110, 0:75,:]
    hsv_test = cv2.cvtColor(cropped_image, cv2.COLOR_BGR2HSV)
    

    arrH = hsv_test[:,:,0].reshape(2625)
    arrS = hsv_test[:,:,1].reshape(2625)
    arrV = hsv_test[:,:,2].reshape(2625)
  
    listH = arrH.tolist()
    listS = arrS.tolist()
    listV = arrV.tolist()
   


    resH = sum(1 for i in listH if i > Hmin and i < Hmax)
    resS = sum(1 for i in listS if (i > Smin and i < Smax))
    resV = sum(1 for i in listV if (i > Vmin and i < Vmax))

    # calculo porcentajes

    porcH = resH*100/len(listH)
    porcS = resS*100/len(listS)
    porcV = resV*100/len(listV)

    print(porcH,porcS,porcV)
 
   


    a = gripper.get_current_joint_values()
    if np.linalg.norm(a - np.asarray(grasped))  >  (np.linalg.norm(a - np.asarray(ungrasped))):
        print ('grasp seems to have failed')
        if porcH > 90 or porcS > 90 or porcV > 90:
            print ('and we are looking the floor')
            return False
        else:
            print("there is something")
            return True
    else:
        print('super primitive grasp detector points towards succesfull ')
        return True


def primitive_grasp_detector():
    a = gripper.get_current_joint_values()
    if np.linalg.norm(a - np.asarray(grasped))  >  (np.linalg.norm(a - np.asarray(ungrasped))):
        print ('grasp seems to have failed')
        return False
    else:
        print('super primitive grasp detector points towards succesfull ')
        return True

def add_object(name, size, pose, orientation):
    p = PoseStamped()
    p.header.frame_id = "map"       # "head_rgbd_sensor_link"
    
    p.pose.position.x = pose[0]
    p.pose.position.y = pose[1]
    p.pose.position.z = pose[2]

    p.pose.orientation.x = orientation[0] * np.pi
    p.pose.orientation.y = orientation[1] * np.pi
    p.pose.orientation.z = orientation[2] * np.pi
    p.pose.orientation.w = orientation[3] * np.pi

    scene.add_box(name, p, size)
def add_transform(child, trans, rot, parent="map"):
    static_transformStamped = TransformStamped()
    static_transformStamped.header.stamp = rospy.Time.now()
    static_transformStamped.header.frame_id = parent
    static_transformStamped.child_frame_id = child
    static_transformStamped.transform.translation.x = trans[0]
    static_transformStamped.transform.translation.y = trans[1]
    static_transformStamped.transform.translation.z = trans[2]
    static_transformStamped.transform.rotation.x = rot[0]    
    static_transformStamped.transform.rotation.y = rot[1]    
    static_transformStamped.transform.rotation.z = rot[2]    
    static_transformStamped.transform.rotation.w = rot[3]    
    tf_static_broadcaster.sendTransform(static_transformStamped)    

def publish_scene():
    add_object("shelf", [1.5, 0.04, 0.4],  [2.5, 4.85, 0.78],  [0.5,0,0,0.5])
    add_object("shelf1", [1.5, 0.04, 0.4], [2.5, 4.85, 0.49], [0.5,0,0, 0.5])
    add_object("shelf2", [1.5, 0.04, 0.4], [2.5, 4.85, 0.18], [0.5,0,0, 0.5])
    add_object("shelf_wall", [1, 1, 0.04], [2.5, 4.9, 0.5], [0.5,0,0, 0.5])
    add_object("shelf_wall1", [.04, 1, 0.4], [2.7, 4.9, 0.5],[0.5,0,0, 0.5])
    add_object("shelf_wall2", [.04, 1, 0.4], [1.8, 4.9, 0.5], [0.5,0,0 ,0.5])    
    add_object("table_big", [1.5, 0.3, 0.5], [0.95, 1.9, 0.34],  [0.5,0,0, 0.5])
    add_object("table_big_legs1",[.01,.6,.2], [1.55,1.8,0.1],       [0.5,0,0, 0.5])
    add_object("table_big_legs2",[.01,.6,.2], [0.45,1.8,0.1],       [0.5,0,0 ,0.5])
    add_object("table_small", [0.9, 0.02, 0.4], [-0.2, 1.85, 0.61],  [0.5,0,0 ,0.5])
    add_object("table_small_legs1",[.01,.6,.2], [-0.3,1.75,0.3],      [0.5,0,0, 0.5])
    add_object("table_small_legs2",[.01,.6,.2], [0.1,1.75,0.3], [0.5,0,0 ,0.5])
    add_object("table_tray", [0.65, 0.01, 0.7], [1.8, -0.65, 0.4], [0.5,0,0, 0.5])
    add_object("containers", [0.3, 0.3, 0.3], [1.4, -0.65, 0.4], [0.5,0,0, 0.5])
    add_object("drawers", [1, 1, 1], [0, -0.65, 0.5], [0.5,0,0, 0.5])

    add_object("big_wall" , [6.0, 0.2, 0.2], [3.2,  2.0, 0.0],  [0,0.0,0.5 ,0.5])
    add_object("mid_wall" , [4.0, 0.2, 0.2], [0.1,  2.1, 0.0],  [0,0.0,0.0 ,1/np.pi])
    add_object("door_wall" , [5.0, 0.2, 0.2], [-0.8, 2.8, 0.0],  [0,0.0,0.5 ,0.5     ])
    add_object("close_wall", [4.0, 0.2, 0.2], [1.1, -0.5, 0.0],  [0,0.0,0.0 ,1/np.pi])
    add_object("far_wall",   [4.0, 0.2, 0.2], [1.1, 5.0, 0.0],  [0,0.0,0.0 ,1/np.pi])
    
    add_transform("Tray_A", [1.665, -0.59, 0.4], [0, 0, 0, 1])
    add_transform("Tray_B", [1.97, -0.59, 0.4], [0, 0, 0, 1])

    static_transformStamped=TransformStamped()

      ##FIXING TF TO MAP ( ODOM REALLY)    
    static_transformStamped.header.stamp = rospy.Time.now()
    static_transformStamped.header.frame_id = "map"
    static_transformStamped.child_frame_id = "Drawer_low" 
    static_transformStamped.transform.translation.x = 0.14
    static_transformStamped.transform.translation.y = -0.344
    static_transformStamped.transform.translation.z = 0.27
    static_transformStamped.transform.rotation.x = 0    
    static_transformStamped.transform.rotation.y = 0    
    static_transformStamped.transform.rotation.z = 0    
    static_transformStamped.transform.rotation.w = 1    
    tf_static_broadcaster.sendTransform(static_transformStamped)

    static_transformStamped.header.stamp = rospy.Time.now()
    static_transformStamped.header.frame_id = "map"
    static_transformStamped.child_frame_id = "Box1" 
    static_transformStamped.transform.translation.x = 2.3
    static_transformStamped.transform.translation.y = -0.5
    static_transformStamped.transform.translation.z = .5
    static_transformStamped.transform.rotation.x = 0    
    static_transformStamped.transform.rotation.y = 0    
    static_transformStamped.transform.rotation.z = 0    
    static_transformStamped.transform.rotation.w = 1    
    tf_static_broadcaster.sendTransform(static_transformStamped)  

    static_transformStamped.header.stamp = rospy.Time.now()
    static_transformStamped.header.frame_id = "map"
    static_transformStamped.child_frame_id = "Drawer_left" 
    static_transformStamped.transform.translation.x = .48
    static_transformStamped.transform.translation.y = -0.39
    static_transformStamped.transform.translation.z = .27
    static_transformStamped.transform.rotation.y = 0    
    static_transformStamped.transform.rotation.z = 0    
    static_transformStamped.transform.rotation.w = 1    
    tf_static_broadcaster.sendTransform(static_transformStamped)  

    static_transformStamped = TransformStamped()

    static_transformStamped.header.stamp = rospy.Time.now()
    static_transformStamped.header.frame_id = "map"
    static_transformStamped.child_frame_id = "Drawer_high" 
    static_transformStamped.transform.translation.x = 0.14
    static_transformStamped.transform.translation.y = -0.39
    static_transformStamped.transform.translation.z = 0.52
    static_transformStamped.transform.rotation.x = 0    
    static_transformStamped.transform.rotation.y = 0    
    static_transformStamped.transform.rotation.z = 0    
    static_transformStamped.transform.rotation.w = 1    

    tf_static_broadcaster.sendTransform(static_transformStamped)


    return True

def get_current_time_sec():

    return rospy.Time.now().to_sec()


def move_abs(vx,vy,vw, time=0.05):
    start_time = get_current_time_sec() 
    while get_current_time_sec() - start_time < time: 
        twist = Twist()
        twist.linear.x = vx
        twist.linear.y = vy
        twist.angular.z = vw / 180.0 * np.pi  
        base_vel_pub.publish(twist)

def pose_2_np(wp_p):
    
    #Takes a pose message and returns array
   
    return np.asarray((wp_p.pose.position.x,wp_p.pose.position.y,wp_p.pose.position.z)) , np.asarray((wp_p.pose.orientation.w,wp_p.pose.orientation.x,wp_p.pose.orientation.y, wp_p.pose.orientation.z)) 
def np_2_pose(position,orientation):
    #Takes a pose np array and returns pose message
    wb_p= geometry_msgs.msg.PoseStamped()
    
    wb_p.pose.position.x= position[0]
    wb_p.pose.position.y= position[1]
    wb_p.pose.position.z= position[2]
    wb_p.pose.orientation.w= orientation[0]
    wb_p.pose.orientation.x= orientation[1]
    wb_p.pose.orientation.y= orientation[2]
    wb_p.pose.orientation.z= orientation[3]
    return wb_p

def open_gripper():
    target_motor=1
    gripper.set_start_state_to_current_state()
    try:
        gripper.set_joint_value_target({'hand_motor_joint':target_motor})
    except:
        print('OOB')
    succ=gripper.go()
def close_gripper():
    target_motor=0.0
    gripper.set_start_state_to_current_state()
    try:
        gripper.set_joint_value_target({'hand_motor_joint':target_motor})
    except:
        print('OOB')
    succ=gripper.go()
def correct_points(low_plane=.0,high_plane=0.2):

    #Corrects point clouds "perspective" i.e. Reference frame head is changed to reference frame map
    data = rospy.wait_for_message('/hsrb/head_rgbd_sensor/depth_registered/rectified_points', PointCloud2)
    np_data=ros_numpy.numpify(data)
    trans,rot=listener.lookupTransform('/map', '/head_rgbd_sensor_gazebo_frame', rospy.Time(0)) 
    
    eu=np.asarray(tf.transformations.euler_from_quaternion(rot))
    t=TransformStamped()
    rot=tf.transformations.quaternion_from_euler(-eu[1],0,0)
    t.header.stamp = data.header.stamp
    
    t.transform.rotation.x = rot[0]
    t.transform.rotation.y = rot[1]
    t.transform.rotation.z = rot[2]
    t.transform.rotation.w = rot[3]

    cloud_out = do_transform_cloud(data, t)
    np_corrected=ros_numpy.numpify(cloud_out)
    corrected=np_corrected.reshape(np_data.shape)

    img= np.copy(corrected['y'])

    img[np.isnan(img)]=2
    #img3 = np.where((img>low)&(img< 0.99*(trans[2])),img,255)
    img3 = np.where((img>0.99*(trans[2])-high_plane)&(img< 0.99*(trans[2])-low_plane),img,255)
    return img3

def plane_seg_square_imgs(lower=500 ,higher=50000,reg_ly= 30,reg_hy=600,plt_images=True,low_plane=.0,high_plane=0.2):

    #Segment  Plane using corrected point cloud
    #Lower, higher = min, max area of the box
    #reg_ly= 30,reg_hy=600    Region (low y  region high y ) Only centroids within region are accepted
    
    image= rgbd.get_h_image()
    iimmg= rgbd.get_image()
    points_data= rgbd.get_points()
    img=np.copy(image)
    img3= correct_points(low_plane,high_plane)


    _,contours, hierarchy = cv2.findContours(img3.astype('uint8'),cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    i=0
    cents=[]
    points=[]
    images=[]
    for i, contour in enumerate(contours):

        area = cv2.contourArea(contour)

        if area > lower and area < higher :
            M = cv2.moments(contour)
            # calculate x,y coordinate of center
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])


            boundRect = cv2.boundingRect(contour)
            #just for drawing rect, dont waste too much time on this
            img=cv2.rectangle(img,(boundRect[0], boundRect[1]),(boundRect[0]+boundRect[2], boundRect[1]+boundRect[3]), (0,0,0), 2)
            # calculate moments for each contour
            if (cY > reg_ly and cY < reg_hy  ):

                image_aux= iimmg[boundRect[1]:boundRect[1]+max(boundRect[2],boundRect[3]),boundRect[0]:boundRect[0]+max(boundRect[2],boundRect[3])]
                images.append(image_aux)
                cv2.circle(img, (cX, cY), 5, (255, 255, 255), -1)
                cv2.putText(img, "centroid_"+str(i)+"_"+str(cX)+','+str(cY)    ,    (cX - 25, cY - 25)   ,cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 0), 2)
                #print ('cX,cY',cX,cY)
                xyz=[]


                for jy in range (boundRect[0], boundRect[0]+boundRect[2]):
                    for ix in range(boundRect[1], boundRect[1]+boundRect[3]):
                        aux=(np.asarray((points_data['x'][ix,jy],points_data['y'][ix,jy],points_data['z'][ix,jy])))
                        if np.isnan(aux[0]) or np.isnan(aux[1]) or np.isnan(aux[2]):
                            'reject point'
                        else:
                            xyz.append(aux)

                xyz=np.asarray(xyz)
                cent=xyz.mean(axis=0)
                cents.append(cent)
                #print (cent)
                points.append(xyz)
            #else:
                #print ('cent out of region... rejected')
    sub_plt=0
    """if plt_images:
                    for image in images:
            
                        sub_plt+=1
                        ax = plt.subplot(5, 5, sub_plt )
            
                        plt.imshow(image)
                        plt.axis("off")"""

    cents=np.asarray(cents)
    ### returns centroids found and a group of 3d coordinates that conform the centroid
    return(cents,np.asarray(points), images)
def seg_square_imgs(lower=2000,higher=50000,reg_ly=0,reg_hy=1000,reg_lx=0,reg_hx=1000,plt_images=False): 

    #Using kmeans for image segmentation find
    #Lower, higher = min, max area of the box
    #reg_ly= 30,reg_hy=600,reg_lx=0,reg_hx=1000,    Region (low  x,y  region high x,y ) Only centroids within region are accepted
    image= rgbd.get_h_image()
    iimmg= rgbd.get_image()
    points_data= rgbd.get_points()
    values=image.reshape((-1,3))
    values= np.float32(values)
    criteria= (  cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER  ,1000,0.1)
    k=6
    _ , labels , cc =cv2.kmeans(values , k ,None,criteria,30,cv2.KMEANS_RANDOM_CENTERS)
    cc=np.uint8(cc)
    segmented_image= cc[labels.flatten()]
    segmented_image=segmented_image.reshape(image.shape)
    th3 = cv2.adaptiveThreshold(segmented_image,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY,11,2)
    kernel = np.ones((5,5),np.uint8)
    im4=cv2.erode(th3,kernel,iterations=4)
    plane_mask=points_data['z']
    cv2_img=plane_mask.astype('uint8')
    img=im4
    _,contours, hierarchy = cv2.findContours(im4.astype('uint8'),cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    i=0
    cents=[]
    points=[]
    images=[]
    for i, contour in enumerate(contours):

        area = cv2.contourArea(contour)

        if area > lower and area < higher :
            M = cv2.moments(contour)
            # calculate x,y coordinate of center
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])


            boundRect = cv2.boundingRect(contour)
            #just for drawing rect, dont waste too much time on this
            image_aux= iimmg[boundRect[1]:boundRect[1]+max(boundRect[3],boundRect[2]),boundRect[0]:boundRect[0]+max(boundRect[3],boundRect[2])]
            images.append(image_aux)
            img=cv2.rectangle(img,(boundRect[0], boundRect[1]),(boundRect[0]+boundRect[2], boundRect[1]+boundRect[3]), (0,0,0), 2)
            #img=cv2.rectangle(img,(boundRect[0], boundRect[1]),(boundRect[0]+max(boundRect[2],boundRect[3]), boundRect[1]+max(boundRect[2],boundRect[3])), (0,0,0), 2)
            # calculate moments for each contour
            if (cY > reg_ly and cY < reg_hy and  cX > reg_lx and cX < reg_hx   ):

                cv2.circle(img, (cX, cY), 5, (255, 255, 255), -1)
                cv2.putText(img, "centroid_"+str(i)+"_"+str(cX)+','+str(cY)    ,    (cX - 25, cY - 25)   ,cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 0), 2)
                #print ('cX,cY',cX,cY)
                xyz=[]


                for jy in range (boundRect[0], boundRect[0]+boundRect[2]):
                    for ix in range(boundRect[1], boundRect[1]+boundRect[3]):
                        aux=(np.asarray((points_data['x'][ix,jy],points_data['y'][ix,jy],points_data['z'][ix,jy])))
                        if np.isnan(aux[0]) or np.isnan(aux[1]) or np.isnan(aux[2]):
                            'reject point'
                        else:
                            xyz.append(aux)

                xyz=np.asarray(xyz)
                cent=xyz.mean(axis=0)
                cents.append(cent)
                #print (cent)
                points.append(xyz)
            else:
                #print ('cent out of region... rejected')
                images.pop()
    sub_plt=0
    if plt_images:
        for image in images:

            sub_plt+=1
            ax = plt.subplot(5, 5, sub_plt )

            plt.imshow(image)
            plt.axis("off")

    cents=np.asarray(cents)
    #images.append(img)
    return(cents,np.asarray(points), images)


def gaze_point(x,y,z):

    ###Moves head to make center point of rgbd image th coordinates w.r.t.map
    ### To do: (Start from current pose  instead of always going to neutral first )
    
    
    
    head_pose = head.get_current_joint_values()
    head_pose[0]=0.0
    head_pose[1]=0.0
    head.set_joint_value_target(head_pose)
    head.go()
    
    trans , rot = listener.lookupTransform('/map', '/head_rgbd_sensor_gazebo_frame', rospy.Time(0)) #
    
  #  arm_pose=arm.get_current_joint_values()
  #  arm_pose[0]=.1
  #  arm_pose[1]= -0.3
  #  arm.set_joint_value_target(arm_pose)
  #  arm.go()
    
    e =tf.transformations.euler_from_quaternion(rot)
    #print('i am at',trans,np.rad2deg(e)[2])
    #print('gaze goal',x,y,z)
    #tf.transformations.euler_from_quaternion(rot)


    x_rob,y_rob,z_rob,th_rob= trans[0], trans[1] ,trans[2] ,  e[2]


    D_x=x_rob-x
    D_y=y_rob-y
    D_z=z_rob-z

    D_th= np.arctan2(D_y,D_x)
    #print('relative to robot',(D_x,D_y,np.rad2deg(D_th)))

    pan_correct= (- th_rob + D_th + np.pi) % (2*np.pi)

    if(pan_correct > np.pi):
        pan_correct=-2*np.pi+pan_correct
    if(pan_correct < -np.pi):
        pan_correct=2*np.pi+pan_correct

    if ((pan_correct) > .5 * np.pi):
        print ('Exorcist alert')
        pan_correct=.5*np.pi
    head_pose[0]=pan_correct
    tilt_correct=np.arctan2(D_z,np.linalg.norm((D_x,D_y)))

    head_pose [1]=-tilt_correct
    
    
    
    head.set_joint_value_target(head_pose)
    succ=head.go()
    return succ

"""def move_base_goal(x, y, theta):
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation = quaternion_from_euler(0, 0, theta)
    navclient.send_goal(goal)
    navclient.wait_for_result()
    state = navclient.get_state()
    return True if state == 3 else False"""

def move_base(x,y,theta,time_out=20):

    #using nav client and toyota navigation go to x,y,yaw
    #To Do: PUMAS NAVIGATION
    #pose = PoseStamped()
    #pose.header.stamp = rospy.Time(0)
    #pose.header.frame_id = "map"
    #pose.pose.position = Point(goal_x, goal_y, 0)
    #quat = tf.transformations.quaternion_from_euler(0, 0, goal_yaw)
    #pose.pose.orientation = Quaternion(*quat)  


    # create a MOVE BASE GOAL
    print ('MOVING TO ', x,y,theta)
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation = quaternion_from_euler(0, 0, theta*180/np.pi)
    navclient.send_goal(goal)
    navclient.wait_for_result(timeout=rospy.Duration(time_out))
    state = navclient.get_state()
    return True if state == 3 else False

    


def static_tf_publish(cents,table=False):

    ## Publish tfs of the centroids obtained w.r.t. head sensor frame and references them to map (static)
    trans , rot = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
    
    #closest_centroid_index=  np.argmin(np.linalg.norm(trans-cents, axis=1))##CLOSEST CENTROID
    closest_centroid_index,closest_centroid_index_table=0,0
    min_D_to_base,min_D_to_base_table=10,10
    for  i ,cent  in enumerate(cents):
        x,y,z=cent
        if np.isnan(x) or np.isnan(y) or np.isnan(z):
            print('nan')
        else:
            broadcaster.sendTransform((x,y,z),rot, rospy.Time.now(), 'Object'+str(i),"head_rgbd_sensor_link")
            rospy.sleep(.2)
            xyz_map,cent_quat= listener.lookupTransform('/map', 'Object'+str(i),rospy.Time(0))
            D_to_base=np.linalg.norm(np.asarray(trans)[:2]-np.asarray(xyz_map)[:2])
            if D_to_base <= min_D_to_base:
                min_D_to_base=D_to_base
                closest_centroid_index=i
                closest_centroid_height= xyz_map[2]
            if D_to_base <= min_D_to_base_table and xyz_map[2] >=0.4:
                min_D_to_base_table=D_to_base
                closest_centroid_index_table=i
                closest_centroid_height_table= xyz_map[2]
            

            #print ('D Base to obj - ',i, np.linalg.norm(np.asarray(trans)[:2]-np.asarray(xyz_map)[:2]))
    
    
    if table: i = closest_centroid_index_table
    else:i = closest_centroid_index
    try :xyz_map,cent_quat= listener.lookupTransform('/map', 'Object'+str(i),rospy.Time(0))
    except:print ('no tf in statif tf publisher')
    print  ('height closest centroid map',xyz_map[2],'index tf',i)
    map_euler=tf.transformations.euler_from_quaternion(cent_quat)
    rospy.sleep(.2)
    static_transformStamped = TransformStamped()
            
       
    ##FIXING TF TO MAP ( ODOM REALLY)    
    #tf_broadcaster1.sendTransform( (xyz[0],xyz[1],xyz[2]),tf.transformations.quaternion_from_euler(0, 0, 0), rospy.Time(0), "obj"+str(ind), "head_rgbd_sensor_link")
    static_transformStamped.header.stamp = rospy.Time.now()
    static_transformStamped.header.frame_id = "map"
    
    static_transformStamped.transform.translation.x = float(xyz_map[0])
    static_transformStamped.transform.translation.y = float(xyz_map[1])
    static_transformStamped.transform.translation.z = float(xyz_map[2])
    #quat = tf.transformations.quaternion_from_euler(-euler[0],0,1.5)
    static_transformStamped.transform.rotation.x = 0#-quat[0]#trans.transform.rotation.x
    static_transformStamped.transform.rotation.y = 0#-quat[1]#trans.transform.rotation.y
    static_transformStamped.transform.rotation.z = 0#-quat[2]#trans.transform.rotation.z
    static_transformStamped.transform.rotation.w = 1#-quat[3]#trans.transform.rotation.w
    if xyz_map[2] > .7 and xyz_map[2] < .85:
        static_transformStamped.child_frame_id = "Object_"+str(i)+"_Table_real_lab"
        tf_static_broadcaster.sendTransform(static_transformStamped)
        
    
    
    if xyz_map[2] > .4 and xyz_map[2] < 0.5888 and xyz_map[1] < 2.0 :   #table 1 
        static_transformStamped.child_frame_id = "Closest_Table_1"
        tf_static_broadcaster.sendTransform(static_transformStamped)
    if xyz_map[2] > .59 and xyz_map[2] < .8 and xyz_map[1] < 2.0:   #table 2 
        static_transformStamped.child_frame_id = "Closest_Table_2"
        tf_static_broadcaster.sendTransform(static_transformStamped)
        
    if  xyz_map[2] < .25:   #Floor
        if xyz_map[1] < 1.62:
            static_transformStamped.child_frame_id = "Closest_Floor"
            tf_static_broadcaster.sendTransform(static_transformStamped)
           
        if  xyz_map[1] >= 1.63:
            static_transformStamped.child_frame_id = "Object_"+str(closest_centroid_index)+"_Floor_risky"
            tf_static_broadcaster.sendTransform(static_transformStamped)
                    
        
    return closest_centroid_height, i

def move_d_to(target_distance=0.5,target_link='Floor_Object0'):
    ###Face towards Targetlink and get target distance close
    try:
        obj_tar,_ =  listener.lookupTransform('map',target_link,rospy.Time(0))
    except(tf.LookupException):
        print ('no  tf found')
        return False
    
    robot, _ =  listener.lookupTransform('map','base_link',rospy.Time(0))
    pose, quat =  listener.lookupTransform('base_link',target_link,rospy.Time(0))
    euler=euler_from_quaternion(quat)

    D=np.asarray(obj_tar)-np.asarray(robot)
    d=D/np.linalg.norm(D)
    if target_distance==-1:
        new_pose=np.asarray(robot)
    else:
        new_pose=(np.asarray(obj_tar)-target_distance*d)
    
    broadcaster.sendTransform(new_pose,(0,0,0,1), rospy.Time.now(), 'D_from_object','map')
    #for_grasp=new_pose
    #for_grasp[0]+=.15* np.sin(  np.arctan2(pose[1],pose[0])-euler[2])
    #for_grasp[1]+=.15* np.cos( np.arctan2(pose[1],pose[0])-euler[2])
    
    #broadcaster.sendTransform(for_grasp,(0,0,0,1), rospy.Time.now(), 'D_from_object_grasp_floor','map')
    wb_v= whole_body.get_current_joint_values()

    arm.set_named_target('go')
    arm.go()

    #print('EULER, WBV', euler_from_quaternion(quat), wb_v[2])
    #succ=move_base( for_grasp[0],for_grasp[1],         np.arctan2(pose[1],pose[0])-euler[2])
    succ=move_base( new_pose[0],new_pose[1],         np.arctan2(pose[1],pose[0])  -euler[2]  -0.07   )
    return succ   
        

    ##### Define state INITIAL #####
#Estado inicial de takeshi, neutral
class Initial(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['succ','failed','tries'],input_keys=['global_counter'])
        self.tries=0

        
    def execute(self,userdata):

        #   publish_scene()
        rospy.loginfo('STATE : robot neutral pose')
        print('Try',self.tries,'of 5 attepmpts') 
        self.tries+=1
        
        
        req=classify_client.request_class()
        images=[]
        image=np.zeros((50,50,3))
        images.append(image)
        images.append(image)
        for image in images:
            img_msg=bridge.cv2_to_imgmsg(image)
            req.in_.image_msgs.append(img_msg)


            resp1 = classify_client(req)
        clear_octo_client()
        close_gripper()
        scene.remove_world_object()
        #Takeshi neutral
        arm.set_named_target('go')
        arm.go()
        head.set_named_target('neutral')
        succ = head.go()  
        #move_base(1.15,0.35,0.5*np.pi)         ###FORDRAWERS
        if self.tries>=2:
            move_base(1.15,0.35,0.5*np.pi)         
            return 'tries'

        
        if succ:
            return 'succ'
        else:
            return 'failed'

class Scan_floor(smach.State):### check next state's goal 
    def __init__(self):
        smach.State.__init__(self,outcomes=['succ','clear','failed','tries','above'] ,input_keys=['clear_table1_flag', 'clear_table2_flag'],output_keys=['clear_table1_flag', 'clear_table2_flag'])
        self.tries=0
    def execute(self,userdata):

        
        global class_resp ,closest_centroid_height,closest_centroid_index , target_tf_class
        rospy.loginfo('State : Scan Floor ')
        #move_base(0.5+0.1*self.tries,0.15+0.1*self.tries,(0.4+0.1*self.tries)*np.pi)
        
        
        if userdata.clear_table1_flag:
            print( 'looking at point 2' ) 
            move_base (0.7,0.7, 0.5*np.pi)
            gaze_point(0.14,1,0.1)
        else:
            print( 'looking at point 1' ) 
            gaze_point(1.0,1.0 ,0.1)
        
        cents,xyz, images=plane_seg_square_imgs(lower=50)

        #   if len (images)==0:cents,xyz, images=seg_square_imgs(plt_images=True)

        
        if (len (images)!=0) :
            
            class_resp=classify_images(images)
            closest_centroid_height,closest_centroid_index=static_tf_publish(cents)
            #target_tf_class=class_resp[closest_centroid_index,0]
            target_tf= 'Closest_Floor'
            #print ('class_names',len(class_names))
            #print ('class_resp',class_names[(int)(class_resp[(int)(3*closest_centroid_index)])] ,class_names[(int)(class_resp[(int)(3*closest_centroid_index)])+1]                                       )

            target_tf_class= class_names[(int)(class_resp[(int)(3*closest_centroid_index)])]   ,class_names[(int)(class_resp[(int)(3*closest_centroid_index)])+1]   ,class_names[(int)(class_resp[(int)(3*closest_centroid_index)])+2]   
            print ("closest_centroid_height,closest_centroid_index",closest_centroid_height,closest_centroid_index,target_tf)#,class_names[target_tf_class])
            dimx,dimy,dimz=max(xyz[closest_centroid_index][:,0])-min(xyz[closest_centroid_index][:,0]),max(xyz[closest_centroid_index][:,1])-min(xyz[closest_centroid_index][:,1]),max(xyz[closest_centroid_index][:,2])-min(xyz[closest_centroid_index][:,2])
            print ( 'dimx', dimx)# max(xyz[closest_centroid_index][:,0])-min(xyz[closest_centroid_index][:,0]))
            print ( 'dimy', dimy)# max(xyz[closest_centroid_index][:,1])-min(xyz[closest_centroid_index][:,1]))
            print ( 'dimz', dimz)# max(xyz[closest_centroid_index][:,2])-min(xyz[closest_centroid_index][:,2]))
            #print ('estimated  dimensions', dim_x, 'x' , dim_y,'x', dim_z)
        

        rospy.sleep(0.25)
        try:
            xyz_map_object, _ =  listener.lookupTransform('map',target_tf,rospy.Time(0))
            print('target_tf,most likely class',target_tf, target_tf_class)
            if closest_centroid_height <= 0.01:return 'above'
            else:return 'succ'
        except:
            print ('no target tf ')
            if userdata.clear_table1_flag:
                userdata.clear_table2_flag=True
            else:
                userdata.clear_table1_flag=True




            return 'clear'

        





        """if  userdata.clear_table1_flag and  userdata.clear_table2_flag:
                                    print('clear floor table 1 flag', userdata.clear_table1_flag)
                                    print('clear floor table 2 flag', userdata.clear_table2_flag)
                                    return 'clear'"""
      
        #xyz_map_grasp, _ =  listener.lookupTransform('map',target_tf,rospy.Time(0))
            #serdata.clear_table1_flag=True
        return 'failed'

    
       
        
        
    
     
###################################

##################################################Pre_grasp_floor()      
class Pre_grasp_floor(smach.State):###get a convenient pre grasp pose
    def __init__(self):
        smach.State.__init__(self,outcomes=['succ','failed','tries'])
        self.tries=0
    def execute(self,userdata):
        rospy.loginfo('State : PRE_GRASP_FLOOR')
        publish_scene()
        #target_tf= 'Object_'+str(closest_centroid_index)+'_Floor'
        target_tf= 'Closest_Floor'
        head.set_named_target('neutral')
        head.go()
        self.tries+=1
        if self.tries==11:return 'tries'
        move_d_to(0.66,target_tf)   ############### PLAY WITH THIS NUMBER
        print ('move D')
        arm.set_named_target('neutral')
        open_gripper()
        arm.go()
        if self.tries <=5:move_abs(0,0,-10,0.051)
        arm_grasp_floor = [0.25,-2.4,-0.26,0.701,0.10,0.0]
        arm.set_joint_value_target(arm_grasp_floor)
        succ=arm.go()
        pose,quat=listener.lookupTransform('map','base_link',rospy.Time(0))
        
        

        pose, quat =  listener.lookupTransform('hand_palm_link',target_tf,rospy.Time(0))
        print ('########################ppose, quat after MOVE D' ,pose, euler_from_quaternion(quat))
        #move_abs(0.0,0.051,0, 0.5*np.abs(pose[0]/.1))
        #rospy.sleep(0.5)

        
        if succ:
            return 'succ'
        self.tries+=1
        if self.tries==3:
            self.tries=0 
            return'tries'
        else:
            return 'failed'

##################################################Grasp_floor()      
class Grasp_floor(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['succ','failed','tries'])
        self.tries=0
    def execute(self,userdata):
        global target_tf_class 
        self.tries+=1
        rospy.loginfo('State : GRASP_FLOOR')

        publish_scene()
        target_tf= 'Closest_Floor'
        print(target_tf,'<->',target_tf_class)


        pose, quat =  listener.lookupTransform('hand_palm_link',target_tf,rospy.Time(0))
        #print ('ppose, quat' ,pose, euler_from_quaternion(quat))
        av=        [0.0,-2.5,-0.26,0.901,0.20,0.0]#[0.0,-2.4,-0.26,0.701,0.20,0.0]
        
        cat = find_category(grasp_dict,target_tf_class[0])
        print( 'class',target_tf_class, 'dest', cat)
        arm.set_joint_value_target(av)
        arm.go(av)
        
        while pose[2] >= 0.091:

            #print ('pose',pose)


            pose, quat =  listener.lookupTransform('hand_palm_link',target_tf,rospy.Time(0))
            
            if pose [1] >- 0.2 and pose[1]<0.02:
                print ('getting close')
                move_abs(0.05,0,0,0.1)
            if pose[1] >= 0.02:
                print ('drift correct   -')
                move_abs(0.0,-0.05,-10, 0.10)   #GRADOS! WTF , DOCKER SEEMS TO WORK THAT WAY
            if pose[1] <= -0.02:
                print ('drift correct   +')
                move_abs(0.0, 0.05,10, 0.10) #GRADOS! WTF , 
        rospy.sleep(0.1)    
            

       
        close_gripper()
        #rospy.sleep(0.25) ### CHECK GRASP
        #save_hand()
        img_before = save_hand(0)
        av[0]+=0.1
        arm.set_joint_value_target(av)
        succ=arm.go(av)
        pose, quat =  listener.lookupTransform('map','hand_palm_link',rospy.Time(0))
        move_abs(-0.1,0.0,0.0,.2)
        move_abs(-0.2,0.0,0.0,.2)
        if pose[1]>=1.6:move_abs(-0.2,0.0,0.0,.5)
            


        move_abs(-0.1,0.0,0.0,.2)

        publish_scene()
        #succ= primitive_grasp_detector()
        #save_hand(1)
        #img_after = save_hand(1)                            ################################################### New !!!!!! #####################
        #grasp_state = grasp_detector(img_after,img_before)  ##### here image grasp detector
        #img = save_hand(self.tries)
        succ= primitive_grasp_detector()
        
        broadcaster.sendTransform(pose,quat,rospy.Time.now(),target_tf,'grasped'  )
        
        publish_scene()
        arm.set_named_target('go')
        arm.go()

        if succ:
            return 'succ'
        
        
               
        if self.tries==10:
            self.tries=0 
            return'tries'
        
        else:
            return 'failed'


########################### Grasping by angle FLOOR ###########################
class Pre_grasp_floor_above(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['succ','failed','tries'])
        self.tries=0
    def execute(self,userdata):
        rospy.loginfo('State : PRE_GRASP_FLOOR_ABOVE')
        publish_scene()

        target_tf= 'Closest_Floor'

        head.set_named_target('neutral')
        head.go()
        self.tries+=1
        if self.tries==11:return 'tries'
        move_d_to(0.66,target_tf)   ############### PLAY WITH THIS NUMBER
        print ('move D')
        arm.set_named_target('neutral')
        open_gripper()
        arm.go()
        if self.tries <=5:move_abs(0,0,-10,0.051)
        arm_grasp_from_above=[0.19, -2.01, 0.0, -0.99, -0.17, 0.0]
        arm.set_joint_value_target(arm_grasp_from_above)
        succ=arm.go()


        pose, quat =  listener.lookupTransform('hand_palm_link',target_tf,rospy.Time(0))
        print ('########################pose, quat after MOVE D' ,pose, euler_from_quaternion(quat))

        rospy.sleep(0.1) 
        if succ:
            return 'succ'
        self.tries+=1
        if self.tries==3:
            self.tries=0 
            return'tries'
        else:
            return 'failed'


class Grasp_floor_from_above(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['succ','failed','tries'])
        self.tries=0
    def execute(self,userdata):
        global target_tf_class 
        self.tries+=1
        rospy.loginfo('State : GRASP_FLOOR_FROM_ABOVE')

        publish_scene()
        target_tf= 'Closest_Floor'
        print(target_tf,'<->',target_tf_class)

        pose, quat =  listener.lookupTransform('hand_palm_link',target_tf,rospy.Time(0))
        rospy.sleep(1)
        arm_grasp_from_above=[0.19, -2.01, 0.0, -0.99, -0.17, 0.0]


        cat = find_category(grasp_dict,target_tf_class[0])
        print( 'class',target_tf_class, 'dest', cat)

        print('Pose_from_above\n')
        #arm.set_joint_value_target(arm_grasp_from_above)
        arm.go(arm_grasp_from_above)
        rospy.sleep(0.1)

        open_gripper()
        rospy.sleep(0.1)

        ref = save_hand(0)
        ref = cv2.cvtColor(ref, cv2.COLOR_BGR2RGB)
        ref = ref[50:400, 0:390]
        print 'Reference shot'

        pose, quat =  listener.lookupTransform('hand_palm_link',target_tf,rospy.Time(0))
        print(pose)


        while abs(pose[0]) >= 0.05:

            print ('pose',pose)
            pose, quat =  listener.lookupTransform('hand_palm_link',target_tf,rospy.Time(0))
            if pose [1] >- 0.2 and pose[1]<0.02:
                print ('getting close Y')
                move_abs(0.031,0.00,0,0.051)
            if pose[1] >= 0.02:
                print ('drift correct   -')
                move_abs(0.0,-0.05,-10, 0.0510)   #GRADOS! WTF , DOCKER SEEMS TO WORK THAT WAY
            if pose[1] <= -0.02:
                print ('drift correct   +')
                move_abs(0.0, 0.05,10, 0.0510) #GRADOS! WTF , 
        rospy.sleep(0.1)
        print('Lined up over Y\n')

        rospy.sleep(0.1)

        publish_scene()

        pose, quat =  listener.lookupTransform('hand_palm_link',target_tf,rospy.Time(0))

        print 'Correcting X'
        wb = whole_body.get_current_joint_values()
        wb[0] = wb[0] + pose[0] + 0.1
        whole_body.go(wb)
        rospy.sleep(0.1)

        pose, quat =  listener.lookupTransform('hand_palm_link',target_tf,rospy.Time(0))

        print('Lined up over X\n'), pose

        wb = whole_body.get_current_joint_values()
        wb[1] = wb[1] + pose[1] + 0.05
        whole_body.go(wb)
        rospy.sleep(0.1)

        pose, quat =  listener.lookupTransform('hand_palm_link',target_tf,rospy.Time(0))

        print('Realined over Y\n'), pose

        img = save_hand(0)
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img = img[50:400, 0:390]
        print 'Angle shot'
        ang = grasp_angle(ref,img)
        
        print('Aligning Angle 1')
        print ang

        wb = whole_body.get_current_joint_values()
        wb[7] = ang + 1.57
        whole_body.go(wb)
        rospy.sleep(0.1)

        pose, quat =  listener.lookupTransform(target_tf, 'hand_palm_link',rospy.Time(0))
        print (pose)
        
        wb=whole_body.get_current_joint_values()
        wb[3] = wb[3] - 0.157
        whole_body.go(wb)
        rospy.sleep(0.1)

        while abs(pose[1]) <= -0.05:

            print ('pose',pose)
            pose, quat =  listener.lookupTransform('hand_palm_link',target_tf,rospy.Time(0))
            if pose [0] >- 0.2 and pose[0]<0.02:
                print ('getting close Y')
                move_abs(0.031,0.00,0,0.051)
            if pose[0] >= 0.02:
                print ('drift correct   -')
                move_abs(0.0,-0.05,-10, 0.0510)   #GRADOS! WTF , DOCKER SEEMS TO WORK THAT WAY
            if pose[0] <= -0.02:
                print ('drift correct   +')
                move_abs(0.0, 0.05,10, 0.0510) #GRADOS! WTF , 
        rospy.sleep(0.1)


        img2 = save_hand(0)
        img2 = cv2.cvtColor(img2, cv2.COLOR_BGR2RGB)
        img2 = img2[50:400, 0:390]

        ang = grasp_angle(ref,img2)

        
        print('Aligning Angle 2')
        print ang

        pose, quat =  listener.lookupTransform('hand_palm_link',target_tf,rospy.Time(0))
        #rospy.sleep(0.1)


        wb=whole_body.get_current_joint_values()
        wb[7] = wb[7] + 0.52
        whole_body.go(wb)
        rospy.sleep(0.1)

        close_gripper()
        rospy.sleep(0.1)

        arm.set_named_target('go')
        arm.go()
        head.set_named_target('neutral')
        head.go()
        close_gripper()
        rospy.sleep(0.1)

        gd2 = save_hand(0)
        gd2 = cv2.cvtColor(gd2, cv2.COLOR_BGR2RGB)
        gd2 = gd2[50:400, 0:390]

        hp_Grasp_Det = super_primitive_grasp_detector(gd2)
        
        broadcaster.sendTransform(pose,quat,rospy.Time.now(),target_tf,'grasped'  )
        a = gripper.get_current_joint_values()
        if np.linalg.norm(a - np.asarray(grasped))  >  (np.linalg.norm(a - np.asarray(ungrasped))):
            print ('Mechanichal Detector: grasp seems to have failed')
            if (hp_Grasp_Det == True):
                print ('Visual grasp detector points towards succesfull')
                close_gripper()
                rospy.sleep(0.1)
                return 'succ'
            else:
                return 'failed'

        else:
            print('super primitive grasp detector points towards succesfull ')
            return'succ'




    

##### Define state SCAN_TABLE #####
class Goto_table(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['succ','failed','tries','end'],input_keys=['clear_table1_flag', 'clear_table2_flag'],output_keys=['table_target'])
        self.tries=0
    def execute(self,userdata):
        self.tries+=1
        
        global cents, rot, trans , target_tf
        
        

                
        

        #goal_x = 0.25 + 0.051*self.tries
        if userdata.clear_table1_flag:
            goal_x,goal_y,goal_yaw=1.00,1.2,1.57
            userdata.table_target=1
        if userdata.clear_table2_flag:
            goal_x,goal_y,goal_yaw=0.13,1.2,1.57
            userdata.table_target=2
        

        goal_xyz=np.asarray((goal_x,goal_y,goal_yaw))
        #move_base(goal_x+.25*self.tries, goal_y , goal_yaw)      
        succ=move_base(goal_x, goal_y , goal_yaw)      
        xyz=whole_body.get_current_joint_values()[:3]

        print ('goal is ',goal_xyz,'current',xyz)
        print ('Distance is ' ,np.linalg.norm(xyz-goal_xyz))
        if (np.linalg.norm(xyz-goal_xyz)  < .3):
            rospy.loginfo("Navigation Close Enough.")
            return 'succ'
        if succ:
            return 'succ'
        
        if self.tries==3:
            self.tries=0 
            return'tries'

        else:
            return'failed'

class Scan_table(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['succ','failed','tries'],input_keys=['table_target'],output_keys=['table_target'])
        self.tries=0
    def execute(self,userdata):
        
        global cents, rot, trans , target_tf, target_tf_class
        self.tries+=1

        #move_base(1.54, 1.1,0.5*np.pi  )
        if userdata.table_target==1:
            gaze_point(1.7-self.tries*0.2,1.7,.41)
            target_tf= 'Closest_Table_1'
        if userdata.table_target==2:
            gaze_point(0.3-self.tries*0.1,1.7,.41)
            target_tf= 'Closest_Table_2'
        cents,xyz, images=seg_square_imgs()
        if len (images)==0:cents,xyz, images=seg_square_imgs(lower=20)
        
        closest_centroid_height,closest_centroid_index = static_tf_publish(cents,table=True)

        req=classify_client.request_class()

        class_resp= classify_images(images)

        
        
        target_tf_class= class_names[(int)(class_resp[(int)(3*closest_centroid_index)])]   ,class_names[(int)(class_resp[(int)(3*closest_centroid_index)])+1]   ,class_names[(int)(class_resp[(int)(3*closest_centroid_index)])+2]   
        print ('target_tf',target_tf,target_tf_class)
        
        
        
        try:
            xyz_map_object, _ =  listener.lookupTransform('map',target_tf,rospy.Time(0))
            self.tries=0
            return 'succ'
            #xyz_map_grasp, _ =  listener.lookupTransform('map',target_tf,rospy.Time(0))
        except:
            if self.tries > 2:
                self.tries=0
                
                userdata.table_target=2
                return'tries'
            print ('no target tf will be found next state')
            return 'failed'

            
            

        else:
            return'failed'


        

        """trans, rot = listener.lookupTransform('/map', '/head_rgbd_sensor_gazebo_frame', rospy.Time(0))
                                euler = tf.transformations.euler_from_quaternion(rot)        
                                cents = segment_table()
                                if len (cents)==0:
                                    cents = segment_table2(2)
                                    
                                    
                                                                    
                                if len (cents)==0:
                                    arm.set_named_target('go')
                                    arm.go()
                                    head.set_named_target('neutral')
                                    head.go()
                                    return 'failed'
                                else:
                                    print ('tfs published (not static)')
                                    #static_tf_publish(cents)
                                    self.tries=0 
                                    return 'succ'"""
##################################################Pre_grasp_table()      
class Pre_grasp_table(smach.State):###get a convenient pre grasp pose
    def __init__(self):
        smach.State.__init__(self,outcomes=['succ','failed','tries'],input_keys=['table_target'])
        self.tries=0
    def execute(self,userdata):
        rospy.loginfo('State : PRE_GRASP_TABLE')
        print ("self.tries",self.tries)
        
        
        table_line_up(1.0,target_tf)
        head.set_named_target('neutral')
        head.go()
        av= arm_grasp_table
        #arm.set_named_target('neutral')
        succ=arm.go(av)
        
        if succ:
            return 'succ'
        else:
            return 'failed'

        
##################################################
class Grasp_table(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['succ','failed','tries'],input_keys=['table_target'])
        self.tries=0
    def execute(self,userdata):
        rospy.loginfo('State : GRASP_TABLE')
        scene.remove_world_object()
        open_gripper()
        av= arm.get_current_joint_values()
        
        print ('grasping in table'+ str(userdata.table_target))
        print ('target tf ',target_tf_class)
        if userdata.table_target==1:av[0]=0.16
        if userdata.table_target==2:av[0]=0.372
        succ = arm.go(av)
        pose, quat =  listener.lookupTransform('hand_palm_link',target_tf,rospy.Time(0))
        open_gripper()
        
        
        while pose[2] >= 0.091:
            
            if pose[1] > 0.025:
                print ('drift correct   -')
                move_abs(0.00,-0.01,-2.5, 0.051)   #GRADOS! WTF , DOCKER SEEMS TO WORK THAT WAY
            if pose[1] < -0.025:
                print ('drift correct   +')
                move_abs(0.00, 0.01,5, 0.051) #GRADOS! WTF , 
            
            else:
            
            
                print ('getting close')
                move_abs(0.051,0,0,0.051)
            pose, quat =  listener.lookupTransform('hand_palm_link',target_tf,rospy.Time(0))
            #rospy.sleep(0.1)    
            
        
        rospy.sleep(0.15)
        

        succ= close_gripper()

        rospy.sleep(0.15)
        pose,quat= listener.lookupTransform('map','base_link',rospy.Time(0))
        #checkgrasp
        #av= arm.get_current_joint_values()
        #av[0]+=0.1
        #arm.set_joint_value_target(av)
        #succ=arm.go(av)
        move_abs(-0.1,0.0,0.0,0.2)
        move_abs(-0.2,0.0,0.0,0.6)
        move_abs(-0.1,0.0,0.0,0.2)

        
        broadcaster.sendTransform(pose,quat,rospy.Time.now(),target_tf,'grasped'  )
        succ= primitive_grasp_detector()
        if succ:
            arm.set_named_target('go')
            arm.go()
            return 'succ'

       

        
        
        
        
        
        
        else:
            arm.set_named_target('go')
            arm.go()
            return 'failed'

############################################               
class Goto_person(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['succ','failed','tries'],input_keys=['counter_in'],output_keys=['counter_out'])
        self.tries=0
    def execute(self,userdata):
        self.tries+=1
        if self.tries==4:
            self.tries=0 
            return'tries'
        goal_x = 0.6
        goal_y = 3.3
        goal_yaw = 2*1.57
        goal_xyz=np.asarray((goal_x,goal_y,goal_yaw))

        # fill ROS message
        pose = PoseStamped()
        pose.header.stamp = rospy.Time(0)
        pose.header.frame_id = "map"
        pose.pose.position = Point(goal_x, goal_y, 0)
        quat = tf.transformations.quaternion_from_euler(0, 0, goal_yaw)
        pose.pose.orientation = Quaternion(*quat)

        goal = MoveBaseGoal()
        goal.target_pose = pose

        # send message to the action server
        navclient.send_goal(goal)

        # wait for the action server to complete the order
        navclient.wait_for_result(timeout=rospy.Duration(10))

        # print result of navigation
        action_state = navclient.get_state()
        print(action_state)
        xyz=whole_body.get_current_joint_values()[:3]
        rospy.loginfo (str(whole_body.get_current_joint_values()[:2]))
        print ('goal is ',goal_xyz,'current',xyz)
        print ('Distance is ' ,np.linalg.norm(xyz-goal_xyz))
        if (np.linalg.norm(xyz-goal_xyz)  < .3):
            rospy.loginfo("Navigation Close Enough.")
            return 'succ'

        if action_state == GoalStatus.SUCCEEDED:
            rospy.loginfo("Navigation Succeeded.")
            return 'succ'
        else:
            print(action_state)
            return'failed'

class Give_object(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['succ','failed','tries'],input_keys=['counter_in'],output_keys=['counter_out'])
        self.tries=0
    def execute(self,userdata):
        self.tries+=1
        

        ###### MOVEIT IT IS A BIT COMPLEX FOR IN CODE COMMENTS; PLEASE CONTACT 
        ######## we are seting all the joints in the "whole body " command group to a known value
        #######  conveniently named give object
        #######   before using  clearing the octomap service might be needed



        clear_octo_client()
        wb_give_object=[0.57, 3.26, 3.10, 0.057,-0.822,-0.0386, -0.724, 0.0, 0.0]
        whole_body.set_joint_value_target(wb_give_object)
        whole_body.go()

        print ('yey')
        return 'succ'






        if self.tries==3:
            self.tries=0 
            return'tries'





        

###################################### GO AND DELIVER ######################################
##### Define state GO_BOX #####
#Se mueve hacia la caja baja el brazo y se acerca mas 
###################################### GO AND DELIVER ######################################
##### Define state GO_BOX #####
#Se mueve hacia la caja baja el brazo y se acerca mas
class Pre_deliver(smach.State):

    def __init__(self):
        smach.State.__init__(self,outcomes=['succ','failed','tries'],input_keys=['delivery_destination'],output_keys=['delivery_destination_out'])

# ///////////////////////////////////////////////////////////////////////////////////////////
#/////////////////////////////      VARIABLES         ///////////////////////////////////////
# ///////////////////////////////////////////////////////////////////////////////////////////
# THESE VARIABLES WERE ADDED TO STORE THE COUNT OF OBJECTS ON THE TRAY <----------------------
        self.tries=0
        self.tA = 0
        self.trayA = np.array([])
        self.tB = 0
        self.tBinB = 0
        self.BinB = np.array([])

    def execute(self,userdata):

        publish_scene()

        whole_body.set_start_state_to_current_state()
        cat = find_category(grasp_dict,target_tf_class[0])
        print ("Class: ", target_tf_class[0])
        print ("Category: ", cat)

    # ///////////////////////////////////////////////////////////////////////////////////////////
    #/////////////////////////////      TRAY A/TRAY B         ///////////////////////////////////
    #/////////////////////////////      MULTIPLE LOCATIONS    ///////////////////////////////////
    # ///////////////////////////////////////////////////////////////////////////////////////////
    #   B     A
    # [0,1] [0,1]
    # [2,3] [2,3]
    # [4,5] [4,5]
        if cat ==1 :
            print(self.tA)
            wb = wb_trayA[self.tA]
            self.tA = self.tA+1
            print ("Deliver to ",'Tray_A')
            #self.trayA = np.append(self.trayA, target_tf_class)
            #print("Tray A has: ", self.trayA)
        elif cat ==2 :
            print(self.tB)
            wb = wb_trayB[self.tB]
            self.tB = self.tB+1
            print ("Deliver to ",'Tray_B')
            #self.trayB = np.append(self.trayB, target_tf_class)
            #print("Tray B has: ", self.trayB)
    # ///////////////////////////////////////////////////////////////////////////////////////////
    #/////////////////////////////      CONTAINERS AND BINS         /////////////////////////////
    # ///////////////////////////////////////////////////////////////////////////////////////////
    #THE CONTAINERS AND BINS WERE ADDED <--------------------------------------------------------       
        elif cat ==3 :
            wb=wb_place_contA
            print ("Deliver to ",'Container_A')
	elif cat ==4 :
	    wb=wb_place_drawer_high
	    print ("Deliver to ", 'Drawer Top' )
	elif cat ==5 :
	    wb=wb_place_dB
	    print ("Deliver to ", 'Drawer Bottom' ) 
        elif cat ==6 :
            wb=wb_place_Box1
            print ("Deliver to ",'Bin_A')
        elif cat ==7 :
            wb=wb_place_contB
            print ("Deliver to ",'Container_B')
        else:
            print(self.tBinB)
            wb=wb_place_Box2
            
            print ("Deliver to ",'Bin_B')
            
            

            self.tries+=1
            print(self.tries,'out of 5')
            arm.set_named_target('go')
            arm.go()
            head.set_named_target('neutral')
            head.go()

    # THE wb[2] WAS ADDED TO CORRECT THE ANGLE OF DELIVERY <-------------------------------------
        succ=move_base(wb[1],wb[0], wb[2])                          
        

    # ///////////////////////////////////////////////////////////////////////////////////////////
    # ///////////////////////////////////////////////////////////////////////////////////////////
    # ///////////////////////////////////////////////////////////////////////////////////////////
        """if wb==wb_place_Box2:                                #<------------------------------
            succ=move_base(wb[1],wb[0], wb[2])
        else:
            #move_d_to(0.6,userdata.delivery_destination)
                succ=move_base(wb[1],wb[0], -0.5*np.pi)
            #succ=move_base(wb[1],wb[0], -90)"""


        wb=wb_place_tray_A
        arm.set_start_state_to_current_state()
        av=arm.get_current_joint_values()
        av[0]+=0.35
        av[1:]=wb[4:]
        succ=arm.go(av)
        print ('*****LIFT*****')

            
        open_gripper()
        arm.set_start_state_to_current_state()
        av=arm.get_current_joint_values()
        av[0]+=-0.05
        arm.go(av)
        arm.set_start_state_to_current_state()
        av=arm.get_current_joint_values()
        av[0]+=0.05
        arm.go(av)
        arm.set_start_state_to_current_state()
        av=arm.get_current_joint_values()
        av[0]+=-0.05
        arm.go(av)
        arm.set_start_state_to_current_state()
        av=arm.get_current_joint_values()
        av[0]+=0.08
        arm.go(av)
        arm.set_named_target('go')
        arm.go()

        if succ:
            self.tries=0
            return 'succ'

        if self.tries==5:
            self.tries=0
            return 'tries'
        if succ != True:return 'failed'

        else: return ('failed')


             
class Pre_drawer(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['succ','failed','tries'])
        self.tries=0
    def execute(self,userdata):
        
        self.tries+=1
        publish_scene()

        scene.remove_world_object()

        
        whole_body.set_start_state_to_current_state()
        #### Preknow grasping position called grasp table
        whole_body.get_current_joint_values()
        whole_body.set_start_state_to_current_state()
        
        if self.tries==1:succ=whole_body.go(wb_pre_drawers)
        if self.tries==2:
            wbv=wb_pre_drawers
            wbv[3]+=0.35
            succ=whole_body.go(wbv)
        if self.tries==3:
            wbv=wb_pre_drawers
            wbv[0]+=0.35
            succ=whole_body.go(wbv)


        open_gripper()
        
        if self.tries==4:
            self.tries=0 
            return'tries'
        if succ:
            
            return 'succ'
        else:
            return 'failed'
class Grasp_drawer(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['succ','failed','tries'])
        self.tries=0
    def execute(self,userdata):
        #publish_scene()
        self.tries+=1
        global target_drawer
        scene.remove_world_object()
        if self.tries==1: target_drawer='Drawer_low'
        if self.tries==2: target_drawer='Drawer_high'
        if self.tries==3: target_drawer='Drawer_left'
        if self.tries==4: return 'tries'
        ## BY COMPARING RELATIVE POSITION BETWEEN HAND AND DRAWER A TWO STAGE SMALL ADJUSTMENTS MOVEMENT IS PROPOSED
        
        trans_hand,rot_hand= listener.lookupTransform( target_drawer,'hand_palm_link',rospy.Time(0))
        wb=whole_body.get_current_joint_values()
        wb[0]+=-trans_hand[1]+.15
        wb[1]+=-trans_hand[0]
        wb[3]+=-trans_hand[2]
        whole_body.go(wb)
        trans_hand,rot_hand= listener.lookupTransform( target_drawer,'hand_palm_link',rospy.Time(0))
        wb=whole_body.get_current_joint_values()
        wb[0]+=-trans_hand[1]+.08
        wb[1]+=-trans_hand[0]
        wb[3]+=-trans_hand[2]
        succ=whole_body.go(wb)
        
        close_gripper()
        if self.tries==5:
            self.tries=0 
            return'tries'
        if succ:
            return 'succ'
        else:
            return 'failed'
        
        
class Post_drawer(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['succ','failed','tries'])
        self.tries=0
    def execute(self,userdata):
        #publish_scene()
        self.tries+=1
       


        trans_hand,rot_hand= listener.lookupTransform( target_drawer,'hand_palm_link',rospy.Time(0))


        
        wb= whole_body.get_current_joint_values()
        wb[0]+= +0.29
        succ=whole_body.go(wb)
        open_gripper()
        wb= whole_body.get_current_joint_values()
        wb[0]+= +0.2
        #wb[1]= .4
        succ=whole_body.go(wb)


        open_gripper()
        rospy.sleep(0.5)
        

        return 'succ'
        
        
        if self.tries==4:
            self.tries=0 
            return'tries'
        if succ:
            return 'succ'
        else:
            return 'failed'
        
class Deliver(smach.State):

    def __init__(self):
        smach.State.__init__(self,outcomes=['succ','failed','tries'],input_keys=['delivery_destination'],output_keys=['delivery_destination_out'])
        self.tries=0

    def execute(self,userdata):
        
        av=arm.get_current_joint_values()
        av[0]+=-0.1
        arm.go(av)
        av=arm.get_current_joint_values()
        av[0]+=0.1
        arm.go(av)
        
        if succ:
            wb = whole_body.get_current_joint_values()
            wb[0] += 0.45
            whole_body.set_joint_value_target(wb)
            succ=whole_body.go()
            move_hand(0)
            arm.set_named_target('go')
            arm.go()
            head.set_named_target('neutral')
            head.go()
            if userdata.delivery_destination == 'Tray_A':
                userdata.delivery_destination_out="Tray_B"
            if userdata.delivery_destination == 'Tray_B':
                userdata.delivery_destination_out="Box1"
            return 'succ'
        else:
            return 'failed'



#Initialize global variables and node
def init(node_name):
    global listener, broadcaster, tfBuffer, tf_static_broadcaster, scene, rgbd  , head,whole_body,arm,gripper  ,goal,navclient,clear_octo_client , classify_client , class_names , bridge , base_vel_pub , target_tf
    rospy.init_node(node_name)
    head = moveit_commander.MoveGroupCommander('head')
    gripper =  moveit_commander.MoveGroupCommander('gripper')
    whole_body=moveit_commander.MoveGroupCommander('whole_body_light')
    arm =  moveit_commander.MoveGroupCommander('arm')
    listener = tf.TransformListener()
    broadcaster = tf.TransformBroadcaster()
    tfBuffer = tf2_ros.Buffer()
    tf_static_broadcaster = tf2_ros.StaticTransformBroadcaster()
    whole_body.set_workspace([-6.0, -6.0, 6.0, 6.0]) 
    scene = moveit_commander.PlanningSceneInterface()
    rgbd = RGBD()
    goal = MoveBaseGoal()
#    navclient = actionlib.SimpleActionClient('/move_base/move', MoveBaseAction)  LOCAL GAZEBO
    navclient = actionlib.SimpleActionClient('/move_base', MoveBaseAction)          #DOCKER

    clear_octo_client = rospy.ServiceProxy('/clear_octomap', Empty)
    bridge = CvBridge()
    #class_names=['002masterchefcan', '003crackerbox', '004sugarbox', '005tomatosoupcan', '006mustardbottle', '007tunafishcan', '008puddingbox', '009gelatinbox', '010pottedmeatcan', '011banana', '012strawberry', '013apple', '014lemon', '015peach', '016pear', '017orange', '018plum', '019pitcherbase', '021bleachcleanser', '022windexbottle', '024bowl', '025mug', '027skillet', '028skilletlid', '029plate', '030fork', '031spoon', '032knife', '033spatula', '035powerdrill', '036woodblock', '037scissors', '038padlock', '040largemarker', '042adjustablewrench', '043phillipsscrewdriver', '044flatscrewdriver', '048hammer', '050mediumclamp', '051largeclamp', '052extralargeclamp', '053minisoccerball', '054softball', '055baseball', '056tennisball', '057racquetball', '058golfball', '059chain', '061foambrick', '062dice', '063-amarbles', '063-bmarbles', '065-acups', '065-bcups', '065-ccups', '065-dcups', '065-ecups', '065-fcups', '065-gcups', '065-hcups', '065-icups', '065-jcups', '070-acoloredwoodblocks', '070-bcoloredwoodblocks', '071nineholepegtest', '072-atoyairplane', '073-alegoduplo', '073-blegoduplo', '073-clegoduplo', '073-dlegoduplo', '073-elegoduplo', '073-flegoduplo', '073-glegoduplo']
    class_names=['002masterchefcan', '003crackerbox', '004sugarbox', '005tomatosoupcan', '006mustardbottle', '007tunafishcan', '008puddingbox', '009gelatinbox', '010pottedmeatcan', '011banana', '012strawberry', '013apple', '014lemon', '015peach', '016pear', '017orange', '018plum', '019pitcherbase', '021bleachcleanser', '022windexbottle', '024bowl', '025mug', '026sponge', '027skillet', '028skilletlid', '029plate', '030fork', '031spoon', '032knife', '033spatula', '035powerdrill', '036woodblock', '037scissors', '038padlock', '040largemarker', '042adjustablewrench', '043phillipsscrewdriver', '044flatscrewdriver', '048hammer', '050mediumclamp', '051largeclamp', '052extralargeclamp', '053minisoccerball', '054softball', '055baseball', '056tennisball', '057racquetball', '058golfball', '058golfball (1)', '059chain', '061foambrick', '062dice', '063-amarbles', '063-bmarbles', '065-acups', '065-bcups', '065-ccups', '065-dcups', '065-ecups', '065-fcups', '065-gcups', '065-hcups', '065-icups', '065-jcups', '070-acoloredwoodblocks', '070-bcoloredwoodblocks', '071nineholepegtest', '072-atoyairplane', '072-btoyairplane', '072-ctoyairplane', '072-dtoyairplane', '072-etoyairplane', '073-alegoduplo', '073-blegoduplo', '073-clegoduplo', '073-dlegoduplo', '073-elegoduplo', '073-flegoduplo', '073-glegoduplo', '077rubikscube']
    classify_client = rospy.ServiceProxy('/classify', Classify)
    base_vel_pub = rospy.Publisher('/hsrb/command_velocity', Twist, queue_size=10)

    #service_client = rospy.ServiceProxy('/segment_2_tf', Trigger)
    #service_client.wait_for_service(timeout=1.0)
   

    
    
  

#Entry point    
if __name__== '__main__':
    print("Takeshi STATE MACHINE...")
    init("takeshi_smach")
    sm = smach.StateMachine(outcomes = ['END'])     #State machine, final state "END"
    sm.userdata.sm_counter = 0
    sm.userdata.clear1 = False
    sm.userdata.clear2 = False
    sm.userdata.delivery_dest = "Tray_A"

    with sm:
        #State machine for grasping on Floor
        
        #smach.StateMachine.add("INITIAL",       Initial(),      transitions = {'failed':'INITIAL',      'succ':'SCAN_FLOOR',    'tries':'SCAN_FLOOR'}) ###FORDRAWERS
        smach.StateMachine.add("INITIAL",       Initial(),      transitions = {'failed':'INITIAL',      'succ':'PRE_DRAWER',    'tries':'SCAN_FLOOR'}) 
        smach.StateMachine.add('PRE_DRAWER',     Pre_drawer(),  transitions = {'failed':'PRE_DRAWER',    'succ': 'GRASP_DRAWER',  'tries':'INITIAL'}) 
        smach.StateMachine.add('GRASP_DRAWER',     Grasp_drawer(),  transitions = {'failed':'GRASP_DRAWER',    'succ': 'POST_DRAWER',  'tries':'INITIAL'}) 
        smach.StateMachine.add('POST_DRAWER',     Post_drawer(),  transitions = {'failed':'POST_DRAWER',    'succ': 'PRE_DRAWER',  'tries':'INITIAL'}) 
        smach.StateMachine.add("SCAN_FLOOR",    Scan_floor(),      transitions = {'failed':'INITIAL',      'succ':'PRE_GRASP_FLOOR','above':'PRE_GRASP_FLOOR_ABOVE'  ,  'tries':'INITIAL' , 'clear':'GOTO_TABLE'}, remapping= {'clear_table1_flag':'clear1','clear_table2_flag':'clear2'}) 
        
        smach.StateMachine.add("PRE_GRASP_FLOOR",   Pre_grasp_floor() ,      transitions = {'failed':'INITIAL',      'succ':'GRASP_FLOOR',    'tries':'INITIAL'}) 
        smach.StateMachine.add("GRASP_FLOOR",   Grasp_floor() ,      transitions = {'failed':'INITIAL',      'succ':'PRE_DELIVER',    'tries':'INITIAL'}) 

        smach.StateMachine.add("PRE_GRASP_FLOOR_ABOVE",   Pre_grasp_floor_above() ,      transitions = {'failed':'INITIAL',      'succ':'GRASP_FLOOR_ABOVE',    'tries':'INITIAL'}) 
        smach.StateMachine.add("GRASP_FLOOR_ABOVE",   Grasp_floor_from_above() ,      transitions = {'failed':'INITIAL',      'succ':'PRE_DELIVER',    'tries':'INITIAL'}) 

        
        smach.StateMachine.add("GOTO_TABLE",    Goto_table(),   transitions = {'failed':'GOTO_TABLE',   'succ':'SCAN_TABLE',     'tries':'GOTO_TABLE', 'end':'INITIAL'},remapping={'clear_table1_flag':'clear1','clear_table2_flag':'clear2','table_target':'sm_counter'})
        smach.StateMachine.add("SCAN_TABLE",    Scan_table(),   transitions = {'failed':'SCAN_TABLE',   'succ':'PRE_GRASP_TABLE',     'tries':'INITIAL'},remapping={'table_target':'sm_counter'})
        smach.StateMachine.add("PRE_GRASP_TABLE",   Pre_grasp_table() ,      transitions = {'failed':'SCAN_TABLE',      'succ':'GRASP_TABLE',    'tries':'INITIAL'},remapping={'table_target':'sm_counter'})
        smach.StateMachine.add("GRASP_TABLE",   Grasp_table() ,      transitions = {'failed':'SCAN_TABLE',      'succ':'PRE_DELIVER',    'tries':'INITIAL'},remapping={'table_target':'sm_counter'})
        
        
        
        smach.StateMachine.add('PRE_DELIVER',   Pre_deliver(),  transitions = {'failed':'PRE_DELIVER',   'succ': 'INITIAL',       'tries':'INITIAL'},remapping={'delivery_destination':'delivery_dest','delivery_destination_out':'delivery_dest'})
        smach.StateMachine.add('DELIVER',       Deliver(),      transitions = {'failed':'DELIVER',       'succ': 'SCAN_FLOOR',    'tries':'PRE_DELIVER'},remapping={'delivery_destination':'delivery_dest','delivery_destination_out':'delivery_dest'})
       


       ##########################################NOT IN USE ATM#######################################################
        #smach.StateMachine.add("PRE_GRASP_FLOOR_ABOVE",   Pre_grasp_floor_above() ,      transitions = {'failed':'INITIAL',      'succ':'PRE_DELIVER',    'tries':'END'}) 
        smach.StateMachine.add("GOTO_PERSON",    Goto_person(),   transitions = {'failed':'GOTO_PERSON',   'succ':'GIVE_OBJECT',     'tries':'INITIAL'},remapping={'counter_in':'sm_counter','counter_out':'sm_counter'})
        smach.StateMachine.add("GIVE_OBJECT",    Give_object(),   transitions = {'failed':'GIVE_OBJECT',   'succ':'END',     'tries':'INITIAL'},remapping={'counter_in':'sm_counter','counter_out':'sm_counter'})
        

        

      

    outcome = sm.execute()
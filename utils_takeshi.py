# -*- coding: utf-8 -*-

from utils import *
import moveit_msgs.msg
#from gazebo_ros import gazebo_interface
import smach
import matplotlib.pyplot as plt
global grasp_dict

##### Publishers #####
scene_pub = rospy.Publisher('planning_scene', moveit_msgs.msg.PlanningScene, queue_size = 5)

##### KNOWN LOCATIONS #####
kl_mess1 = [1.04, 0.3, 90]
kl_tray =  [2.318411366833172, 0.09283744344925589, -90]
kl_box1 =  [-0.04168256822546347, 2.427268271720426, -90]
kl_table1 = [1.04, 1.2, 90]
kl_table2= [0 , 1.2,90] 
kl_drawers=  [0.06, 0.5, -90]


##### GRASP 
ungrasped = [-0.00047048998088961014,
 -0.03874743486886725,
 -0.04825256513113274,
 0.038463464485261056,
 -0.03874743486886725]
grasped = [0.12814103131904275,
 -0.30672794406396453,
 0.21972794406396456,
 0.13252877558892262,
 -0.30672794406396453]

##Whole BOdy
wb_pre_drawers=   [0.40, 0.21,-1.47,0.05,-1.77,0.0,0.20, -1.56, 0.0]

#////////////////////////////////////////////////////////////////////////////////////////////
#/////////////////////////////      LOCATIONS         ///////////////////////////////////////
# ///////////////////////////////////////////////////////////////////////////////////////////

####################################
wb_place_drawer_high=[0.295, 0.153, 17.342117985532223, 0.33991078475300646, -0.8847825066143127, 0.19262435689434065, -1.5324150820525055, -0.009624341234966138, 0.0]

#LOCATION FOR: DRAWERS
wb_place_dT = [0.45, 0.12, -1.5,0.5,-1.4,0.0,-0.60, 0.0, 0.0]
wb_place_dB = [0.45, 0.43, -1.50,0.25,-1.4,0.0,-0.60, 0.0, 0.0]

#LOCATIONS FOR: CONTAINERS, BINS AND CHAIRS <-------------------------------------------------
wb_place_contA=   [0.055, 1.12,-1.75, 0.348, -1.718, -0.026, -0.024, 0.0, 0.0]   #CONTAINER A
wb_place_contB=   [0.058, 1.3,-1.74, 0.349, -1.719, -0.027, -0.022, 0.0, 0.0]   #CONTAINER B 
wb_place_tray_A= [0.043, 1.64, -1.71, 0.22, -1.679, -0.027, -0.016, 0.0, 0.0] 	#TRAY A
wb_place_tray_B= [0.043, 1.94, -1.71, 0.22, -1.679, -0.027, -0.016, 0.0, 0.0] 	#TRAY B
wb_place_Box1= [0.043, 2.4, -1.71, 0.181, -1.679, -0.027, -0.016, 0.0, 0.0] 	#BIN A
wb_place_Box2 = [0.02, 2.58, -1.15, 0.180, -1.683, -0.018, -0.016, 0.0, 0.0] 	#BIN B 
wb_personA = [2.9, 0.85, 3.1, 0.180, -1.683, -0.018, -0.016, 0.0, 0.0]  #CHAIR A <--- 8
wb_personB = [3.9, 0.85, 3.1, 0.180, -1.683, -0.018, -0.016, 0.0, 0.0]  #CHAIR B ---> 9

#LOCATIONS FOR MULTIPLE OBJECTS ON TRAY A: <-------------------------------------------------
wb_trayA = [[0.042, 1.63, -1.60, 0.35, -1.68, -0.035, -0.016,  0.0, 0.0],   # [0,0]
            [0.042, 1.50, -1.60, 0.35, -1.68, -0.035, -0.016, 0.0, 0.0],   # [0,1]
            [0.12, 1.63, -1.60, 0.35, -1.68, -0.035, -0.016, 0.0, 0.0],   # [1,0]
            [0.12, 1.50, -1.60, 0.35, -1.68, -0.035, -0.016, 0.0, 0.0],   # [1,1]
            [0.21, 1.63, -1.60, 0.35, -1.68, -0.035, -0.016, 0.0, 0.0],   # [2,0]
            [0.21, 1.50, -1.60, 0.35, -1.68, -0.035, -0.016, 0.0, 0.0]    # [2,1]
           ]

#LOCATIONS FOR MULTIPLE OBJECTS ON TRAY B: <-------------------------------------------------
wb_trayB = [[0.002, 1.94, -1.60, 0.35, -1.68, -0.035, -0.016,  0.0, 0.0],   # [0,0]
            [0.002, 1.80, -1.60, 0.35, -1.68, -0.035, -0.016, 0.0, 0.0],   # [0,1]
            [0.10, 1.94, -1.60, 0.35, -1.68, -0.035, -0.016, 0.0, 0.0],   # [1,0]
            [0.10, 1.80, -1.60, 0.35, -1.68, -0.035, -0.016, 0.0, 0.0],   # [1,1]
            [0.20, 1.94, -1.60, 0.35, -1.68, -0.035, -0.016, 0.0, 0.0],   # [2,0]
            [0.20, 1.80, -1.60, 0.35, -1.68, -0.035, -0.016, 0.0, 0.0]    # [2,1]
           ]
# ///////////////////////////////////////////////////////////////////////////////////////////
# ///////////////////////////////////////////////////////////////////////////////////////////
# ///////////////////////////////////////////////////////////////////////////////////////////

##### ARM #####
arm_grasp_from_above=[0.19, -2.01, 0.0, -0.99, -0.17, 0.0]

arm_grasp_from_above_table=[0.434, -1.57, -0.02, -1.205, 0.223, 0.0]

arm_grasp_table=[0.41349380130577407,
 -1.671584191489468,
 0.0,
 0.0,
 0.0,
 0.0]
arm_grasp_floor = [-1.5151551103007697e-05,
 -2.4,
 -0.2620865401925543,
 0.7019536624449207,
 0.20120924571306453,
 0.0]
arm_train_pose = [0.033749214744071214,
 -2.1204421063180217,
 -1.3982377978814715,
 -1.7296544561013807,
 2.135675364707808,
 0.0]
arm_ready_to_place = [0.03999320441056991,
 -0.4729690540086997,
 0.19361475012179108,
 -1.5269847787383313,
 -0.009753879176134461,
 0.0]
arm_high_drawer=[0.2539946870715912,
 -1.6765040634258677,
 -0.02776609034055033,
 0.0726899567834991,
 1.5763667194117463,
 0.0]
arm_ready_to_place_drawer=[0.3399152069888128,
 -0.8839069227299579,
 0.19316694610801077,
 -1.5324885970087543,
 -0.009615429265148023,
 0.0]   

##### GRASP 
ungrasped=[-0.00047048998088961014,
 -0.03874743486886725,
 -0.04825256513113274,
 0.038463464485261056,
 -0.03874743486886725]
grasped=[0.12814103131904275,
 -0.30672794406396453,
 0.21972794406396456,
 0.13252877558892262,
 -0.30672794406396453]



########## Base Class for takeshi state machine ##########
class Takeshi_states(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['succ', 'failed', 'tries'])
        self.tries = 1
        self.max_tries = 5
    
    #This function must be overwritten
    def takeshi_run(self):
        print("Takeshi execute")
        success = True        
        return success

    def execute(self, userdata):
        #rospy.loginfo('STATE : INITIAL')
        print("Excecuting Takeshi State: " + self.__class__.__name__)
        print('Try', self.tries, 'of 5 attempts')
        succ = self.takeshi_run()
        if succ: 
            print('success')            
            return 'succ'
        else:
            print('failed')
            self.tries += 1
            if self.tries > self.max_tries:
                return 'tries'
            else:
                return 'failed'


########## Util Functions ##########

def cart2spher(x,y,z):
    ro= np.sqrt(x**2+y**2+z**2)
    th=np.arctan2(y,x)
    phi=np.arctan2((np.sqrt(x**2+y**2)),z)
    return np.asarray((ro,th,phi))


def spher2cart(ro,th,phi):
    x= ro * np.cos(th)* np.sin(phi)
    y= ro * np.sin(th)* np.sin(phi)
    z= ro*  np.cos(th)
    return np.asarray((x,y,z))

def point_2D_3D(points_data, px_y, px_x):
    ##px pixels /2D world  P1 3D world
    P = np.asarray((points_data[px_y, px_x]['x'], points_data[px_y, px_x]['y'], points_data[px_y, px_x]['z']))
    return P

#////////////////////////////////////////////////////////////////////////////////////////////
#/////////////////////////////      DICTIONARY         //////////////////////////////////////
# ///////////////////////////////////////////////////////////////////////////////////////////

grasp_dict={
#////////////KNOWN OBJECTS///////////////////
#-----------FOOD CATEGORY - 1 y 2----------------
# Deliver to: TRAY_A AND TRAY_B
 '002masterchefcan': 1,
 '003crackerbox': 1,
 '004sugarbox': 1,
 '005tomatosoupcan': 1,
 '006mustardbottle': 1,
 '007tunafishcan': 1,
 '008puddingbox': 1,
 '009gelatinbox': 1,
 '010pottedmeatcan': 1,
 '011banana': 2,
 '012strawberry': 2,
 '013apple': 2,
 '014lemon': 2,
 '015peach': 2,
 '016pear': 2,
 '017orange': 2,
 '018plum': 2,
#missing: Pringles chips can
#--------------KITCHEN ITEMS - 3----------------
# Deliver to: CONTAINER_A
 '019pitcherbase': 3,
 '021bleachcleanser': 3,
 '022windexbottle': 3,
 '024bowl': 3,
 '025mug': 3,
 '026sponge': 3,
 '027skillet': 3,
 '028skilletlid': 3, #<------
 '029plate': 3,
 '033spatula': 3,
#missing: Pitcher lid, Wine glass
#--------------TOOL ITEMS - 4--------------------
# Deliver to: DRAWER_TOP AND DRAWER_BOTTOM
 '035powerdrill': 4,
 '038padlock': 4,
 '042adjustablewrench': 4,
 '043phillipsscrewdriver': 4,
 '044flatscrewdriver': 4, 
 '048hammer': 4, 
 '050mediumclamp': 4,
 '051largeclamp': 4,
 '052extralargeclamp': 4,
#missing: Bolt and Nut
#--------------SHAPE ITEMS - 5 ------------------
# Deliver to: DRAWER_LEFT
 '053minisoccerball': 5,
 '054softball': 5, 
 '055baseball': 5,
 '056tennisball': 5,
 '057racquetball': 5,
 '058golfball': 5,
 '059chain': 5,
 '061foambrick': 5, #<-----
 '062dice': 5,
 '063-amarbles': 5,
 '063-bmarbles': 5,
 '065-acups': 5,
 '065-bcups': 5,
 '065-ccups': 5,
 '065-dcups': 5,
 '065-ecups': 5,
 '065-fcups': 5,
 '065-gcups': 5,
 '065-hcups': 5,
 '065-icups': 5,
 '065-jcups': 5,
#missing: Rope, Credit card blank
#--------------TASK ITEMS - 6--------------------
# Deliver to: BIN_A
 '036woodblock': 6,
 '070-acoloredwoodblocks': 6,
 '070-bcoloredwoodblocks': 6,
 '071nineholepegtest': 6,
 '072-atoyairplane': 6,
 '073-alegoduplo': 6,
 '073-blegoduplo': 6,
 '073-clegoduplo': 6,
 '073-dlegoduplo': 6,
 '073-elegoduplo': 6,
 '073-flegoduplo': 6,
 '073-glegoduplo': 6,
 '077-rubikscube': 6,
#missing: Black t-shirt, Timer, Magazine
    
#////////////ORIENTATION-BASED ITEMS - 7//////////
# Deliver to: CONTAINER_B
 '030fork': 7,
 '031spoon': 7,
 '032knife': 7,
 '037scissors': 7,
 '040largemarker':7
#missing: smallmarker
#/////////////UNKNOWN OBJECTS- 8////////////////////
# Deliver to: BIN_B
}

# ///////////////////////////////////////////////////////////////////////////////////////////
# ///////////////////////////////////////////////////////////////////////////////////////////
# //////////////////////////////////////////////////////////////////////////////////////////

def grasp_angle(ref,img):
    angle = 0
    cv2.putText(img,str(img.shape),(100,100),cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)
    gray_image = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    gray_ref = cv2.cvtColor(ref, cv2.COLOR_BGR2GRAY)
    ret,thresh = cv2.threshold(gray_image,100,255,0)
    im2, _,_ = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    ret_ref,thresh_ref = cv2.threshold(gray_ref,100,255,0)
    im2_ref, _, _ = cv2.findContours(thresh_ref,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    img_xor    = cv2.bitwise_xor(im2, im2_ref)
    kernel = np.ones((5, 5), np.uint8)
    image = cv2.erode(img_xor, kernel)
    image = cv2.dilate(image, kernel)
    im_floodfill = image.copy()
    h, w = image.shape[:2]
    mask = np.zeros((h+2, w+2), np.uint8)
    cv2.floodFill(im_floodfill, mask, (0,0), 255);
    im_floodfill_inv = cv2.bitwise_not(im_floodfill)

    image = image | im_floodfill_inv
    img_fin, contours, hierarchy = cv2.findContours(image,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    for c in contours:
        M = cv2.moments(c)
        if M["m00"] != 0:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
        else:
            cX, cY = 0, 0

    for i, contour in enumerate(contours):
            area = cv2.contourArea(contour)

            if area > 1000 and area < 50000 :
                boundRect = cv2.boundingRect(contour)
                rect = cv2.minAreaRect(contour)
                angle = rect[2]
                box = cv2.boxPoints(rect)
                box = np.int0(box)
                cv2.drawContours(img_fin,[box],0,(0,0,255),2)
                img=cv2.rectangle(img_fin,(boundRect[0], boundRect[1]),(boundRect[0]+boundRect[2], boundRect[1]+boundRect[3]), (255,0,0), 2)
                M = cv2.moments(contour)
                # calculate x,y coordinate of center
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                if cX < 300:
                    cv2.circle(img_fin, (cX, cY), 5, (255, 255, 255), -1)
                    cv2.putText(img_fin, "centroid_"+str(i)+','+str(angle) ,    (cX - 25, cY - 25)   ,cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                angle = math.radians(angle)
    return(angle)

def super_primitive_grasp_detector(gd2):

    img = gd2
    gray_image = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    ret,thresh = cv2.threshold(gray_image,100,255,0)
    im2, _,_ = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

    mean_x = ((0.0,0.0,52.38))
    delta_x = 50

    img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    img_b, img_g, img_r = cv2.split(img_hsv)

    up_h = mean_x[0] + delta_x
    lw_h = mean_x[0] - delta_x
    up_s = mean_x[1] + delta_x
    lw_s = mean_x[1] - delta_x
    up_v = mean_x[2] + delta_x
    lw_v = mean_x[2] - delta_x
    up = (up_h,up_s,up_v,0.0)
    lw = (lw_h,lw_s,lw_v,0.0)

    Fil = cv2.inRange(img_hsv, lw, up)
    Fil = cv2.bitwise_not(Fil)

    img_xor    = cv2.bitwise_xor(im2, Fil)

    kernel = np.ones((5, 5), np.uint8)
    image = cv2.erode(img_xor, kernel)

    im_floodfill = image.copy()
    h, w = image.shape[:2]
    mask = np.zeros((h+2, w+2), np.uint8)
    cv2.floodFill(im_floodfill, mask, (0,0), 255);
    im_floodfill_inv = cv2.bitwise_not(im_floodfill)

    image = image | im_floodfill_inv

    img_fin, contours, hierarchy = cv2.findContours(image,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    for c in contours:
      M = cv2.moments(c)
      if M["m00"] != 0:
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
      else:
        cX, cY = 0, 0

    for i, contour in enumerate(contours):
      area = cv2.contourArea(contour)
      if area > 300 and area < 50000 :
        boundRect = cv2.boundingRect(contour)
        rect = cv2.minAreaRect(contour)
        box = cv2.boxPoints(rect)
        box = np.int0(box)
        cv2.drawContours(img_fin,[box],0,(0,0,255),2)
        img=cv2.rectangle(img_fin,(boundRect[0], boundRect[1]),(boundRect[0]+boundRect[2], boundRect[1]+boundRect[3]), (255,0,0), 2)
        img = cv2.dilate(img_xor, kernel)
        img_disp = img
        print('There might be an object in gripper')
        print area
        return True
    else:
        return False
    

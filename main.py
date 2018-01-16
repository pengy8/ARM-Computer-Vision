import AR_Camera
import sys
import threading
import scipy.linalg as linalg
import numpy as np
from ControlParams import *
import rpi_abb_irc5
import time
from oct2py import octave
from oct2py import Oct2Py

CAMERA_ID = 0
VIDEO_MODE = True
MAXHEIGHT = 115 # Dobot Max Z Value

# AR TAGS
Tag_A = [12345678, 176]
Tag_B = [87654321, 176]
Tag_C = [-100, 176] # [ -100, SIZE] returns any tag not equal to Tag_A/Tag_B
REGISTERED_TAGS = [Tag_A, Tag_B, Tag_C]

# Display poses of all objects
# [ [Vector tag to camera] ,  [Rotation of tag] ]
# If not present, [None, None] will be reported
def get_data(camera):
    print "----------------------------"
    print "A Pose:", camera.A_Pose
    print "B Pose:", camera.B_Pose
    print "C Pose:", camera.C_Pose
    print "----------------------------"

def Coordinate_mapping(M):
    A = np.array([[0.24435874021417,-0.652789446147632,-0.00342558335872489,1623.66192687898], [0.366376954836548,0.0264973664299329,-0.874573689966082,-46.5649901205182], [0.846396358437279,0.646827636930246,0.254455439418749,-1070.88068083344]])
    Xd = np.matmul(A, np.concatenate((M,np.array([[1]])),axis =0))   
    return Xd
    
def get_Poa(camera):
    Roc = np.array([[0.4860,-0.1868,-0.8537], [0.0583,   -0.9678,    0.2450], [-0.8720,   -0.1688,   -0.4595]])    
    Poc = np.array([[3579.5], [-826.5], [1416.7]])
     
    S = camera.A_Pose[0]
    print S
    if np.shape(S) !=():
        PoA = np.matmul(Roc, S) + Poc
        PoA = Coordinate_mapping(PoA)
    else:
        PoA = None
    S = camera.B_Pose[0]
    if np.shape(S) !=():
        PoB = np.matmul(Roc, S) + Poc
        PoB = Coordinate_mapping(PoB)
    else:
        PoB = None
    S = camera.C_Pose[0]
    if np.shape(S) !=():
        PoC = np.matmul(Roc, S) + Poc
        PoC = Coordinate_mapping(PoC)
    else:
        PoC = None       

    print "----------------------------"
    print "PoA:", np.transpose(PoA)
    print "PoB:", np.transpose(PoB)
    print "PoC:", np.transpose(PoC)
    print "----------------------------"


# USER INPUT THREAD - Used for break signals
def input_thread(usr_list):
    raw_input()
    usr_list.append(None)


# FUNCTION: Track - Follow an AR tag by hovering the camera above it.
# AR TAGS: DUCKY = 0  DUCKYBOT = 1   OBSTACLE = 2
def track(camera):
    # EVENT LISTENER
    octave.addpath('/home/Shuyang/Downloads/update/Velocity_Control_Constrained_ABB_OpenRAVE')

    egm=rpi_abb_irc5.EGM()

# Initialize Robot Parameters
    
    [ex,ey,ez,n,P,q_nouse,H,ttype,dq_bounds] = octave.feval('robotParams', nout=9)

# Initialize Control Parameters
# initial joint angles
    q = np.zeros((6, 1))
    [R,pos] = octave.feval('fwdkin', q,ttype,H,P,n, nout=2)
    dq = np.zeros((int(n),1))
    
    # velocities
    w_t = np.zeros((3, 1))
    v_t = np.zeros((3, 1))
    
    # create a handle of these parameters for interactive modifications
    obj = ControlParams(ex,ey,ez,n,P,H,ttype,dq_bounds,q,dq,pos,orien,pos_v,ang_v[None, :],w_t,v_t,epsilon,view_port,axes_lim,inc_pos_v,inc_ang_v,0,er,ep,0)

    dt = 0
    counter = 0
       
    req_exit = []
    StateVector = []
    listener = threading.Thread(target=input_thread, args=(req_exit,))
    listener.start()
    tag_index = 0
    Roc = np.array([[0.4860,-0.1868,-0.8537], [0.0583,   -0.9678,    0.2450], [-0.8720,   -0.1688,   -0.4595]])    
    Poc = np.array([[3579.5], [-826.5], [1416.7]])

    # Kalman Filter Parameters
    Phi = np.matrix('1 0 0 1 0 0; 0 1 0 0 1 0; 0 0 1 0 0 1; 0 0 0 1 0 0; 0 0 0 0 1 0; 0 0 0 0 0 1')
    R = np.eye(3,3)*1000 
    Q = np.eye(6)*1000
    H = np.eye(3,6)
    Cov = np.eye(6,6)*1500

    while not req_exit:
        if counter != 0:
        	end = time.time()
        	dt = end-start
            
        start = time.time()
        counter = counter + 1
        
        if counter != 0:               
            J_eef = octave.feval('getJacobian', obj.params['controls']['q'], obj.params['defi']['ttype'], obj.params['defi']['H'], obj.params['defi']['P'], obj.params['defi']['n'])
            
            data = []
            
            # Search if the camera module fails to find the tag for 30 consecutive frames
            for x in range(0,30):
            	data = camera.get_all_poses()[tag_index]
            	if data != []:
            	    break;

        	# SEARCH TERMINATED DUE TO BREAK SIGNAL
            if data == []:
            	print "No data."
            	return
        
			# Follow tag while it is in view
            else:                
            	Z = data[0] 
            	
            	# Kalman Filter: Initialization 
            	if StateVector == []:
            		StateVector = np.concatenate( (Z,np.array([[10],[10],[10]])),axis =0 )	    
    	        	# Kalman Filter: Prediction
            		StateVector = np.matmul(Phi, StateVector)
            		Cov = np.matmul(np.matmul(Phi, Cov),np.transpose(Phi)) + Q
            	
            		# Kalman Filter: Update
            		K = np.matmul(np.matmul(Cov, np.transpose(H)),np.linalg.inv(np.matmul(H,np.matmul(Cov, np.transpose(H))) +R))
            		StateVector = StateVector + np.matmul(K,(Z-np.matmul(H,StateVector)))
            		Cov = (np.eye(6,6)-K*H)*Cov        
            		Z_est = np.array(StateVector[:3]) 
            
            		Poa = np.matmul(Roc, Z_est) + Poc
            		Poa = Coordinate_mapping(Poa)
            		print "Poa:", np.transpose(Poa)
            		obj.params['controls']['dq'] = np.linalg.inv(J_eef[3:6, :])*(Poa - obj.params['controls']['q'])
            		
            		# desired joint velocity
            		obj.params['controls']['q'] = obj.params['controls']['q'] + obj.params['controls']['dq']*dt
            		res, state = egm.receive_from_robot(0.01)
      
            		if res:
		                a = np.array(state.joint_angles)
        		        a = a * 180 / np.pi
        		        print "Joints: " + str(a)
        		        egm.send_to_robot([float(x)*180/np.pi for x in obj.params['controls']['q']])
        		        print "Target Joints: " + str([float(x)*90/np.pi for x in obj.params['controls']['q']])
	   

if __name__ == '__main__':
    sys.stdout.write("Initiliazing the Camera..." )

    camera = AR_Camera.Camera(CAMERA_ID, VIDEO_MODE, Tag_A, Tag_B, Tag_C)
    camera.start()

    print "OK!\n"

    # DISPLAY CAMERA PROPERTIES
    print camera
    
    options = '''COMMANDS:
    get data
    get poses
    get poa
    track
    quit

    ENTER COMMAND:
    '''
    
    while True:
        print options
        command = raw_input("").lower()

        if command == "get poses":
	        # Get camera data
                data = camera.get_all_poses()
                print data
                
        elif command == "get poa":
            get_Poa(camera)           
            
        elif command == "get data":
	    # This will print out the current camera capture data
            get_data(camera)
            
        elif command == "track":
	    # This will print out the current camera capture data
            # Begin Tracking
            track(camera)
            
        elif command == "quit":
            # Stop the camera device
            sys.stdout.write("Releasing camera...")
            camera.release()
	    camera.join()
            print "OK!"
            break;    

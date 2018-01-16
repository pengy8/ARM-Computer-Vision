import cv2
import argparse
import numpy as np
import threading
import time
from hampy import detect_markers
import warnings
import PyCapture2


def printBuildInfo():
	libVer = PyCapture2.getLibraryVersion()
	print "PyCapture2 library version: ", libVer[0], libVer[1], libVer[2], libVer[3]
	print

def printCameraInfo(cam):
	camInfo = cam.getCameraInfo()
	print "\n*** CAMERA INFORMATION ***\n"
	print "Serial number - ", camInfo.serialNumber
	print "Camera model - ", camInfo.modelName
	print "Camera vendor - ", camInfo.vendorName
	print "Sensor - ", camInfo.sensorInfo
	print "Resolution - ", camInfo.sensorResolution
	print "Firmware version - ", camInfo.firmwareVersion
	print "Firmware build time - ", camInfo.firmwareBuildTime
	print

    
    
class Camera(threading.Thread):
    def __init__(self, camera_id = 0, video_mode = False, TagA = [None, None] ,TagB = [None, None], TagC = [None, None], constant_update = True):
        warnings.simplefilter("ignore")

        threading.Thread.__init__(self)
        self.thread_id = 1

        self.ID = camera_id
        self.A_tag = TagA
        self.B_tag = TagB
        self.C_tag = TagC
        
        # Should the program get data continuously or only when requested?
        # Set to false if you have limited cpu resources
        self.constant_update = constant_update

        # Intrinsic Parameters
        self.camMatrix = np.zeros((3, 3),dtype=np.float64)
        self.camMatrix[0][0] = 1775.2#2017.7
        self.camMatrix[0][2] = 775.8#727.1
        self.camMatrix[1][1] = 1769.5#2015.3
        self.camMatrix[1][2] = 668.9#651.7
        self.camMatrix[2][2] = 1.0

        self.distCoeff = np.zeros((1, 5), dtype=np.float64)
        self.distCoeff[0][0] = 0.3926#-0.4534#0.01584
        self.distCoeff[0][1] = 0.3781#0.4613#0.37926
        self.distCoeff[0][2] = 0.0#-0.00056
        self.distCoeff[0][3] = 0.0#0.00331
        self.distCoeff[0][4] = 0.0

        # Initial Capture Data Values
        self.img = None
        self.A_Pose = [None, None]
        self.B_Pose = [None, None]
        self.C_Pose = [None, None]

        self.play_video = video_mode

        self.initialize()


    def initialize(self):
        # Initialize video capture
        # Ensure sufficient cameras are found
        bus = PyCapture2.BusManager()
        numCams = bus.getNumOfCameras()
        print "Number of cameras detected: ", numCams
        if not numCams:
        	print "Insufficient number of cameras. Exiting..."
        	self.exit()
        
        # Select camera on 0th index
        self.c = PyCapture2.Camera()
        uid = bus.getCameraFromIndex(0)
        self.c.connect(uid)
        printCameraInfo(self.c)
               
        print "Starting image capture..."
#        self.c.startCapture()
#
#
#        try:
#			  image = self.c.retrieveBuffer()
#        except PyCapture2.Fc2error as fc2Err:
#			  print "Error retrieving buffer : ", fc2Err
#
#        self.cap = image.convert(PyCapture2.PIXEL_FORMAT.BGR)
##        newimg.save("fc2TestImage.png", PyCapture2.IMAGE_FILE_FORMAT.PNG)
#              
#        self.c.stopCapture()
        

    def release(self):     
        self.is_stopped = True
        time.sleep(1)
        # get rid of video stream window
        if self.play_video:
            cv2.destroyWindow('live')
        # Release video capture
        self.c.disconnect()
#        self.cap.release()


    def __str__(self):
        # String representation of this camera
        output = "===========CAMERA INFORMATION===========\n"
        output += "Camera Device ID: " + str(self.ID)
        output += "\n\nIntrinsic Parameters: \n" + str(self.camMatrix) + "\n"
        output += "\nRegistered AR Tags:"
        if self.A_tag != [None, None]:
            output += "\nA: \t ID: {:10} \t Size: {:3}mm".format(self.A_tag[0], self.A_tag[1])
        if self.B_tag != [None, None]:
            output += "\nB: \t ID: {:10} \t Size: {:3}mm".format(self.B_tag[0], self.B_tag[1])
        if self.C_tag != [None, None]:
            output += "\nC: \t ID: {:10} \t Size: {:3}mm".format(self.C_tag[0], self.C_tag[1])
        output += "\n========================================\n"
        return output


    def run(self):
        self.is_stopped = False
        while not self.is_stopped:
            # Get Data as often as possible if playing video or set to constantly update
            if self.constant_update or self.play_video:
                self.capture_data()

            # Show Video
            if self.play_video:
                if self.img is not None:
                    cv2.imshow('live', self.img)

                    if cv2.waitKey(1) & 0xFF == ord('p'):
                        cv2.imwrite('out.jpg', self.img)


    def capture_data(self):

        # Get Frame
        self.c.startCapture()


        try:
			  image = self.c.retrieveBuffer()
        except PyCapture2.Fc2error as fc2Err:
			  print "Error retrieving buffer : ", fc2Err

        self.cap = image.convert(PyCapture2.PIXEL_FORMAT.BGR)
#        newimg.save("fc2TestImage.png", PyCapture2.IMAGE_FILE_FORMAT.PNG)
              
        self.c.stopCapture()        
        
        self.img = np.array(self.cap.getData(), dtype="uint8").reshape( (self.cap.getRows(), self.cap.getCols(),3) )

            
        if self.img is None:
            # Bad Image, do nothing
            return
        
        self.img = cv2.cvtColor(self.img, cv2.COLOR_BGR2RGB)


        markers = detect_markers(self.img)
        
        #print markers
        # Assume no objects are available until proven otherwise
        A_unavailable = True
        B_unavailable = True
        C_unavailable = True

        # for each valid tag, get the pose
        for m in markers:
            # Draw the marker outline on the image
            m.draw_contour(self.img)

            # Label the tag ID on the image
            if cv2.__version__[0] == "2":
                # Latest Stable Version
                cv2.putText(self.img, str(m.id), tuple(int(p) for p in m.center), cv2.cv.CV_FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 3)
            else:
                # version 3.1.0 (Dylans Version)
                cv2.putText(self.img, str(m.id), tuple(int(p) for p in m.center), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 3)
            
            # Get A pose if AR tag was detected
            if (m.id == self.A_tag[0]):
                self.A_Pose = self.get_object_pose(m, self.A_tag)
                A_unavailable = False

            # Get B pose if AR tag was detected
            elif (m.id == self.B_tag[0]):
                self.B_Pose = self.get_object_pose(m, self.B_tag)
                B_unavailable = False

            # Get C pose if AR tag was detected
            elif (m.id == self.C_tag[0]):
                self.C_Pose = self.get_object_pose(m, self.C_tag)
                C_unavailable = False
            elif (self.C_tag[0] == -100):
                # C tag is in return any mode, get this unknown tag and set it to C pose
                self.C_Pose = self.get_object_pose(m, self.C_tag)
                C_unavailable = False                

        # set poses for objects not found
        if A_unavailable:
            self.A_Pose = [None, None]
        if B_unavailable:
            self.B_Pose = [None, None]
        if C_unavailable:
            self.C_Pose = [None, None]

        # Finished capturing available data, return
        return


    # Given a matching marker and tag, get the pose
    def get_object_pose(self, marker, tag):
        # AR Tag Dimensions
        objPoints = np.zeros((4, 3), dtype=np.float64)
        objPoints[0,0] = -1.0*tag[1]/2.0
        objPoints[0,1] = tag[1]/2.0
        objPoints[0,2] = 0.0
        objPoints[1,0] = tag[1]/2.0
        objPoints[1,1] = tag[1]/2.0
        objPoints[1,2] = 0.0
        objPoints[2,0] = tag[1]/2.0
        objPoints[2,1] = -1*tag[1]/2.0
        objPoints[2,2] = 0.0
        objPoints[3,0] = -1*tag[1]/2.0
        objPoints[3,1] = -1*tag[1]/2.0
        objPoints[3,2] = 0.0

        # Get each corner of the tags
        imgPoints = np.zeros((4, 2), dtype=np.float64)
        for i in range(4):
            imgPoints[i, :] = marker.contours[i, 0, :]

        camPos = np.zeros((3, 1))
        camRot = np.zeros((3, 1))

        # SolvePnP
        retVal, rvec, tvec = cv2.solvePnP(objPoints, imgPoints, self.camMatrix, self.distCoeff)
        Rca, b = cv2.Rodrigues(rvec)
        Pca = tvec

        return [Pca, Rca]

    def get_all_poses(self):
        # If the program is set to not automatically capture data, capture it now
        if not self.constant_update and not self.play_video:
            self.capture_data()     
        # Return up to date data
        return (self.A_Pose, self.B_Pose, self.C_Pose)

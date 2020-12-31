import sys
sys.path.insert(0,'/home/salar/Libraries/DJITelloPy')
from djitellopy import Tello
import cv2
import pygame
from pygame.locals import *
import numpy as np
import time

import roslib
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


# Frames per second of the pygame window display
FPS = 25

class FrontEnd(object):
    """ Maintains the Tello display and moves it through the keyboard keys.
        Press escape key to quit.
    """

    def __init__(self):
        # Init pygame
        pygame.init()
        # Creat pygame window
        pygame.display.set_caption("Tello video stream")
        self.screen = pygame.display.set_mode([960, 720])
        # create update timer
        pygame.time.set_timer(USEREVENT + 1, 50)

        # Init Tello object that interacts with the Tello drone
        self.tello = Tello()
        self.send_rc_control = False

        #ROS publisher
        self.image_pub = rospy.Publisher("tello_frame",Image,queue_size = 10)
        self.bridge = CvBridge()

    def run(self):

        if not self.tello.connect():
            print("Tello not connected")
            return
        # In case streaming is on. This happens when we quit this program without the escape key.
        if not self.tello.streamoff():
            print("Could not stop video stream")
            return
        if not self.tello.streamon():
            print("Could not start video stream")
            return

        frame_read = self.tello.get_frame_read()

        should_stop = False
        while not should_stop:
            for event in pygame.event.get():
                if event.type == USEREVENT + 1:
                    self.update()
                elif event.type == QUIT:
                    should_stop = True
                elif event.type == KEYDOWN:
                    if event.key == K_ESCAPE:
                        should_stop = True

            if frame_read.stopped:
                frame_read.stop()
                break

            self.screen.fill([0, 0, 0])
            frame = cv2.cvtColor(frame_read.frame, cv2.COLOR_BGR2RGB)

            # Publish to ROS`
            try:
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, "rgb8"))
            except CvBridgeError as e:
                print(e)

            frame = np.rot90(frame)
            frame = np.flipud(frame)

            frame = pygame.surfarray.make_surface(frame)
            self.screen.blit(frame, (0, 0))
            pygame.display.update()

            time.sleep(1 / FPS)

        # Call it always before finishing. I deallocate resources.
        self.tello.end()

    def update(self):
        """ Update routine. Send velocities to Tello."""
        return

if __name__ == '__main__':
    rospy.init_node('SLAM_ROS', anonymous=True)
    #rospy.spin()
    FrontEnd().run()

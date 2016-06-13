'''
ArduinoTracker
==============

Uses OpenCV mean-shift based color tracking.
Select color objects by clicking and dragging around your selection.

The green ellipse tracks the object. The center of the region of interest
is marked, along with the approximate axis of orientation.

Usage:
------
    ArduinoTracker.py [<video source>] [<serial port>] [<baud rate>]

    Baud rate is 115200 by default.

    After selecting object to track, click to choose destination.
    A red dot will mark the intended destination, connected to the center 
    of the object of interest by a blue line.

    Press ESC to quit.
'''

import cv2
import numpy as np
import math
import time

class RCTracker(object):
    def __init__(self, video_src):
        self.cam = cv2.VideoCapture(video_src)
        ret, self.frame = self.cam.read()
        cv2.namedWindow('RC Tracker')
        cv2.setMouseCallback('RC Tracker', self.mouse)

        self.selection = None
        self.drag_start = None
        self.tracking_state = 0
        self.dest = None
        self.state = 'Servo: Pivot'
        self.pivot_dir = 'l' # default pivot direction left
        self.move_dir = 'f' # default move direction forward
        self.prev_angle = 0
        self.prev_dist = 0
        self.counter = 0

    def mouse(self, event, x, y, flags, param):
        '''
        Actions on mouse click.
        '''
        if event == cv2.EVENT_LBUTTONDOWN:
            self.drag_start = (x, y)
            if self.tracking_state == 1:
                self.dest = (x,y)
                self.state = 'Idle'
            else:
                self.tracking_state = 0

        if self.drag_start:
            if flags & cv2.EVENT_FLAG_LBUTTON:
                h, w = self.frame.shape[:2]
                xo, yo = self.drag_start
                x0, y0 = np.maximum(0, np.minimum([xo, yo], [x, y]))
                x1, y1 = np.minimum([w, h], np.maximum([xo, yo], [x, y]))
                self.selection = None
                if x1-x0 > 0 and y1-y0 > 0:
                    self.selection = (x0, y0, x1, y1)
            else:
                self.drag_start = None
                if self.selection is not None:
                    self.tracking_state = 1

    def dist(self, pt1, pt2):
        '''
        Calculate distance between two points.
        '''
        return ((pt1[0] - pt2[0])**2 + (pt1[1] - pt2[1])**2)**0.5

    # def doAction(self, diff, center):
    #     '''
    #     Handle actions and state changes
    #     '''

    #     # Test default direction
    #     if self.state == 'Servo: Pivot':
    #         self.prev_angle = diff
    #         self.robot.sendCommand(self.pivot_dir)
    #         self.state = 'Pivot: Wait'

    #     # Wait 4 cycles
    #     elif self.state == 'Pivot: Wait':
    #         self.counter += 1
    #         if self.counter == 4:
    #             self.counter = 0
    #             self.robot.sendCommand('s')
    #             self.state = 'Determine Direction'

    #     # If default pivot direction wrong, change direction
    #     elif self.state == 'Determine Direction':
    #         if (
    #                 (abs(diff) > abs(self.prev_angle) and abs(self.prev_angle) < 90) or 
    #                 (abs(diff) < abs(self.prev_angle) and abs(self.prev_angle) > 90)  
    #             ):
    #             if self.pivot_dir == 'l':
    #                 self.pivot_dir = 'r'
    #             else:
    #                 self.pivot_dir = 'l'
    #         self.robot.sendCommand(self.pivot_dir)
    #         self.state = 'Turning'

    #     # Continue turning until angle between lines is small
    #     elif self.state == 'Turning':
    #         d = self.dist(center, self.dest)
    #         if d < 50:
    #             self.state = 'Done'
    #             self.robot.sendCommand('s')
    #         if abs(diff) < 8 or abs(diff) > 172:
    #             self.state = 'Correct'
    #             self.robot.sendCommand('s')

    #     # Correc the angle slightly
    #     elif self.state == 'Correct':
    #         if self.pivot_dir == 'l':
    #             self.robot.sendCommand('r')
    #         else:
    #             self.robot.sendCommand('l')
    #         self.state = 'Correct: Wait'

    #     # Wait 3 cycles
    #     elif self.state == 'Correct: Wait':
    #         self.counter += 1
    #         if self.counter == 3:
    #             self.counter = 0
    #             self.robot.sendCommand('s')
    #             self.state = 'Servo: Move'

    #     # Test default movement direction
    #     elif self.state == 'Servo: Move':
    #         self.prev_dist = self.dist(center, self.dest)
    #         self.robot.sendCommand(self.move_dir)
    #         self.state = 'Move: Wait'

    #     # Wait 6 cycles -- Movement direction is more critical, so wait longer
    #     elif self.state == 'Move: Wait':
    #         self.counter += 1
    #         if self.counter == 6:
    #             self.counter = 0
    #             self.robot.sendCommand('s')
    #             self.state = 'Determine Motion'

    #     # If movement direction wrong, change direction
    #     elif self.state == 'Determine Motion':
    #         d = self.dist(center, self.dest)
    #         if d > self.prev_dist:
    #             if self.move_dir == 'f':
    #                 self.move_dir = 'b'
    #             else:
    #                 self.move_dir = 'f'
    #         self.robot.sendCommand(self.move_dir)
    #         self.state = 'Moving'

    #     # Move until distance is small or if angle becomes too large
    #     elif self.state == 'Moving':
    #         if abs(diff) > 45 and abs(diff) < 135:
    #             self.state = 'Servo: Pivot'
    #             self.robot.sendCommand('s')
    #         d = self.dist(center, self.dest)
    #         if d < 50:
    #             self.robot.sendCommand('s')
    #             self.state = 'Done'


    def run(self):
        '''
        Main loop.
        '''

        while True:

            # read image from camera
            ret, self.frame = self.cam.read()
            vis = self.frame.copy()
            hsv = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)

            # create mask
            mask = cv2.inRange(hsv, np.array((0., 60., 32.)), np.array((180., 255., 255.)))

            # track the selected object
            if self.selection:
                x0, y0, x1, y1 = self.selection
                self.track_window = (x0, y0, x1-x0, y1-y0)
                hsv_roi = hsv[y0:y1, x0:x1]
                mask_roi = mask[y0:y1, x0:x1]
                hist = cv2.calcHist( [hsv_roi], [0], mask_roi, [16], [0, 180] )
                cv2.normalize(hist, hist, 0, 255, cv2.NORM_MINMAX);
                self.hist = hist.reshape(-1)

                vis_roi = vis[y0:y1, x0:x1]
                cv2.bitwise_not(vis_roi, vis_roi)
                vis[mask == 0] = 0

            if self.tracking_state == 1:
                self.selection = None
                prob = cv2.calcBackProject([hsv], [0], self.hist, [0, 180], 1)
                prob &= mask
                term_crit = ( cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 1 )
                track_box, self.track_window = cv2.CamShift(prob, self.track_window, term_crit)

                # show center of ellipse
                center = (int(track_box[0][0]), int(track_box[0][1]))
                cv2.circle(vis, center, 8, (0,255,0), -1)

                # show destination
                cv2.circle(vis, self.dest, 8, (0,0,255), -1) # red circle at destination

                # show direction line
                if self.dest:
                    cv2.line(vis, self.dest, center, (255,0,0), 2) # blue line to destination

                # calculate orientation
                or_angle = track_box[2]

                # show orientation axis
                pt2_x = int(round(center[0] + 100 * math.sin(-math.pi/180 * or_angle), 0))
                pt2_y = int(round(center[1] + 100 * math.cos(-math.pi/180 * or_angle), 0))
                pt2 = (pt2_x, pt2_y)

                try:
                    cv2.line(vis, center, pt2, (0, 255, 0), 2) # green line through orientation axis
                except:
                    pt2 = (center[0], center[1] + 30)
                    cv2.line(vis, center, pt2, (0, 255, 0), 2) 
                    print(center)

                try:
                    cv2.ellipse(vis, track_box, (0, 255, 0), 2) # green ellipse around selection
                except:
                    print(track_box)


                # Arduino calculations and movement
                if self.dest:
                    # Calculate angle difference
                    # Note that angles will be measured from horizontal and in degrees

                    or_angle = 90 - or_angle # correct orientation angle for consistent angle measure

                    dir_dx = center[0] - self.dest[0]
                    dir_dy = center[1] - self.dest[1]

                    dir_angle = math.atan2(dir_dy, dir_dx) # atan2 is better than atan
                    dir_angle = 180 - dir_angle * 180/math.pi # correct direction angle

                    diff = dir_angle - or_angle
                    if diff > 180:
                        diff = 180 - diff

                    # self.doAction(diff, center)

            cv2.imshow('RC Tracker', vis)

            # Quit on ESC
            ch = 0xFF & cv2.waitKey(5)
            if ch == 27:
                break

        # self.robot.sendCommand('s')
        cv2.destroyAllWindows()


if __name__ == '__main__':
    import sys
    try:
        video_src = sys.argv[1]
    except:
        video_src = 0
    print(__doc__)

    RCTracker(video_src).run()
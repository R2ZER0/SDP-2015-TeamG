import cv2
import numpy as np
import math
from collections import namedtuple
import warnings
import pdb
# Turning on KMEANS fitting:
KMEANS = False

# Turn off warnings for PolynomialFit
warnings.simplefilter('ignore', np.RankWarning)
warnings.simplefilter('ignore', RuntimeWarning)


BoundingBox = namedtuple('BoundingBox', 'x y width height')
Center = namedtuple('Center', 'x y')

class Tracker(object):
    def preprocess(self, frame, crop, min_color, max_color, contrast, blur):
        # Crop frame
        frame = frame[crop[2]:crop[3], crop[0]:crop[1]]

        # Apply simple kernel blur
        # Take a matrix given by second argument and calculate average of those pixels
        if blur > 1:
            frame = cv2.blur(frame, (blur, blur))

        # Set Contrast
        if contrast > 1.0:
            frame = cv2.add(frame, np.array([float(contrast)]))

        # Convert frame to HSV
        frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Create a mask
        frame_mask = cv2.inRange(frame_hsv, min_color, max_color)

        kernel = np.ones((5, 5), np.uint8)
        erosion = cv2.erode(frame_mask, kernel, iterations=1)

        # Apply threshold to the masked image, no idea what the values mean
        return_val, threshold = cv2.threshold(frame_mask, 127, 255, 0)

        # Find contours, they describe the masked image - our T
        contours, hierarchy = cv2.findContours(
            threshold,
            cv2.RETR_TREE,
            cv2.CHAIN_APPROX_SIMPLE
        )
        return (contours, hierarchy, frame_mask)
    def get_contours(self, frame, adjustments):
        """
        Adjust the given frame based on 'min', 'max', 'contrast' and 'blur'
        keys in adjustments dictionary.
        """
        try:
            if frame is None:
                return None
            if adjustments['blur'] > 1:
                frame = cv2.blur(frame, (adjustments['blur'], adjustments['blur']))

            if adjustments['contrast'] > 1.0:
                frame = cv2.add(frame, np.array([float(adjustments['contrast'])]))

            # Convert frame to HSV
            frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            # Create a mask
            frame_mask = cv2.inRange(frame_hsv, adjustments['min'], adjustments['max'])

            # Apply threshold to the masked image, no idea what the values mean
            return_val, threshold = cv2.threshold(frame_mask, 127, 255, 0)

            # Find contours
            contours, hierarchy = cv2.findContours(
                threshold,
                cv2.RETR_TREE,
                cv2.CHAIN_APPROX_SIMPLE
            )

            #print contours
            return contours
        except:
            return None

    def get_contour_extremes(self, cnt):
        """
        Get extremes of a countour.
        """
        leftmost = tuple(cnt[cnt[:, :, 0].argmin()][0])
        rightmost = tuple(cnt[cnt[:, :, 0].argmax()][0])
        topmost = tuple(cnt[cnt[:, :, 1].argmin()][0])
        bottommost = tuple(cnt[cnt[:, :, 1].argmax()][0])
        return (leftmost, topmost, rightmost, bottommost)

    def get_bounding_box(self, points):
        """
        Find the bounding box given points by looking at the extremes of each coordinate.
        """
        leftmost = min(points, key=lambda x: x[0])[0]
        rightmost = max(points, key=lambda x: x[0])[0]
        topmost = min(points, key=lambda x: x[1])[1]
        bottommost = max(points, key=lambda x: x[1])[1]
        return BoundingBox(leftmost, topmost, rightmost - leftmost, bottommost - topmost)

    def get_contour_corners(self, contour):
        """
        Get exact corner points for the plate given one contour.
        """
        if contour is not None:
            rectangle = cv2.minAreaRect(contour)
            box = cv2.cv.BoxPoints(rectangle)
            return np.int0(box)

    def join_contours(self, contours):
        """
        Joins multiple contours together.
        """
        cnts = []
        for i, cnt in enumerate(contours):
            if cv2.contourArea(cnt) > 5:
                cnts.append(cnt)
        return reduce(lambda x, y: np.concatenate((x, y)), cnts) if len(cnts) else None

    def get_largest_contour(self, contours):
        """
        Find the largest of all contours.
        """
        areas = [cv2.contourArea(c) for c in contours]
        return contours[np.argmax(areas)]

    def get_contour_centre(self, contour):
        """
        Find the center of a contour by minimum enclousing circle approximation.

        Returns: ((x, y), radius)
        """
        return cv2.minEnclosingCircle(contour)

    def get_angle(self, line, dot):
        """
        From dot to line
        """
        diff_x = dot[0] - line[0]
        diff_y = line[1] - dot[1]
        angle = np.arctan2(diff_y, diff_x) % (2 * np.pi)
        return angle

class RobotTracker(Tracker):

    def __init__(self, color, crop, offset, pitch, name, calibration):
        """
        Initialize tracker.

        Params:
            [string] color      the name of the color to pass in
            [(left-min, right-max, top-min, bot-max)]
                                crop  crop coordinates
            [int]       offset          how much to offset the coordinates
            [int]       pitch           the pitch we're tracking - used to find the right colors
            [string]    name            name for debug purposes
            [dict]      calibration     dictionary of calibration values
        """
        self.name = name
        self.crop = crop

        self.color = [calibration[color]]

        self.color_name = color
        self.offset = offset
        self.pitch = pitch
        self.calibration = calibration

    def crop_to(self, frame, x, y, crop):
        min_x = int(max(0, x - crop / 2))
        max_x = int(min(frame.shape[1], x + crop / 2))
        min_y = int(max(0, y - crop / 2))
        max_y = int(min(frame.shape[0], y + crop / 2))
        return frame[min_y:max_y, min_x:max_x]

    def find_I(self, frame, colour):
        """
        Find the coloured bit on the plate.
        """
        # Apply the settings set by the user
        # Adjustments are colors and contrast/blur
        adjustments = self.calibration[colour]
        contours = self.get_contours(frame.copy(), adjustments)
        return self.get_contour_corners(self.join_contours(contours))

    def find_dot(self, frame, x_offset, y_offset, angle=None):
        #pdb.set_trace()
        detector = cv2.SimpleBlobDetector()
        kp = detector.detect(frame)
        if len(kp) == 1:
            return Center(kp[0].pt[0] + x_offset, kp[0].pt[1] + y_offset)
        elif len(kp) > 1:
            height = frame.shape[0]
            width= frame.shape[1]
            r = 6
            scores = []
            for p in kp:
                x = p.pt[0]
                y = p.pt[1]
                mask_frame = np.zeros((height,width,3), np.uint8)
                screen_frame = frame.copy()
                cv2.circle(mask_frame, (x, y), r, (255, 255, 255), -1)
                # Mask the original image
                mask_frame = cv2.cvtColor(mask_frame, cv2.COLOR_BGR2GRAY)
                c = cv2.bitwise_and(screen_frame, screen_frame, mask=mask_frame)
                adjustment = self.calibration['dot']
                contours = self.get_contours(screen_frame, adjustment)
                if contours and len(contours) > 0:
                    # Take the largest contour
                    contour = self.get_largest_contour(contours)
                    contour_area = cv2.contourArea(contour)
                    scores.append(((x,y), contour_area))
            if len(scores) > 0:
                scores.sort(key=lambda x: x[1], reverse=True)
                return Center(int(scores[0][0][0] + x_offset), int(scores[0][0][1] + y_offset))
        return None


    def find(self, frame, queue, oldPos):
        old_x = oldPos[0]
        old_y = oldPos[1]
        old_angle = oldPos[2]
        # Trim the frame to a square centered around our old position, or if that is unknown to the entire zone
        if old_x is not None and old_y is not None:
            cropped = self.crop_to(frame, old_x, old_y, 100)
            offset_x = int(max(0, old_x - 50))
            offset_y = int(max(0, old_y - 50))
        else:
            cropped = frame[self.crop[2]:self.crop[3], self.crop[0]:self.crop[1]]
            offset_x = self.offset
            offset_y = 0
        # find the the coloured bit
        colour = self.find_I(cropped.copy(), self.color_name)
        if colour is not None:
            # find the center of they coloured bit
            ((x, y), _) = self.get_contour_centre(colour)
            x += offset_x
            y += offset_y

            # (2) Trim to create a smaller frame
            plate_frame = self.crop_to(frame.copy(), x, y, 50)
            offset_x = int(max(0, x - 25))
            offset_y = int(max(0, y - 25))
            # (3) Search for the dot
            
            # Pass in corners of the plate for dot estimation
            dot = self.find_dot(plate_frame, offset_x, offset_y, angle=old_angle)

            if dot is None:
                gray = cv2.cvtColor(plate_frame, cv2.COLOR_BGR2GRAY)
                gray = cv2.equalizeHist(gray)
                dot = self.find_dot(gray, offset_x, offset_y, angle = old_angle)
            if dot is None:
                x_edge = cv2.Sobel(gray, cv2.cv.CV_16S, 1, 0, 3)
                x_edge = cv2.convertScaleAbs(x_edge, x_edge)

                y_edge = cv2.Sobel(gray, cv2.cv.CV_16S, 0, 1, 3)
                y_edge = cv2.convertScaleAbs(y_edge, y_edge)

                total = cv2.addWeighted(x_edge,0.5,y_edge,0.5,0)
                dot = self.find_dot(total, offset_x, offset_y, angle = old_angle)

            if dot is not None:
                angle = math.atan2(y - dot.y, x - dot.x)

            else:
                side_A = math.hypot(colour[0][0] - colour[1][0], colour[0][1] - colour[1][1])
                side_B = math.hypot(colour[1][0] - colour[2][0], colour[1][1] - colour[2][1])
                # find the longer edge
                if side_A > side_B:
                    angle = math.atan2(colour[0][1] - colour[1][1], colour[1][0] - colour[0][0])
                else:
                    angle = math.atan2(colour[1][1] - colour[2][1], colour[2][0] - colour[1][0])
            while angle < 0:
                angle += 2 * math.pi
            while angle > 2 * math.pi:
                angle -= 2 * math.pi

            #front_centre = front_right = front_left= [x + math.cos(angle) * 15, y + math.sin(angle) * 15]
            #back_right = back_centre = back_left = 
            front_centre = [x + math.cos(angle) * 15, y + math.sin(angle) * 15]
            front_right = [front_centre[0] + math.cos(angle + math.pi / 2) * 15, front_centre[1] + math.sin(angle + math.pi / 2) * 15]
            front_left = [front_centre[0] - math.cos(angle + math.pi / 2) * 15, front_centre[1] - math.sin(angle + math.pi / 2) * 15]

            back_centre = [x - math.cos(angle) * 15, y - math.sin(angle) * 15]
            back_right = [back_centre[0] + math.cos(angle + math.pi / 2) * 15, back_centre[1] + math.sin(angle + math.pi / 2) * 15]
            back_left = [back_centre[0] - math.cos(angle + math.pi / 2) * 15, back_centre[1] - math.sin(angle + math.pi / 2) * 15]
            
            plate_corners = np.int0([front_right, front_left, back_left, back_right])
            # Front of the kicker should be the first two points in distances
            front = [front_right, front_left]
            rear = [back_left, back_right]

            # Put the results together
            sides = [
                (
                    Center(front_right[0], front_right[1]),
                    Center(back_right[0], back_right[1])
                ),
                (
                    Center(front_left[0], front_left[1]),
                    Center(back_left[0], back_left[1])
                )
            ]

            # Direction is a line between the front points and rear points
            direction = (
                    Center(front_centre[0], front_centre[1]),
                    Center(back_centre[0], back_centre[1])
            )        
            queue.put({
                'x': x, 'y': y,
                'name': self.name,
                'angle': angle,
                'dot': dot,
                'box': plate_corners,
                'direction': direction,
                'front': front
            })
            return
        queue.put({
            'x': None, 'y': None,
            'name': self.name,
            'angle': None,
            'dot': None,
            'box': None,
            'direction': None,
            'front': None
        })
        return
    def kmeans(self, plate):

        prep = plate.reshape((-1,3))
        prep = np.float32(prep)

        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)
        k = 4
        ret, label, colour_centers = cv2.kmeans(prep, k, criteria, 20, cv2.KMEANS_RANDOM_CENTERS)
        colour_centers = np.uint8(colour_centers)

        # Get the new image based on the clusters found
        res = colour_centers[label.flatten()]
        res2 = res.reshape((plate.shape))

        # if self.name == 'Their Defender':
        #     colour_centers = np.array([colour_centers])
        #     print "********************", self.name
        #     print colour_centers
        #     print 'HSV######'
        #     print cv2.cvtColor(colour_centers, cv2.COLOR_BGR2HSV)

        return res2


class BallTracker(Tracker):
    """
    Track red ball on the pitch.
    """

    def __init__(self, crop, offset, pitch, calibration, name='ball'):
        """
        Initialize tracker.

        Params:
            [string] color      the name of the color to pass in
            [(left-min, right-max, top-min, bot-max)]
                                crop  crop coordinates
            [int] offset        how much to offset the coordinates
        """
        self.crop = crop
        # if pitch == 0:
        #     self.color = PITCH0['red']
        # else:
        #     self.color = PITCH1['red']
        self.color = [calibration['red']]
        self.offset = offset
        self.name = name
        self.calibration = calibration

    def find(self, frame, queue, oldPos):

        for color in self.color:
            contours, hierarchy, mask = self.preprocess(
                frame,
                self.crop,
                color['min'],
                color['max'],
                color['contrast'],
                color['blur']
            )

            if len(contours) <= 0:
                # print 'No ball found.'
                pass
                # queue.put(None)
            else:
                # Trim contours matrix
                cnt = self.get_largest_contour(contours)

                # Get center
                (x, y), radius = cv2.minEnclosingCircle(cnt)

                queue.put({
                    'name': self.name,
                    'x': x,
                    'y': y,
                    'angle': None,
                    'velocity': None
                })
                return

        queue.put({
                    'name': self.name,
                    'x': None,
                    'y': None,
                    'angle': None,
                    'velocity': None
                })
        return

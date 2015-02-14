import cv2
import numpy as np
import tools
import argparse
import math

'''
calibrate.py contains a simple utility for determining the correct coordinates for 
cropping input frmaes and identifying the individual zones for each robot. It presents
a GUI that requires clicking on defined points around the perimeter of the Pitch and 
Zones and outputs the results to vision/calibrations/croppings.json

The utility is supplied with a pitch number indicating the desired pitch to calibrate,
with 0 being the main pitch and 1 being the side-room pitch.

Example for main pitch calibration:
        
        $ python calibrate.py 0
'''

FRAME_NAME = 'ConfigureWindow'

WHITE = (255,255,255)
BLACK = (0,0,0)
BLUE = (255, 0, 0)
GREEN = (0, 255, 0)
RED = (0, 0, 255)

# Retrieves the data for fixing barrel distortion of the feed
distort_data = tools.get_radial_data()
NCMATRIX = distort_data['new_camera_matrix']
CMATRIX = distort_data['camera_matrix']
DIST = distort_data['dist']

class Configure(object):
        '''
        The Configure class supplies a simple GUI for identifying cropping positions
        and zones.
        '''

        def __init__(self, pitch):
                '''
                Initialises the Configure GUI for the supplied parameters.

                :param pitch: The number of the pitch to calibrate croppings for. \
                        0 indicates the main pitch, 1 the side pitch.
                '''
                self.pitch = pitch

                # Intitialise video capture on standard port
                self.camera = cv2.VideoCapture(0)

                self.polygon = self.polygons = []
                self.points = []

                # The various keys used as indices into the drawing dictionary during
                # calibration.
                keys = ['outline', 'Zone_0', 'Zone_1', 'Zone_2', 'Zone_3']
                self.data = self.drawing = {}

                # Initialise empty lists for each key
                for key in keys:
                        self.data[key] = []
                        self.drawing[key] = []

                # The drawing colour for polygons
                self.color = RED


        def run(self, camera=False):
                '''
                Run constructs a new OpenCV window, displays the frame for cropping and
                begins the routine of the program asking for user input.

                :param camera: True indicates using the camera feed, False will use a \
                        image entitled '00000001.jpg' instead.
                '''
                frame = cv2.namedWindow(FRAME_NAME)

                # Set mouse callback to the self.draw function
                cv2.setMouseCallback(FRAME_NAME, self.draw)

                # If we're using the camera, pull multiple images to ensure
                # we have a valid one. (Initial pulled images can be strange.)
                if camera:
                        cap = cv2.VideoCapture(0)
                        for i in range(10):
                                status, image = cap.read()
                
                # Otherwise, use plain image
                else:
                        image = cv2.imread('00000001.jpg')

                # Apply barrel distortion fix to the image
                self.image = cv2.undistort(image, CMATRIX, DIST, None, NCMATRIX)

                self.get_zone('Zone_0', 'draw LEFT Defender')
                self.get_zone('Zone_2', 'draw LEFT Attacker')
                self.get_zone('Zone_1', 'draw RIGHT Attacker')
                self.get_zone('Zone_3', 'draw RIGHT Defender')
		
		self.data['Zone_3'].extend(self.data['Zone_2'])
		self.data['Zone_2'].extend(self.data['Zone_1'])
		self.data['Zone_1'].extend(sorted(self.data['Zone_0'], key = lambda x : x[0], reverse = True)[:2])
		minx = min(self.data['outline'], key = lambda x: x[0])[0]
		miny = min(self.data['outline'], key = lambda x: x[1])[1]		
		
		for k in ['Zone_0', 'Zone_1', 'Zone_2', 'Zone_3']:
			self.data[k] = [(x-minx, y-miny) for (x,y) in self.data[k]]
		for k in ['outline', 'Zone_0', 'Zone_1', 'Zone_2', 'Zone_3']:
			self.data[k] = self.sortPoly(self.data[k])

                # Calculate the goal positions for each zone
                self.get_goal('Zone_0')
                self.get_goal('Zone_3')

		self.draw_poly(self.reshape())
                print 'Press any key to finish.'
                cv2.waitKey(0)
                cv2.destroyAllWindows()

                # Write out the data
                tools.save_croppings(pitch=self.pitch, data=self.data)
	
	def sortPoly(self, points):
		cent = (sum([p[0] for p in points])/len(points), sum([p[1] for p in points])/len(points))
		# sort by polar angle
		points.sort(key=lambda p: math.atan2(p[1]-cent[1],p[0]-cent[0]))
		return points

        def reshape(self):
                return np.array(self.data[self.drawing], np.int32).reshape((-1,1,2))

        def draw_poly(self, points):
                '''
                Draws a series of points as a polygon onto our frame.

                :param points: A series of polygon points
                '''
                cv2.polylines(self.image, [points], True, self.color)
                cv2.imshow(FRAME_NAME, self.image)

        def get_zone(self, key, message):
                '''
                Initialises the request for positions of a new zone. Assigns the zone's
                key as the new drawing index and begins tracking points. 

                :param key: The area's key: ['outline', 'Zone_0', 'Zone_1', 'Zone_2', 'Zone_3']
                :param message: The message to prompt the user
                '''
                print '%s. %s' % (message, "Continue by pressing q")
                self.drawing = key
		k = True

                while k != ord('q'):
                        cv2.imshow(FRAME_NAME, self.image)
                        k = cv2.waitKey(100) & 0xFF

                

        def get_pitch_outline(self):
                '''
                Prompts the user to draw the Pitch outline as a series of points, then users these as
                extreme values to crop the camera image to these points and mask the outer edges to 
                remove outer noise.
                '''
                self.get_zone('outline', 'Draw the outline of the pitch. Contine by pressing \'q\'')

                # Setup black mask to remove outside noise
                self.image = tools.mask_pitch(self.image, self.data[self.drawing])

                # Get crop size based on points and crop
                size = tools.find_crop_coordinates(self.image, self.data[self.drawing])
                self.image = self.image[size[2]:size[3], size[0]:size[1]]

                cv2.imshow(FRAME_NAME, self.image)

        def draw(self, event, x, y, flags, param):
                '''
                Callback for the Mouse button press event. Detects if a mouse button is pressed and
                appends this position to our list of drawn points, and displays a point on the frame
                to indicate the press.
                '''
                if event == cv2.EVENT_LBUTTONDOWN:
                        color = self.color
                        cv2.circle(self.image, (x-1, y-1), 2, color, -1)
                        self.data[self.drawing].append((x,y))
			self.data['outline'].append((x,y))

        def get_goal(self, zone):
                '''
                Returns the top and bottom corner of the goal for the given zone by sorting the points
                into extreme left / right x-positions and retrieving the first/last two points.

                Writes the given positions under the Zone_X_goal key into croppings.
                '''
                coords = self.data[zone]
                reverse = int(zone[-1]) % 2
                goal_coords = sorted(coords, reverse=reverse)[:2]
                if goal_coords[0][1] > goal_coords[1][1]:
                        topCorner = goal_coords[1]
                        bottomCorner = goal_coords[0]
                else:
                        topCorner = goal_coords[0]
                        bottomCorner = goal_coords[1]
                self.data[zone + '_goal'] = [topCorner, bottomCorner]


if __name__ == '__main__':
        parser = argparse.ArgumentParser()
        parser.add_argument('pitch', help='Select pitch to be cropped [0, 1]')
        args = parser.parse_args()
        pitch_num = int(args.pitch)
        assert pitch_num in [0, 1]

        c = Configure(pitch=pitch_num)
        c.run(camera=True)

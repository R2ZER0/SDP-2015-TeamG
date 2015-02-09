import cv2

class Preprocessing(object):
    '''
    Preprocessing is a very simple class that currently performs minimal preprocessing before
    attempting to use the vision frames to locate objects on the pitch.

    Currently performs normalisation of colours and background subtraction based upon the settings
    in the main GUI.
    '''

    def __init__(self, options=None):
        '''
        Constructs a new preprocessing instance. Options default to None which sets 
        all options to False by default.
        '''

        if not options:
            # Determines which tasks to run
            self.options = {
                'normalize': False,
                'background_sub': False
            }

        # Assign None to indicate non-initialisation. After BackgroundSubtractor is 
        # initialised this will be assigned a running instance.
        self.background_sub = None

    def get_options(self):
        return self.options;

    def run(self, frame, options):
        '''
        Accepts a given frame and options and runs the preprocessing steps specified by
        options.

        Attributes:
            frame   The image to preprocess
            options A dictionary containing True/False values for each preprocessing task.
        '''
        self.options = options

        # Supply the original frame as one result
        results = {
            'frame': frame
        }

        # Apply normalization
        if self.options['normalize']:
            # Normalize only the saturation channel
            results['frame'] = self.normalize(frame)

        # Apply background subtraction
        if self.options['background_sub']:

            # Apply a minor blur before background subtraction to reduce noise
            frame = cv2.blur(frame, (2,2))

            # If we're not in the first initialisation, update
            if self.background_sub is not None:
                bg_mask = self.background_sub.apply(frame)

            # Otherwise, begin the background subtraction
            else:
                self.background_sub = cv2.BackgroundSubtractorMOG2(0, 30, False)
                bg_mask = self.background_sub.apply(frame)

            results['background_sub'] = bg_mask

        return results

    def normalize(self, frame):
        '''
        Normalises the given frame purely on it's saturation value.

        Attributes:
            frame   The image frame to normalise 

        Returns:
            A new frame, with the saturation values normalised and converted
            back to BGR indexing.
        '''
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        frame[:, :, 1] = cv2.equalizeHist(frame[:, :, 1])
        return cv2.cvtColor(frame, cv2.COLOR_HSV2BGR)

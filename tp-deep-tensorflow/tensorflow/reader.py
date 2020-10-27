import numpy as np
import cv2
import os

class PNGReader:
    # PNG reader, reads files and stores them in memory.
    def __init__(self, path2trainfile, path2train, path2valfile, path2val):
        self.train_file = path2trainfile
        self.val_file = path2valfile
        self.train_root = path2train
        self.val_root = path2val
        self.load_data()
        self.compute_mean()

    def load_data(self):
        # LOAD TRAIN
        x_path, self.y_train = self.read_file(self.train_file)
        self.x_train = self.load_x(x_path, self.train_root)
        self.train_size = self.x_train.shape[0]
        # LOAD TEST
        x_path, self.y_val = self.read_file(self.val_file)
        self.x_val = self.load_x(x_path, self.val_root)
        self.val_size = self.x_val.shape[0]
        
    def read_file(self, filepath):
        # Reads the label file
        x_path = []
        y = []
        with open(filepath) as f:
            line = f.readline()
            while line:
                data = line.strip().split(' ')
                x_path.append(data[0])
                y.append(int(data[1]))
                line = f.readline()

        return x_path, np.array(y)

    def load_x(self, path_list, root):
        # Read the image from the label file and store them in memory.
        '''
        If you want to make RGB-D (RGB-Depth) network it's possible since we use
        .png files, simply add an alpha channels to them possibly adapt your mean/
        std algorithms and it should be fine.
        '''
        x = []
        for path in path_list:
            path = os.path.join(root, path)
            raw = cv2.imread(path, cv2.IMREAD_UNCHANGED)
            sx, sy = raw.shape[0:2]
            # TODO : Reshape the image to the desired shape
            height = 32 
            width = 32
            x.append(cv2.resize(raw,(0,0),fx=height/sx,fy=width/sy, interpolation = cv2.INTER_AREA))
        return np.array(x)

    def compute_mean(self):
        # TODO : Compute the mean and standard deviation of the dataset channel wise
        '''
        By channel wise it means that the mean and standard deviation
        must be a array of size number of channel (if BGR then 3), where the
        first element is the mean or std_dev of the blue channel,
        the second the mean or std_dev of the green channel and the
        third the mean or std_dev of the red channel.
        '''
        # Keep in mind that openCV is BGR, not RGB, the mean must be computed only
        # on the training set.

        self.mean = #TODO
        self.std = #TODO 

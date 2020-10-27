import numpy as np

class UniformSampler:
    '''
    Samples batches uniformly out of the dataset.
    Relies on python3 generators.
    '''
    def __init__(self, Dataset):
        self.DS = Dataset
        self.TB = None
        self.TEB = None
        self.VB = None

    def sample(self, x, y, bs):
        # Shuffle before each epoch
        s = list(range(x.shape[0]))
        np.random.shuffle(s)
        x = x.copy()[s]
        y = y.copy()[s]
        
        # Split into batches
        bx = []
        by = []
        for i in range(int(x.shape[0]/bs)):
            bx.append(x[i*bs:(i+1)*bs,:,:,:])
            by.append(y[i*bs:(i+1)*bs])
        bx = np.array(bx)
        by = np.array(by)
        
        # Iterate one step further, every time the object is called
        # with 'next'
        for i in zip(bx,by):
            yield i[0], i[1]

    def sample_train_batch(self, bs):
        # Tries to call 'next' if fail recreates the generator
        try:
            return next(self.TB)
        except:
            self.TB = self.sample(self.DS.x_train, self.DS.y_train, bs)
            return next(self.TB)

    def sample_train_eval_batch(self, bs):
        # Tries to call 'next' if fail recreates the generator
        try:
            return next(self.TEB)
        except:
            self.TEB = self.sample(self.DS.x_train, self.DS.y_train, bs)
            return next(self.TEB)

    def sample_val_batch(self, bs):
        # Tries to call 'next' if fail recreates the generator
        try:
            return next(self.VB)
        except:
            self.VB = self.sample(self.DS.x_val, self.DS.y_val, bs)
            return next(self.VB)


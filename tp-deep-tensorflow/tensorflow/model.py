import tensorflow as tf

# Made for TensorFlow 1.14 (If you are using 2.X.X consider disabling eager execution)

# This code implements a Convolutional Neural Network with a tensorboard front-end. Tensorboard
# is a nice visualization tool embedded within TensorFlow.
# Please note that some errors have been intentionaly made inside the code. They mostly concern
# layer connections or layer size.
# Please note that editing the name of the variables may break the tensorboard compatibility.

class CNNModel:
    # Please note that this model is not build using KERAS, only the keras layers.
    def __init__(self, height, width, channels, classes, learning_rate, mean, std):
        '''
        PLACEHOLDERS: The placeholders are the inputs of the network in tensorflow.
        "?" is an unknown dimension: in this case the size of the batch
        '''
        # TODO : set the height, width and channels so they match the reader settings
        self.X           = tf.compat.v1.placeholder(tf.float32, [None, height, width, channels], name='source') 
        self.Y           = tf.compat.v1.placeholder(tf.int64, [None], name='labels')
        self.dropout     = tf.compat.v1.placeholder(tf.float32, name='drop_prob')
        self.is_training = tf.compat.v1.placeholder(tf.bool, name='is_training')
        self.step        = tf.compat.v1.placeholder(tf.int32, name='train_step')
        '''
        REGULARIZATION: Norm using average of each channel and divide by std devition
        '''
        print('### PRINTING LAYERS SHAPE ###)')
        print('input_dim: ', self.X.shape)
        self.mean = tf.convert_to_tensor(mean, dtype = tf.float32, name='mean')
        self.std = tf.convert_to_tensor(std, dtype = tf.float32, name='std')
        # TODO : Normalize the input i.e. (input-mean)/std
        self.x = (self.X - self.mean)/self.std
        print('regularization_dim: ', self.x.shape)
        '''
        MODEL: The following operations define the model of the neural network.
        '''
        # TODO : Rebuild the network it should work as is with a dropout value
        # of 0.1, to rebuild the network keep in mind that it should be
        # sequential and that the layers are written in execution order
        # TODO : Tune some hyper-parameters ? Change the amount of dense neurons ?
        self.conv1  = tf.keras.layers.Conv2D(6, [6,6], strides=(2,2), padding='same', activation=tf.nn.relu, name='conv_1')(self.x)
        self.drop1  = tf.keras.layers.Dropout(self.dropout, name='dropout_cv1')(self.conv1, training=self.is_training )
        print('convolution_1_dim: ', self.conv1.shape)
        self.norm1  = tf.keras.layers.LayerNormalization(name='layer_norm_1')(self.drop1)
        print('norm_1_dim: ', self.norm1.shape)
        self.conv2  = tf.keras.layers.Conv2D(12, [5,5], strides=(2,2), padding='same', activation=tf.nn.relu, name='conv_2')(self.norm1)
        self.drop2  = tf.keras.layers.Dropout(self.dropout, name='dropout_cv2')(self.conv2, training=self.is_training )
        print('convolution_2_dim: ', self.drop2.shape)
        self.norm2  = tf.keras.layers.LayerNormalization(name='layer_norm_2')(self.drop2)
        print('norm_2_dim: ', self.norm2.shape)
        self.conv3  = tf.keras.layers.Conv2D(24, [4,4], strides=(2,2), padding='same', activation=tf.nn.relu, name='conv_3')(self.norm2)
        self.drop3  = tf.keras.layers.Dropout(self.dropout, name='dropout_cv3')(self.conv3, training=self.is_training )
        print('convolution_3_dim: ', self.conv3.shape)
        self.flat   = tf.keras.layers.Flatten()(self.drop3)
        print('flat_dim: ', self.flat.shape)
        self.dense1 = tf.keras.layers.Dense(200, name='dense')(self.flat)
        print('dense_1_dim: ', self.dense1.shape)
        self.drop4  = tf.keras.layers.Dropout(self.dropout, name='dropout')(self.dense1, training=self.is_training )
        print('dropout_1_dim: ', self.drop4.shape)
        self.y_     = tf.keras.layers.Dense(classes, name='raw_output')(self.drop4)
        print('raw_output_dim: ', self.y_.shape)
        self.pred = tf.nn.softmax(self.y_, name='predictions')
        print('prediction_dim: ', self.pred.shape)
        '''
        OPERATIONS: The following operation allows to train and evaluate the model
        '''
        # From there it should be fine
        with tf.name_scope('loss_op'):
            '''
            If you want to use regression you will have to change a few things here.
            Remove the OneHot embedding of the labels
            Change the types of the labels to tf.float32 in the placeholder defined above
            You'll probably have to rewrite the reader so it reads arrays instead of single points
            Change the loss to tf.losses.Hubert or tf.losses.MeanSquaredError.
            '''
            self.y = tf.one_hot(self.Y, classes, name='onehot_label_casting', dtype=tf.int64)
            print('one_hot_labels_shape: ', self.y_.shape)
            self.loss = tf.reduce_mean(tf.nn.softmax_cross_entropy_with_logits_v2(self.y, self.y_))
            tf.summary.scalar('loss', self.loss)
        print('loss_dim: ', self.loss.shape)
        self.train = tf.compat.v1.train.AdamOptimizer(learning_rate,).minimize(self.loss)
        with tf.name_scope('accuracy_op'):
            '''
            If you want to use regression you will have to change a few things here.
            Rewrite the accuracy function to RMSE
            '''
            self.accuracy = tf.reduce_mean(tf.cast(tf.equal(self.Y, tf.argmax(self.pred,axis=-1)),dtype=tf.float32))
            tf.summary.scalar('accuracy', self.accuracy)
        print('accuracy_dim: ',self.accuracy.shape)
        # SUMMARIES: Tensorboard frontend link
        self.merged = tf.summary.merge_all()




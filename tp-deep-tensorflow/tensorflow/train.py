import tensorflow as tf
import numpy as np
import os

from parser import ParseArgs
from sampler import UniformSampler
from reader import PNGReader
from model import CNNModel
#from model import KerasModel

class Train:
    def __init__(self):
        self.settings = ParseArgs()
        self.dataset = PNGReader(self.settings.train_file, self.settings.train_root, self.settings.val_file, self.settings.val_root)
        self.sampler = UniformSampler(self.dataset)
        self.model = CNNModel(32, 32, 3, self.settings.num_class, self.settings.lr, self.dataset.mean, self.dataset.std)

    def train(self):
        best_acc = 10000
        with tf.compat.v1.Session() as self.sess:
            self.train_writer = tf.compat.v1.summary.FileWriter(self.settings.tensorboard_dir+'/train',self.sess.graph)
            self.val_writer = tf.compat.v1.summary.FileWriter(self.settings.tensorboard_dir+'/val')
            self.sess.run(tf.compat.v1.global_variables_initializer())
            self.saver = tf.compat.v1.train.Saver()
            print('Training has started...')
            print('Open tensorboard to visualize the training resutls.')
            for step in range(self.settings.max_iter):
                TBX, TBY = self.sampler.sample_train_batch(self.settings.bs)
                _ = self.sess.run(self.model.train, feed_dict={self.model.X:TBX,
                                                               self.model.Y:TBY,
                                                               self.model.dropout:self.settings.dropout,
                                                               self.model.is_training:True,
                                                               self.model.step: step})
                
                if step%10==0: # Eval on train-set
                    TEBX, TEBY = self.sampler.sample_train_eval_batch(500)
                    summary, tr_loss, tr_acc = self.sess.run([self.model.merged,
                                                              self.model.loss,
                                                              self.model.accuracy],
                                                              feed_dict={self.model.X:TEBX,
                                                                         self.model.Y:TEBY,
                                                                         self.model.dropout:0,
                                                                         self.model.is_training:False,
                                                                         self.model.step: step})
                    self.train_writer.add_summary(summary, step)
                    
                if step%25==0: # Eval on validation-set
                    VBX, VBY = self.sampler.sample_val_batch(500)
                    summary, v_loss, v_acc  = self.sess.run([self.model.merged,
                                                             self.model.loss,
                                                             self.model.accuracy],
                                                             feed_dict={self.model.X:VBX,
                                                                        self.model.Y:VBY,
                                                                        self.model.dropout:0,
                                                                        self.model.is_training:False,
                                                                        self.model.step: step})
                    self.val_writer.add_summary(summary, step)
                    if v_acc < best_acc:
                        best_acc = v_acc
                        self.saver.save(self.sess, os.path.join(self.settings.model_dir,'best_model'))
                if step%100==0:
                    print('Train loss : ', tr_loss, 'train accuracy : ', tr_acc*100,'%')
                    print('Val loss   : ', v_loss,  'val accuracy   : ', v_acc*100,'%')
            print('Training done ! ^_^')

if __name__ == '__main__':
    T = Train()
    T.train()

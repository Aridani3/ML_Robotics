import os
import argparse

class ParseArgs():
    def __init__(self):
        self.assign_args()

    def arg_parser(self):
        parser = argparse.ArgumentParser()
        parser.add_argument('--train_file', type=str, help='path to the train description file')
        parser.add_argument('--train_root', type=str, help='path to the training files directory')
        parser.add_argument('--val_file', type=str, help='path to the val description file')
        parser.add_argument('--val_root', type=str, help='path to the validation files directory')
        parser.add_argument('--output', type=str, help='path to where the model will be stored')
        parser.add_argument('--learning_rate', type=float, help='learning rate')
        parser.add_argument('--iter', type=int, help='maximum amount of iterations')
        parser.add_argument('--classes', type=int, help='number of output classes')
        parser.add_argument('--batch_size', type=int, default=32, help='size of the batch')
        parser.add_argument('--dropout', type=float, default=0.7, help='keep probability value')
        args = parser.parse_args()
        return args

    def assign_args(self):
        args = self.arg_parser()
        if os.path.exists(args.train_file):
            self.train_file = args.train_file
        else:
            raise ValueError(args.train_file,' doesn\'t exist, check the path.')
        if os.path.exists(args.train_file):
            self.train_root = args.train_root
        else:
            raise ValueError(args.train_root,' doesn\'t exist, check the path.')
        if os.path.exists(args.val_file):
            self.val_file = args.val_file
        else:
            raise ValueError(args.val_file,' doesn\'t exist, check the path.')
        if os.path.exists(args.val_root):
            self.val_root = args.val_root
        else:
            raise ValueError(args.val_root,' doesn\'t exist, check the path.')
        try:
            self.output_dir = args.output
            self.model_dir = os.path.join(args.output,'models')
            self.tensorboard_dir = os.path.join(args.output,'tensorboard')
            os.mkdir(args.output)
            os.mkdir(self.model_dir)
            os.mkdir(self.tensorboard_dir)
        except:
            raise ValueError('Could not create directory at: ', args.output, ' check that a directory with the same name doesn\'t already exist or that the parent directory exists.')
        self.lr = args.learning_rate
        self.max_iter = args.iter
        self.num_class = args.classes
        self.bs = args.batch_size
        self.dropout = args.dropout

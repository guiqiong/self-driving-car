import argparse
import os
import csv
import math

from sklearn.utils import shuffle
from sklearn.model_selection import train_test_split

import cv2
import numpy as np
import random

import matplotlib.image as mpimg

from keras.models import Model
from keras.layers import Input, Dense, Lambda, Flatten, Dropout
from keras.layers.convolutional import Conv2D
from keras.callbacks import ModelCheckpoint
from keras.layers import Cropping2D
from keras.regularizers import l2
from keras.optimizers import Adam

import pandas as pd

cameras = ['left', 'center', 'right']
cameras_steering_correction = [.25, 0., -.25]


def balance_data(data_folder):
    """
    resample the data by the frequency of different steering angles
    this will ensure the distribution are similar for each angle
    """
    
    driving_log_file = ['driving_log.csv', 'driving_log_1.csv', 'driving_log_2.csv']
    # combine all the log files: one from the course and two recorded by myself
    frame = pd.DataFrame()
    list_ = []
    for i in range(len(driving_log_file)):
        
        file_name = data_folder +'/' + driving_log_file[i]
        
        if i==0:
            df = pd.read_csv(file_name,index_col=None, header=0)
        else:
            df = pd.read_csv(file_name,index_col=None, header=1)
        list_.append(df)
        
    frame = pd.concat(list_)
    
    # balance data
    balanced = pd.DataFrame() # Balanced dataset
    bins = 1000 # N of bins
    bin_n = 200 # N of examples to include in each bin (at most)

    start = 0
    for end in np.linspace(0, 1, num=bins):  
        df_range = frame[(np.absolute(frame.steering) >= start) & (np.absolute(frame.steering) < end)]
        range_n = min(bin_n, df_range.shape[0])
        if range_n>0:
            balanced = pd.concat([balanced, df_range.sample(range_n)])
        start = end
    balanced.to_csv(data_folder +'/driving_log_balanced.csv', index=False)
    
    return balanced


def load_img_name(data_folder):
    """
    load the center, right and left image names of the balanced samples
    and corresponding steering angles, 
    ignore the frames that the speed is very slow, close to 0
    """
    
    samples = []
    targets = []
    
    #driving_log_file = ['driving_log.csv']
    driving_log_file = ['driving_log_balanced.csv']
    #driving_log_file = ['driving_log.csv', 'driving_log_1.csv', 'driving_log_2.csv']
    
    for log_name in driving_log_file:
        
        file_name = data_folder +'/' + log_name
        with open(file_name) as csvfile:
            reader = csv.reader(csvfile)
            for line in reader:
                if(float(line[6]) < 0.1):
                    continue
                    
                # center image
                samples.append(line[0].split('/')[-1])
                targets.append(float(line[3]))
                
                # left image
                samples.append(line[1].split('/')[-1])
                targets.append(float(line[3])+angle_offset)
                
                # right image
                samples.append(line[2].split('/')[-1])
                targets.append(float(line[3])-angle_offset)
    
    return samples, targets

def generate_samples(data_folder, data, augment=True):
    """
    build the generator to yield batches of training/validation data.
    Add random shadow and flipping to some randomly selected images.
    Applies data augmentation pipeline if 'augment' is True, 
    but during validation and testing, it should be false.
    """
    
    while True:
        # Generate random batch of indices
        indices = np.random.permutation(data.shape[0])
        batch_size = 128
        for batch in range(0, len(indices), batch_size):
            batch_indices = indices[batch:(batch + batch_size)]
            # Output arrays
            x = np.empty([0, 160, 320, 3], dtype=np.float32)
            y = np.empty([0], dtype=np.float32)
            
            # Read in and preprocess a batch of images
            for i in batch_indices:
                # Randomly select camera and set the angle with corresponding offset
                camera = np.random.randint(len(cameras)) if augment else 1
                image = mpimg.imread(data_folder+data[cameras[camera]].values[i].split('/')[-1])
                angle = data.steering.values[i] + cameras_steering_correction[camera]
                if augment:
                    # Add random shadow as a vertical slice of image
                    h, w = image.shape[0], image.shape[1]
                    [x1, x2] = np.random.choice(w, 2, replace=False)
                    k = h / (x2 - x1)
                    b = - k * x1
                    for i in range(h):
                        c = int((i - b) / k)
                        image[i, :c, :] = (image[i, :c, :] * .5).astype(np.int32)
                
                x = np.append(x, [image], axis=0)
                y = np.append(y, [angle])
            # Randomly flip half of images in the batch
            flip_indices = random.sample(range(x.shape[0]), int(x.shape[0] / 2))
            x[flip_indices] = x[flip_indices, :, ::-1, :]
            y[flip_indices] = -y[flip_indices]
            yield (x, y)
    

def model():
    """
    build the neural network architecture of Nvidia
    """
    
    inputs = Input(shape=(160,320,3))
    
    crop = Cropping2D(cropping=((65,25),(0,0)), input_shape=(160,320,3), name='crop')(inputs)
    norm = Lambda(lambda x: (x / 127.5) - 1, input_shape=(70,320,3), name='normalize')(crop)
    
    conv_1 = Conv2D(24, 5, 5, subsample=(2,2), activation='relu', name='conv_1')(norm)
    conv_2 = Conv2D(36, 5, 5, subsample=(2,2), activation='relu', name='conv_2')(conv_1)
    conv_3 = Conv2D(48, 5, 5, subsample=(2,2), activation='relu', name='conv_3')(conv_2)
    conv_4 = Conv2D(64, 3, 3, subsample=(1,1), activation='relu', name='conv_4')(conv_3)
    conv_5 = Conv2D(64, 3, 3, subsample=(1,1), activation='relu', name='conv_5')(conv_4)
    
    dense_1 = Flatten(name='flatten')(conv_5)
    dense_1 = Dense(100, activation='relu', name='dense_1')(dense_1)
    dense_2 = Dropout(0.5)(dense_1)
    dense_2 = Dense(50, activation='relu', name='dense_2')(dense_2)
    dense_3 = Dropout(0.25)(dense_2)
    dense_3 = Dense(10, activation='relu', name='dense_3')(dense_3)
    
    prediction = Dense(1, activation='linear', name='prediction')(dense_3)
    
    model = Model(input=inputs, output=prediction)
    
    #print(model.summary())
    return model
    
if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Train the driving model.')
    parser.add_argument(
        'data_folder',
        type=str,
        default='',
        help='Path to driving log and image folder. The model will be trained from these images.'
    )
    parser.add_argument(
        'model_folder',
        type=str,
        default='',
        help='Path to model folder. The trained model will be saved in this folder')
    args = parser.parse_args()

    df = balance_data(args.data_folder)
    
    # split the data into train (80%), val (20%)
    df_train, df_val = train_test_split(df, test_size=0.2)
    print("df_train: ", df_train.shape)
    print("df_val: ", df_val.shape)
    
    # compile and train the model using the generator function
    train_generator = generate_samples(args.data_folder+'/IMG/', df_train)
    val_generator = generate_samples(args.data_folder+'/IMG/', df_val, augment=False)
    
    # build the model
    model = model()
    
    # train the model and save the model with best validation loss
    print("Training the model")
    model.compile(loss='mse', optimizer=Adam(lr=1e-4))
    
    model_saved_path = args.model_folder + '/model.h5'
    checkpointer = ModelCheckpoint(filepath=model_saved_path, verbose=1, save_best_only=True)
    history = model.fit_generator(train_generator, samples_per_epoch=df_train.shape[0], verbose=1, 
                        validation_data=val_generator, nb_val_samples=df_val.shape[0], nb_epoch=30, callbacks=[checkpointer])
    
    #model.save('model_saved_path')
    

    

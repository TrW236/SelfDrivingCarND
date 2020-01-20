import csv
import cv2
import numpy as np
from sklearn.utils import shuffle
from sklearn.model_selection import train_test_split

from keras.models import Sequential
from keras.layers import Flatten, Dense, Lambda, Cropping2D, Dropout
from keras.layers import Conv2D
from keras.layers.pooling import MaxPooling2D
from keras.preprocessing.image import img_to_array, load_img

import matplotlib.pyplot as plt

# Constants
CORRECTION = 0.2
VALIDATION_SAMPLES = 0.1
BATCH_SIZE = 16

RIGHT = 1
LEFT = -1
MID = 0

# read info from csv
samples = []
with open('./data/driving_log.csv') as csvfile:
    reader = csv.reader(csvfile)
    for line in reader:
        samples.append(line)

del samples[0]

# split data to train and validation
train_samples, validation_samples = train_test_split(samples, test_size=VALIDATION_SAMPLES)

# calc the steps_epoch_train and steps_epoch_validation
n_samples = len(train_samples)
n_validation = len(validation_samples)

STEPS_EPOCH_TRAIN = n_samples // BATCH_SIZE
STEPS_EPOCH_VALIDATION = n_validation // BATCH_SIZE


def append_imgs_angs(images, angles, imgname, pos, batch_sample):
    corr = None
    if pos == 0:
        corr = 0
    elif pos == 1:  # right
        corr = -CORRECTION
    elif pos == -1:  # left
        corr = CORRECTION
    img = load_img(imgname)
    img = img_to_array(img)

    angle = float(batch_sample[3]) + corr
    images.append(img)

    mirror_img = np.fliplr(img)
    images.append(mirror_img)

    angles.append(angle)

    mirror_angle = -angle
    angles.append(mirror_angle)


# define generator
def generator(samples, batch_size=32):
    num_samples = len(samples)
    while 1:  # Loop forever so the generator never terminates
        shuffle(samples)
        for offset in range(0, num_samples, batch_size):
            batch_samples = samples[offset:offset+batch_size]

            images = []
            angles = []
            for batch_sample in batch_samples:
                name = './data/IMG/'+batch_sample[0].split('/')[-1]
                name_left = './data/IMG/'+batch_sample[1].split('/')[-1]
                name_right = './data/IMG/'+batch_sample[2].split('/')[-1]

                append_imgs_angs(images, angles, name, MID, batch_sample)
                append_imgs_angs(images, angles, name_left, LEFT, batch_sample)
                append_imgs_angs(images, angles, name_right, RIGHT, batch_sample)

            X_train = np.array(images)
            y_train = np.array(angles)
            yield shuffle(X_train, y_train)

# create generators
train_generator = generator(train_samples, batch_size=BATCH_SIZE)
validation_generator = generator(validation_samples, batch_size=BATCH_SIZE)

# network architecture
model = Sequential()
model.add(Lambda(lambda x: x/127.5 - 1., input_shape=(160, 320, 3)))
model.add(Cropping2D(cropping=((70, 25), (0, 0))))
model.add(Conv2D(6, (5, 5), activation='tanh'))
model.add(MaxPooling2D())
model.add(Conv2D(32, (5, 5), activation='tanh'))
model.add(MaxPooling2D())
model.add(Conv2D(48, (5, 5), activation='tanh'))
model.add(MaxPooling2D())

model.add(Flatten())
model.add(Dense(150))
model.add(Dropout(0.2))
model.add(Dense(90))
model.add(Dropout(0.2))
model.add(Dense(40))
model.add(Dropout(0.2))
model.add(Dense(1))

# training
model.compile(loss='mse', optimizer='adam')
history_object = model.fit_generator(train_generator,
                    steps_per_epoch=STEPS_EPOCH_TRAIN,
                    validation_data=validation_generator,
                    validation_steps=STEPS_EPOCH_VALIDATION,
                    epochs=5,
                    verbose=1)

model.save('model.h5')

# plots
print(history_object.history.keys())

plt.plot(history_object.history['loss'])
plt.plot(history_object.history['val_loss'])
plt.title('model mean squared error loss')
plt.ylabel('mean squared error loss')
plt.xlabel('epoch')
plt.legend(['training set', 'validation set'], loc='upper right')
plt.show()

print(0)
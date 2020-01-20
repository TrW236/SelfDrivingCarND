# **Behavioral Cloning** 

## Write-up

---

The goals / steps of this project are the following:
* Use the simulator to collect data of good driving behavior
* Build, a convolution neural network in Keras that predicts steering angles from images
* Train and validate the model with a training and validation set
* Test that the model successfully drives around track one without leaving the road
* Summarize the results with a written report


[//]: # (Image References)

[loss_plot]: ./pics/loss_plot.png "plot loss"

### Usage

#### 1. Train the Model and Save the Model

``model.py`` is the python-file to train the model and after training file ``model.h5`` will be saved as trained model.

#### 2. Test the Trained Model

```
python drive.py model.h5
```

You can use the command above and wait a few seconds until it's ready, then start the simulator with ``autonomous`` mode. The car will drive autonomously.

You can download the simulator in [this repository](https://github.com/udacity/self-driving-car-sim).

#### 3. Create Video of the Driving

```sh
python drive.py model.h5 run1 
```

With the command above you can run the simulator with ``autonomous`` mode and at the same time the pictures will be saved in the folder ``run1``.

```sh
python video.py run1
```

With the command above you can create a video from the pictures in the folder mentioned.


### Model Architecture

#### 1. An appropriate model architecture has been employed

My model consists of three convolution neural network with 5x5 filter sizes and depths with 6, 32 and 48. After the convolution layers there are four fully connected layers with nodes 150, 90, 40 and 1. 

The model includes RELU layers to introduce nonlinearity and the data are normalized in the model using a Keras lambda layer.

The RELU activation function cannot keep the negative sign of the inputs, see the plots blow. But RELU activation function is useful in pattern recognization, because it can keep the value when a similar pattern is detected (the input will be positive and big).

![relu activation function](https://wikimedia.org/api/rest_v1/media/math/render/svg/8d1e78eaf8445e3c1a9d48229abb921a61f30bad)

![relu activation function](https://upload.wikimedia.org/wikipedia/commons/thumb/f/fe/Activation_rectified_linear.svg/120px-Activation_rectified_linear.svg.png)

#### 2. Attempts to reduce overfitting in the model

* The model contains dropout layers in order to reduce overfitting. 

* The model was trained and validated on different data sets to ensure that the model was not overfitting.

* After every convolution layer are max pooling layer used.

* Dataset should be shuffled enough.

* Dataset should be more generalized and should be as many as enough.

#### 3. Model parameter tuning

The model used an adam optimizer, so the learning rate was not tuned manually.

The correction for the angle used for the side images is simply set to 0.2 according to experiences from others.

#### 4. Appropriate training data

Training data was chosen to keep the vehicle driving on the road. 
I used the pictures taken from center, left and right cameras and flip them all to generate another data set in order to augment the training data.

### Training

The overall strategy for deriving a model architecture was to experiment different networks. Firstly I can use complex network to train my model and check if it's overfitting or underfitting. 
* When it's overfitting, I should use some technics like Dropout, Simplifying the Model or Training the model with less epochs, to let the model not be overfitting. 
* When it's underfitting, I should use a more complex network to train the model. 

My first step was to use a relatively complex convolution neural network model. I thought this model might be appropriate because we have many datasets of different pictures.

In order to gauge how well the model was working, I split my image and steering angle data into a training and validation set. I found that my first model had a low mean squared error on the training set but a high mean squared error on the validation set. This implied that the model was overfitting like the plot blow. 

![loss_plot]

To combat the overfitting, I modified the model so that the number of epochs will be set to 5 and I added some Dropout layers into my network.

The final step was to run the simulator to see how well the car was driving around track one. 

### Additional Materials
#### Final Video is Uploaded on Youtube [(link)](https://www.youtube.com/watch?v=ThG_q8SHgas "final video")

The screenshot from the video is below.

[![video_final](https://img.youtube.com/vi/ThG_q8SHgas/0.jpg)]

### Discussion

#### 1. Image Format

Firstly I used the datasets directly from Udacity. But the car drove autonomously off the road. 
Then I drove the car two laps to collect my own datasets and I trained my model based on my own datasets. The problem was still there, that the car drove off the road 
from the right side. Finally I found that the problem was, that I used ``cv2.imread(image_name)`` and this method is loading the image with BGR format, but 
the car reads the picture using ``Keras.preprocessing.image.load_img``, which loads the image with RGB format. Finally I changed the method to load the image and the car drove not off the road
[(link)](https://discussions.udacity.com/t/my-car-remain-on-the-right-side-of-the-lane/247563).
```sh
from keras.preprocessing.image import img_to_array, load_img

image = load_img(image_path)
image = img_to_array(image)
```

#### 2. Data Augmentation
To augment the data sat, I also flipped images and angles and I used the left images and right images with corrected angles. I think this would 
enlarge my datasets, which has positive effect to the training.

#### 3. Data Shuffling
I randomly shuffled the data set and put 10% of the data into a validation set. 
This action would generlize that dataset, which would have positive effect on the test result. 

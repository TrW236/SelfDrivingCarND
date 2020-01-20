# Traffic Sign Recognition

## Other Necessary Files

Please visit this [repository](https://github.com/udacity/CarND-Traffic-Sign-Classifier-Project) from Udacity.

## Data Set Summary & Exploration

### 1. Summary of Training Data

* The shape of the images is (32, 32, 3). 

* The data set for training has 34799 images.

* The data set for validation has 4410 images.

* The data set for the test has 12630 images.

* The number of classes is 43.

The training data can be found on this website. [[link]](http://benchmark.ini.rub.de/?section=gtsrb&subsection=dataset)

### 2. Exploratory visualization of the dataset.

Some examples of the training data is shown below:

<img src="https://filedn.com/lUE8ye7yWpzFOF1OFLVsPau/Github/TrafficSignClassifier/pics/training_data_exploration.png" alt="training data exploration" width="888">

Here is an exploratory visualization of the data set. It is a bar chart showing how is the data distribution. `X-axis` is the labels, and `y-axis` is the number of images of every label.

<img src="https://filedn.com/lUE8ye7yWpzFOF1OFLVsPau/Github/TrafficSignClassifier/pics/data_explore.png" alt="data exploration" width="488">

## Data Preprocessing and Augmentation

### 1. Data preprocessing

* Firstly the images were cropped from size 32x32 to 28x28. 
I think that this process would reduce the influence of the background so that the neural network could focus more on the patterns in the middle of the images.

* Secondly the images were stretched according to their lightness. 
I first converted the images from RGB format to HLS format; then I stretched the L channel to enhance the contrast. 
In the end, I changed the images from HLS back to RGB, so that the color information was preserved.

* Finally I normalized the images so that the range of numbers was changed from [0, 255] to [-1, 1].

Below is an example of the preprocessing.

<img src="https://filedn.com/lUE8ye7yWpzFOF1OFLVsPau/Github/TrafficSignClassifier/pics/preprocess.png" alt="data preprocessing" width="688">

### 2. Data Augmentation

I randomly changed the lightness, rotation and did some shears of the images, so that the training data would be more generalized.

Below is the test of the data augmentation.

<img src="https://filedn.com/lUE8ye7yWpzFOF1OFLVsPau/Github/TrafficSignClassifier/pics/augmentation.png" alt="augmentation" width="888">

The number of training data would be much more.
 
<img src="https://filedn.com/lUE8ye7yWpzFOF1OFLVsPau/Github/TrafficSignClassifier/pics/augmentation_sum.png" alt="augmentation summary" width="488">

## Neural Network

### 1. Architecture of the model

My final model consisted of the following layers:

| Layer                 |     Description                                | 
|:---------------------:|:---------------------------------------------:| 
| Input                 | 28x28x3 RGB image                               | 
| Convolution 5x5         | 1x1 stride, valid padding, outputs 24x24x16    |
| RELU                    |                                                |
| Max pooling              | 2x2 stride,  outputs 12x12x16                 |
| Convolution 5x5        | 1x1 stride, valid padding, outputs 8x8x32     |
| RELU                    |                                                 |
| Max pooling              | 2x2 stride,  outputs 4x4x32                     |
| Convolution 5x5         | 1x1 stride, SAME padding, outputs 4x4x64        |
| RELU                    |                                                 |
| Max pooling              | 2x2 stride,  outputs 2x2x64                     |
| Flatten                 | 256                                             |
| Dropout                 |                                                  |
| Fully Connected         | 128                    |
| RELU                    |                                                 |
| Dropout                 |                                                  |
| Fully Connected         | 84                     |
| RELU                    |                                                 |
| Dropout                 |                                                  |
| Logits         | 43                     |

### 2. Training the model

I used AdamOptimizer with learning rate 0.001, Batch size 128, 
keep percentage 50% for the dropout layer. 
I trained and retrained the model altogether for 20 epochs.

The accuracy of training and validation is shown below.

<img src="https://filedn.com/lUE8ye7yWpzFOF1OFLVsPau/Github/TrafficSignClassifier/pics/training.png" alt="training accuracy" width="488">

### 3. Discussion to reach a high accuracy of validation

My final model results were:

* training set accuracy of 99.3%

* validation set accuracy of 98.5% 

* test set accuracy of 96.5%

The approaches
 
 1. Tried to overfit the training data using more complex neural network
 
 2. Tried to use techniques to reduce the overfitting. These include:

    * Augmented the training data. 
 
    * Used max pooling and dropout layers.
 
    * Shuffled the training data more.

## Test the Model on New Images

### 1. Test the model on test data

Below is some visual exploration of the test data.

<img src="https://filedn.com/lUE8ye7yWpzFOF1OFLVsPau/Github/TrafficSignClassifier/pics/test_data_explore.png" alt="test data exploration" width="688">

<img src="https://filedn.com/lUE8ye7yWpzFOF1OFLVsPau/Github/TrafficSignClassifier/pics/test_data_sum.png" alt="test data summary" width="488">

The total test accuracy was about 96.5%. And I have chosen the first 20 test images to predict. The result is shown below.


<img src="https://filedn.com/lUE8ye7yWpzFOF1OFLVsPau/Github/TrafficSignClassifier/pics/test_examples.png" alt="test data examples" width="688">

### 2. Predict traffic signs found on the web.

Here are some images that I found on the web:

<img src="https://filedn.com/lUE8ye7yWpzFOF1OFLVsPau/Github/TrafficSignClassifier/pics/pics_web.png" alt="test data from web" width="888">

The first three signs were easy to recognize because the neural network was trained for those patterns. 
And the last five patterns were false recognized because that the neural network had not been trained for those patterns.

Interesting was that, the neural network recognized the fifth picture with sure percentage about 98% as 'STOP' sign, 
because of the red color of the pattern and some letters in the pattern and the preprocessing (Scaling) of the image. So the computer saw them as very similar.

The result is shown below.

<img src="https://filedn.com/lUE8ye7yWpzFOF1OFLVsPau/Github/TrafficSignClassifier/pics/test_res.png" alt="test data result from web" width="888">


## References

1. [Training data](http://benchmark.ini.rub.de/?section=gtsrb&subsection=dataset)

2. Udacity Self-Driving Car Engineer Nanodegree
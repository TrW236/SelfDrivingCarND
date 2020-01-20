# Semantic Segmentation

## Dependencies and Usage

* Please visit the [repository](https://github.com/udacity/CarND-Semantic-Segmentation) from Udacity.

## Data Exploration and Augmentation

I used the KITTI training data to train the model, and the training data would be augmented.

The training data as matrixes have the size of `(?, height, width, 3)`, in which the last dimension 3 means the images have three channels respectively RGB. The label images as matrixes have the size of `(?, height, width, 2)`, in which the last dimension 2 means there are two types of objects respectively road and non-road surface.

<img src="https://filedn.com/lUE8ye7yWpzFOF1OFLVsPau/Github/semantic_segmentation/data_augmentation.png" alt="traning data" width="666">

The images above show,

1. The first row is the original image and the corresponding label-image. It is to mention that the white pixels in the label-image are the actual road surface. The black area is the non-road area.

2. The second row is the image, which is flipped horizontally in comparison with the original image.

3. The third row is the original image, in which the light is added by 30.

4. The fourth row is the flipped image, in which the light is added by 30.

5. The fifth row is the original image, in which the light is reduced by 30.

6. The sixth row is the flipped image, in which the light is reduced by 30.

## Neural Network Architecture

<img src="https://filedn.com/lUE8ye7yWpzFOF1OFLVsPau/Github/semantic_segmentation/fcns.png" alt="architecture" width="888">

As the diagram above shows, 

1. The first row in this picture is a part of the VGG16 architecture. 

2. The output of the layer 7 is connected with a `1x1` convolutional layer with the depth (the last dimension of a layer) of 2, which indicates that there are overall two types of objects to be classified, respectively road and non-road.

3. The output of the layer 3 and 4 would be respectively also connected with a `1x1` convolutional layer, whose depth, in other words, the last dimension, was 2.

4. All the three layers mentioned above were upsampled and added together. Finally, the last layer was upsampled, to form a matrix, whose width and height were the same as the original image but the depth is 2.

## Train the Model

Some important parameters for training were set as following:

```
batch_size = 1
learning_rate = 1e-5
epoches = 60
keep_prob = 0.8
```

The training losses are shown below:

<img src="https://filedn.com/lUE8ye7yWpzFOF1OFLVsPau/Github/semantic_segmentation/losses.png" alt="training losses" width="488">

From the diagram above we can see that the losses are convergent good when the epoch is around 45.

## Result

Some images for testing the performance of the model is shown below,

<img src="https://filedn.com/lUE8ye7yWpzFOF1OFLVsPau/Github/semantic_segmentation/tst1.png" alt="test 1" width="488">

<img src="https://filedn.com/lUE8ye7yWpzFOF1OFLVsPau/Github/semantic_segmentation/tst2.png" alt="test 2" width="488">

<img src="https://filedn.com/lUE8ye7yWpzFOF1OFLVsPau/Github/semantic_segmentation/tst3.png" alt="test 3" width="488">

<img src="https://filedn.com/lUE8ye7yWpzFOF1OFLVsPau/Github/semantic_segmentation/tst4.png" alt="test 4" width="488">

<img src="https://filedn.com/lUE8ye7yWpzFOF1OFLVsPau/Github/semantic_segmentation/tst5.png" alt="test 5" width="488">

<img src="https://filedn.com/lUE8ye7yWpzFOF1OFLVsPau/Github/semantic_segmentation/tst6.png" alt="test 6" width="488">

From the images above we can see that the results are satisfying among the testing data.

## References

1. Udacity Nanodegree Self Driving Car Engineer

2. [VGG16 Architecture](https://blog.heuritech.com/2016/02/29/a-brief-report-of-the-heuritech-deep-learning-meetup-5/)

3. Training data [KITTI](http://www.cvlibs.net/datasets/kitti/eval_road.php) and [download](http://www.cvlibs.net/download.php?file=data_road.zip)

4. Other possible training data [cityscapes](https://www.cityscapes-dataset.com/)

5. Jonathan Long, Evan Shelhamer, Trevor Darrell - Fully Convolutional Networks for Semantic Segmentation [[FCNs]](https://people.eecs.berkeley.edu/~jonlong/long_shelhamer_fcn.pdf) - UC Berkeley
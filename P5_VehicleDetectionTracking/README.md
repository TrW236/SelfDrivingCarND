# Vehicle Detection Project

## Other Necessary Files and Dependencies

Please visit the [repository](https://github.com/udacity/CarND-Vehicle-Detection) from Udacity

## Training Data

The training data can be downloaded here [veshicles](https://s3.amazonaws.com/udacity-sdc/Vehicle_Tracking/vehicles.zip)
and [non-vehicles](https://s3.amazonaws.com/udacity-sdc/Vehicle_Tracking/non-vehicles.zip).

### Data Exploration

<img src="https://filedn.com/lUE8ye7yWpzFOF1OFLVsPau/Github/vehicleDetection/data_explore.png" alt="data exploration" width="888">

## Extract features fo the pictures

### 1. Extract spatial bins features

In ``image_processing.py`` the function ``bin_spatial`` is defined to extract spatial bins features from a picture. To notice is that when we
use ``pyplot.imread`` to read a ``png`` image, the scale of the data will be ``[0, 1]``, comparatively to read ``jpg`` we will get a scale ``[0, 255]``. So after the ``png`` images are loaded, they will be
scaled (multiplied by 255) like ``tst_car *= 255``.

The spatial bins features extracted from test picture is below. We can see that all the different color channels will make much difference between car and notcar images.

<img src="https://filedn.com/lUE8ye7yWpzFOF1OFLVsPau/Github/vehicleDetection/spatial_bins.png" alt="examples spatial bins" width="888">

### 2. Extract color histogram features

In ``ìmage_processing.py`` the function ``color_hist`` is defined to extract color features from three channels.

The test result is shown below. We can see that the RGB channels make the most difference between car and notcar images.

<img src="https://filedn.com/lUE8ye7yWpzFOF1OFLVsPau/Github/vehicleDetection/color_hist.png" alt="examples color histogram" width="888">

### 3. Extract HOG features

In ``image_processing.py`` the functions ``get_hog_features`` uses ``skimage.feature.hog`` function to extract
HOG features from one channel of a picture.

I have taken different experiments to determine how the HOG features would be extracted.

Below are the comparisons between HOG images of different channels. We can see that channel YCrCb and HLS make more difference between car and notcar.

<img src="https://filedn.com/lUE8ye7yWpzFOF1OFLVsPau/Github/vehicleDetection/hog1.png" alt="examples HOG different channels" width="888">

Below is the parameter study of HOG features. I finally chose ``orient=9, pix_per_cell=8, cell_per_block=2``.

<img src="https://filedn.com/lUE8ye7yWpzFOF1OFLVsPau/Github/vehicleDetection/hog2.png" alt="examples final HOG" width="888">

## Train the classifier

### 1. Extract all features of a picture

In ``image_processing.py`` the function ``single_img_features`` is defined to extract all the features. The order of the features vector
is by default ``32 x 32 x 3`` spatial bins (RGB channels) + ``32 x 3`` color histogram (RGB channels) + ``7 x 7 x 2 x 2 x 9 x 3`` HOG features (YCrCb channels).

A test result is shown below.

<img src="https://filedn.com/lUE8ye7yWpzFOF1OFLVsPau/Github/vehicleDetection/allfeat_notnorm.png" alt="all features extract" width="888">

The function ``extract_features_imglist`` is defined to extract features vectors from a list of images.

### 2. Normalize the features vectors

In ``classification.py`` in function ``train_classifier`` a scaler from ``sklearn`` will be used to normalize the data.

```
X_scaler = StandardScaler().fit(X)
scaled_X = X_scaler.transform(X)
```

To notice that, ``X`` must be an ``n x m`` array, whereby ``n`` is the number of the datasets and must ``> 1`` here and ``m``
is the number of the features.

A test result is below.

<img src="https://filedn.com/lUE8ye7yWpzFOF1OFLVsPau/Github/vehicleDetection/allfeat_norm.png" alt="normalized all features" width="888">


### 3. Training

In ``classification.py`` the function ``train_classifier`` is defined to train the classifier.

### 4. Prediction

In ``classification.py`` the function ``predict_trained_model`` is defined to predict from the test data.

## Sliding windows

### 1. Extract features in each window

In ``search_cars.py`` two functions ``gen_windows_single_layer`` and ``gen_all_windows`` are defined to realize the sliding windows searching.
Function ``gen_windows_single_layer`` is to generate a row of windows and ``gen_all_windows`` is to generate all windows in the search field.
A function ``search_windows``is defined to search the cars with this method, a test result is below.

A test Result is shown below.

<img src="https://filedn.com/lUE8ye7yWpzFOF1OFLVsPau/Github/vehicleDetection/slide_wins_notsub.png" alt="sliding windows not HOG" width="888">

To mention is that, using this method it will take relatively long time to process the image, in order to get a satisfactory result.

### 2. Using heat map method to get a more precise result

In ``search_cars.py`` function ``add_heat``, ``apply_threshold`` are defined to apply the heatmap method.
A test result is below.

<img src="https://filedn.com/lUE8ye7yWpzFOF1OFLVsPau/Github/vehicleDetection/heatmap_filtered.png" alt="Heatmap" width="888">

### 3. Extract HOG features once in one picture

Because the method above will take a lot of time to compute, the HOG sub-sampling method is used.
In ``search_cars.py`` the function ``find_cars`` is defined.

A test result is shown below.

<img src="https://filedn.com/lUE8ye7yWpzFOF1OFLVsPau/Github/vehicleDetection/hogsub.png" alt="sub HOG 1" width="888">

<img src="https://filedn.com/lUE8ye7yWpzFOF1OFLVsPau/Github/vehicleDetection/hogsub2.png" alt="sub HOG 2" width="888">

## Pipeline for Video Processing

A test result for sliding windows searching extracting features in each window is below. 
We can see that the result is not satisfactory and the parameters for this method should be tuned more.

<img src="https://filedn.com/lUE8ye7yWpzFOF1OFLVsPau/Github/vehicleDetection/slide_wins_notsub_test.png" alt="sliding windows not sub HOG all imgs" width="888">

A test result for sub HOG sampling is below. We can see that the result is satisfactory and the processing time is much fewer.

<img src="https://filedn.com/lUE8ye7yWpzFOF1OFLVsPau/Github/vehicleDetection/hogsub_test.png" alt="sliding windows sub HOG all imgs" width="888">

I used the HOG sub-sampling method to process the video.
 
In ``process_pipeline.py`` the class ``Pipeline_subsamp`` is defined to perform a full search method. ``img_process``
and method ``img_process_small`` is to perform the small search.

Class ``Organizer`` is defined to control the video processing. It is responsible for deciding to use small searching or full searching method.


## Result and Final Effect

The result is ``result.mp4`` and uploaded on Youtube. [[link]](https://www.youtube.com/watch?v=awsRmzZ1h6k&index=2&list=PLNDTbGbATLcED0iX8K-zY3vrNbwhxV8gC)

## Discussion

The algorithm is not very stable; it will sometimes detect non-cars and lose the detections of the real cars. The improvements can be:

* To extract features of samples should be studied for more efficient detection. For example which color spaces should be used.

* Using more complex learning algorithm and more data to train the classifier.

* Many hyperparameters should be better tuned.

* A more stable algorithm should be found to smooth the searching results between the frames.

## References

1. Udacity Self Driving Car Engineer Nanodegree

2. [Sklearn Documentation](http://scikit-learn.org/stable/)

3. Training data [GTI vehicle image database](http://www.gti.ssr.upm.es/data/Vehicle_database.html), [KITTI vision benchmark suite](http://www.cvlibs.net/datasets/kitti/), [Udacity labeled dataset](https://github.com/udacity/self-driving-car/tree/master/annotations)
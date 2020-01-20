from sklearn.svm import LinearSVC
from sklearn.preprocessing import StandardScaler
from image_processing import *
from sklearn.model_selection import train_test_split

import time
import numpy as np


def train_classifier(cars, notcars, color_space='RGB',   # imgs
                     spatial_size=(32, 32),  # for spatial histogram
                     hist_bins=32, bins_range=(0, 256),  # for color histogram
                     orient=9, pix_per_cell=8, cell_per_block=2, hog_channel='ALL',  # for Hog feature
                     spatial_feat=True, hist_feat=True, hog_feat=True):  # flags
    t = time.time()
    car_features = extract_features_imglist(
        cars, color_space=color_space,
        spatial_size=spatial_size,
        hist_bins=hist_bins, bins_range=bins_range,
        orient=orient, pix_per_cell=pix_per_cell, cell_per_block=cell_per_block, hog_channel=hog_channel,
        spatial_feat=spatial_feat, hist_feat=hist_feat, hog_feat=hog_feat)

    notcar_features = extract_features_imglist(
        notcars, color_space=color_space,
        spatial_size=spatial_size,
        hist_bins=hist_bins, bins_range=bins_range,
        orient=orient, pix_per_cell=pix_per_cell, cell_per_block=cell_per_block, hog_channel=hog_channel,
        spatial_feat=spatial_feat, hist_feat=hist_feat, hog_feat=hog_feat)

    t2 = time.time()
    print(round(t2-t, 2), 'Seconds to extract HOG features...')
    # Create an array stack of feature vectors
    X = np.vstack((car_features, notcar_features)).astype(np.float64)
    print("Total number of cars and notcars: ", len(X))
    print("Number of features: ", len(X[0]))
    # Fit a per-column scaler
    X_scaler = StandardScaler().fit(X)
    # Apply the scaler to X
    scaled_X = X_scaler.transform(X)

    # Define the labels vector
    y = np.hstack((np.ones(len(car_features)), np.zeros(len(notcar_features))))

    # Split up data into randomized training and test sets
    rand_state = np.random.randint(0, 100)
    X_train, X_test, y_train, y_test = train_test_split(
        scaled_X, y, test_size=0.2, random_state=rand_state)

    print('Using:', orient,'orientations', pix_per_cell,
        'pixels per cell and',  cell_per_block, 'cells per block')
    print('Feature vector length:', len(X_train[0]))
    # Use a linear SVC
    svc = LinearSVC()

    # Check the training time for the SVC
    t = time.time()
    svc.fit(X_train, y_train)
    t2 = time.time()
    print(round(t2-t, 2), 'Seconds to train SVC...')
    # Check the score of the SVC
    print('Test Accuracy of SVC = ', round(svc.score(X_test, y_test), 4))
    return svc, X_scaler, (X_train, X_test, y_train, y_test)


def predict_trained_model(svc, X_test, y_test, idx=None):
    # Check the prediction time for a single sample
    t = time.time()
    if idx is None:
        idx = list(range(0, len(X_test)))
    print(svc.predict(X_test[idx]), ' -> Predictions')
    print(y_test[idx], ' -> Labels')
    print(idx, ' -> Indices')
    t2 = time.time()
    print(round(t2 - t, 5), 'Seconds to predict', idx, 'labels with SVC')
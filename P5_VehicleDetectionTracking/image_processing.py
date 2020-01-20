from skimage.feature import hog
import cv2
import matplotlib.image as mpimg
import numpy as np


def convert_color(img, conv='RGB2YCrCb'):
    if conv == 'RGB2YCrCb':
        return cv2.cvtColor(img, cv2.COLOR_RGB2YCrCb)
    # if conv == 'BGR2YCrCb':
    #     return cv2.cvtColor(img, cv2.COLOR_BGR2YCrCb)
    if conv == 'RGB2LUV':
        return cv2.cvtColor(img, cv2.COLOR_RGB2LUV)
    if conv == 'RGB2HLS':
        return cv2.cvtColor(img, cv2.COLOR_RGB2HLS)
    # if conv == 'RGB2HSV':
    #     return cv2.cvtColor(img, cv2.COLOR_RGB2HSV)


def get_hog_features(img, orient, pix_per_cell, cell_per_block, visualise=False, feature_vec=True):
    # Call with two outputs if visualise==True
    if visualise:
        features, hog_image = hog(img, orientations=orient, pixels_per_cell=(pix_per_cell, pix_per_cell),
                                  cells_per_block=(cell_per_block, cell_per_block), transform_sqrt=True,
                                  visualise=visualise, feature_vector=feature_vec)
        return features, hog_image
    # Otherwise call with one output
    else:
        features = hog(img, orientations=orient, pixels_per_cell=(pix_per_cell, pix_per_cell),
                       cells_per_block=(cell_per_block, cell_per_block), transform_sqrt=True,
                       visualise=visualise, feature_vector=feature_vec)
        return features


# Define a function to extract features from a list of images
def extract_features_imglist(images, color_space='RGB',
                             spatial_size=(32, 32),
                             hist_bins=32,  bins_range=(0, 256),
                             orient=9, pix_per_cell=8, cell_per_block=2, hog_channel='ALL',
                             spatial_feat=True, hist_feat=True, hog_feat=True):
    # Create a list to append feature vectors to
    features = []
    # Iterate through the list of images
    for file in images:
        image = mpimg.imread(file)
        image = (image*255).astype(np.uint8)
        file_features = single_img_features(
            image, color_space=color_space,
            spatial_size=spatial_size,
            hist_bins=hist_bins, bins_range=bins_range,
            orient=orient, pix_per_cell=pix_per_cell, cell_per_block=cell_per_block, hog_channel=hog_channel,
            spatial_feat=spatial_feat, hist_feat=hist_feat, hog_feat=hog_feat)
        features.append(file_features)
    # Return list of feature vectors
    return features


def single_img_features(img, color_space='RBG',
                        spatial_size=(32, 32),
                        hist_bins=32, bins_range=(0, 256),
                        orient=9, pix_per_cell=8, cell_per_block=2, hog_channel='ALL',
                        spatial_feat=True, hist_feat=True, hog_feat=True):
    # 1) Define an empty list to receive features
    img_features = []
    # 2) Apply color conversion if other than 'RGB'
    # if color_space != 'RGB':
    #     if color_space == 'HSV':
    #         feature_image = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
    #     elif color_space == 'LUV':
    #         feature_image = cv2.cvtColor(img, cv2.COLOR_RGB2LUV)
    #     elif color_space == 'HLS':
    #         feature_image = cv2.cvtColor(img, cv2.COLOR_RGB2HLS)
    #     elif color_space == 'YUV':
    #         feature_image = cv2.cvtColor(img, cv2.COLOR_RGB2YUV)
    #     elif color_space == 'YCrCb':
    #         feature_image = cv2.cvtColor(img, cv2.COLOR_RGB2YCrCb)
    # else:
    feature_image = np.copy(img)
    # 3) Compute spatial features if flag is set
    if spatial_feat:
        spatial_features = bin_spatial(feature_image, size=spatial_size)
        # 4) Append features to list
        img_features.append(spatial_features)
    # 5) Compute histogram features if flag is set
    if hist_feat:
        hist_features = color_hist(feature_image, hist_bins=hist_bins, bins_range=bins_range)
        # 6) Append features to list
        img_features.append(hist_features)
    # 7) Compute HOG features if flag is set
    ycc = convert_color(feature_image)  # convert to ycrcb
    if hog_feat:
        if hog_channel == 'ALL':
            hog_features = []
            for channel in range(ycc.shape[2]):
                hog_features.extend(get_hog_features(ycc[:, :, channel],
                                                     orient, pix_per_cell, cell_per_block,
                                                     visualise=False, feature_vec=True))
        else:
            hog_features = get_hog_features(ycc[:, :, hog_channel], orient,
                                            pix_per_cell, cell_per_block, visualise=False, feature_vec=True)
        # 8) Append features to list
        img_features.append(hog_features)

    # 9) Return concatenated array of features
    return np.concatenate(img_features)


# Define a function to compute binned color features
def bin_spatial(img, size=(32, 32)):
    # imcopy = np.copy(img)
    # if scale:
    #     imcopy /= 255
    # Use cv2.resize().ravel() to create the feature vector
    features = cv2.resize(img, size).ravel()
    # Return the feature vector
    return features


# Define a function to compute color histogram features
# NEED TO CHANGE bins_range if reading .png files with mpimg!
def color_hist(img, hist_bins=32, bins_range=(0, 256)):
    # Compute the histogram of the color channels separately
    channel1_hist = np.histogram(img[:, :, 0], bins=hist_bins, range=bins_range)
    channel2_hist = np.histogram(img[:, :, 1], bins=hist_bins, range=bins_range)
    channel3_hist = np.histogram(img[:, :, 2], bins=hist_bins, range=bins_range)
    # Concatenate the histograms into a single feature vector
    hist_features = np.concatenate((channel1_hist[0], channel2_hist[0], channel3_hist[0]))
    # Return the individual histograms, bin_centers and feature vector
    return hist_features

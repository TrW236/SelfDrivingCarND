import numpy as np
import cv2
from image_processing import*
import math
from utils import *


# Define a function to draw bounding boxes
def draw_boxes(img, bboxes, color=(0, 0, 255), thick=6):
    # Make a copy of the image
    imcopy = np.copy(img)
    # Iterate through the bounding boxes
    for bbox in bboxes:
        # Draw a rectangle given bbox coordinates
        cv2.rectangle(imcopy, bbox[0], bbox[1], color, thick)
    # Return the image copy with boxes drawn
    return imcopy


# Define a function you will pass an image
# and the list of windows to be searched (output of slide_windows())
def search_windows(img, windows, clf, scaler,
                   color_space='RGB',
                   spatial_size=(32, 32),
                   hist_bins=32, bins_range=(0, 256),
                   orient=9, pix_per_cell=8, cell_per_block=2, hog_channel='ALL',
                   spatial_feat=True, hist_feat=True, hog_feat=True):

    # 1) Create an empty list to receive positive detection windows
    on_windows = []
    # 2) Iterate over all windows in the list
    for window in windows:
        # 3) Extract the test window from original image
        test_img = cv2.resize(img[window[0][1]:window[1][1], window[0][0]:window[1][0]], (64, 64))  # resize is important
        # 4) Extract features for that window using single_img_features()
        features = single_img_features(test_img, color_space=color_space,
                                       spatial_size=spatial_size,
                                       hist_bins=hist_bins, bins_range=bins_range,
                                       orient=orient, pix_per_cell=pix_per_cell, cell_per_block=cell_per_block, hog_channel=hog_channel,
                                       spatial_feat=spatial_feat, hist_feat=hist_feat, hog_feat=hog_feat)
        # 5) Scale extracted features to be fed to classifier
        test_features = scaler.transform(np.array(features).reshape(1, -1))
        # 6) Predict using your classifier
        prediction = clf.predict(test_features)
        # 7) If positive (prediction == 1) then save the window
        if prediction == 1:
            on_windows.append(window)
    # 8) Return windows for positive detections
    return on_windows


def gen_all_windows(img, width_expand=2, n_layers=5, vanish_point=None, overlap=0.5, base_win_w=None):
    if vanish_point is None:
        vanish_point=[img.shape[1]/2, img.shape[0]/2]  # in the middle of the img

    x_bot_start = - img.shape[1] * width_expand
    x_bot_end = img.shape[1] * (width_expand + 1)

    base_rate = 1.0 / float(n_layers+2)
    if base_win_w is None:
        base_win_w = img.shape[1] * 0.35

    all_windows = []

    for i in range(n_layers):
        rate = base_rate*(i+1.0)
        y_end = val_lin(vanish_point[1], img.shape[0], rate)
        x_start = max(val_lin(vanish_point[0], x_bot_start, rate), 0)
        x_end = min(val_lin(vanish_point[0], x_bot_end, rate), img.shape[1])
        win_w = base_win_w * rate
        wins_layer = gen_windows_single_layer(win_w, x_start, x_end, y_end, overlap)
        all_windows += wins_layer
    return all_windows


def gen_windows_single_layer(win_h, x_start, x_end, y_end, overlap=0.5):
    windows = []
    x_span = x_end - x_start
    w_step = win_h * (1.0-overlap)
    n_layer = int(math.ceil((x_span-win_h) / w_step))
    assert n_layer > 0, "No Windows generated because of no space."
    starty = int(y_end - win_h)
    endy = int(y_end)
    for i in range(n_layer):
        startx = int(x_start + i*w_step)
        endx = int(startx + win_h)
        windows.append(((startx, starty), (endx, endy)))
    return windows


def add_heat(heatmap, bbox_list):
    # Iterate through list of bboxes
    for box in bbox_list:
        # Add += 1 for all pixels inside each bbox
        # Assuming each "box" takes the form ((x1, y1), (x2, y2))
        heatmap[box[0][1]:box[1][1], box[0][0]:box[1][0]] += 1

    # Return updated heatmap
    return heatmap


def apply_threshold(heatmap, threshold):
    # Zero out pixels below the threshold
    heatmap_filtered = np.copy(heatmap)
    heatmap_filtered[heatmap <= threshold] = 0
    # Return thresholded map
    return heatmap_filtered


def draw_labeled_bboxes(img, labels, valid_idx=None):
    img_copy = np.copy(img)
    # Iterate through all detected cars
    if valid_idx is None:
        for car_number in range(1, labels[1] + 1):
            __draw_boxes(car_number, img_copy, labels)
    else:
        for car_number in valid_idx:
            __draw_boxes(car_number, img_copy, labels)
    # Return the image
    return img_copy


def __draw_boxes(car_number, img_copy, labels):
    # Find pixels with each car_number label value
    nonzero = (labels[0] == car_number).nonzero()
    # Identify x and y values of those pixels
    nonzeroy = np.array(nonzero[0])
    nonzerox = np.array(nonzero[1])
    # Define a bounding box based on min/max x and y
    bbox = ((np.min(nonzerox), np.min(nonzeroy)), (np.max(nonzerox), np.max(nonzeroy)))
    # Draw the box on the image
    cv2.rectangle(img_copy, bbox[0], bbox[1], (0, 0, 255), 6)


# Define a single function that can extract features using hog sub-sampling and make predictions
def find_cars(img, svc, X_scaler, window=64, cells_per_step=2,
              orient=9, pix_per_cell=8, cell_per_block=2, bins_range=(0, 256), spatial_size=(32,32), hist_bins=32,
              ystart=400, ystop=656, scale=1.5,xstart=None, xend=None):
    # draw_img = np.copy(img)
    # img = img.astype(np.float32) / 255  # normalize
    if xstart is None:
        xstart = 0
    if xend is None:
        xend = img.shape[1]

    img_tosearch = img[ystart:ystop, xstart:xend, :]
    # ctrans_tosearch = convert_color(img_tosearch, conv='RGB2YCrCb')
    if scale != 1:
        imshape = img_tosearch.shape
        img_tosearch = cv2.resize(img_tosearch, (np.int(imshape[1] / scale), np.int(imshape[0] / scale)))

    ch1 = img_tosearch[:, :, 0]
    ch2 = img_tosearch[:, :, 1]
    ch3 = img_tosearch[:, :, 2]

    # Define blocks and steps as above
    nxcells = (ch1.shape[1] // pix_per_cell)  # - cell_per_block + 1
    nycells = (ch1.shape[0] // pix_per_cell)  # - cell_per_block + 1

    # nfeat_per_block = orient * cell_per_block ** 2

    # 64 was the orginal sampling rate, with 8 cells and 8 pix per cell
    ncells_per_window = (window // pix_per_cell) - cell_per_block + 1

    # Instead of overlap, define how many cells to step
    nxsteps = (nxcells - ncells_per_window) // cells_per_step
    nysteps = (nycells - ncells_per_window) // cells_per_step

    ycc = convert_color(img_tosearch)
    # Compute individual channel HOG features for the entire image
    hog1 = get_hog_features(ycc[:, :, 0], orient, pix_per_cell, cell_per_block, feature_vec=False)
    hog2 = get_hog_features(ycc[:, :, 1], orient, pix_per_cell, cell_per_block, feature_vec=False)
    hog3 = get_hog_features(ycc[:, :, 2], orient, pix_per_cell, cell_per_block, feature_vec=False)

    on_wins =[]
    for xb in range(nxsteps):
        for yb in range(nysteps):
            ypos = yb * cells_per_step
            xpos = xb * cells_per_step
            # Extract HOG for this patch
            hog_feat1 = hog1[ypos:ypos + ncells_per_window, xpos:xpos + ncells_per_window].ravel()
            hog_feat2 = hog2[ypos:ypos + ncells_per_window, xpos:xpos + ncells_per_window].ravel()
            hog_feat3 = hog3[ypos:ypos + ncells_per_window, xpos:xpos + ncells_per_window].ravel()
            hog_features = np.hstack((hog_feat1, hog_feat2, hog_feat3))

            xleft = xpos * pix_per_cell
            ytop = ypos * pix_per_cell

            # Extract the image patch
            subimg = cv2.resize(img_tosearch[ytop:ytop + window, xleft:xleft + window], (64, 64))

            # Get color features
            spatial_features = bin_spatial(subimg, size=spatial_size)
            hist_features = color_hist(subimg, hist_bins=hist_bins, bins_range=bins_range)

            # Scale features and make a prediction
            feat_all = np.hstack((spatial_features, hist_features, hog_features))
            feat_all = feat_all.reshape((1, -1))
            test_features = X_scaler.transform(feat_all)  # .reshape(1, -1)
            # test_features = X_scaler.transform(np.hstack((shape_feat, hist_feat)).reshape(1, -1))
            test_prediction = svc.predict(test_features)

            if test_prediction == 1:
                xbox_left = np.int(xleft * scale)
                ytop_draw = np.int(ytop * scale)
                win_draw = np.int(window * scale)
                # cv2.rectangle(draw_img, (xbox_left, ytop_draw + ystart),
                #               (xbox_left + win_draw, ytop_draw + win_draw + ystart), (0, 0, 255), 6)
                on_wins.append(((xbox_left+xstart, ytop_draw+ystart), (xbox_left+win_draw+xstart, ytop_draw+win_draw+ystart)))
    # return draw_img
    return on_wins





# Define a function that takes an image,
# start and stop positions in both x and y,
# window size (x and y dimensions),
# and overlap fraction (for both x and y)
# def slide_window(img, x_start_stop=None, y_start_stop=None,
#                  xy_window=(64, 64), xy_overlap=(0.5, 0.5)):
#     if y_start_stop is None:
#         y_start_stop = [None, None]
#     if x_start_stop is None:
#         x_start_stop = [None, None]
#
#     # If x and/or y start/stop positions not defined, set to image size
#     if x_start_stop[0] is None:
#         x_start_stop[0] = 0
#     if x_start_stop[1] is None:
#         x_start_stop[1] = img.shape[1]
#     if y_start_stop[0] is None:
#         y_start_stop[0] = 0
#     if y_start_stop[1] is None:
#         y_start_stop[1] = img.shape[0]
#
#     # Compute the span of the region to be searched
#     xspan = x_start_stop[1] - x_start_stop[0]
#     yspan = y_start_stop[1] - y_start_stop[0]
#
#     # Compute the number of pixels per step in x/y
#     nx_pix_per_step = np.int(xy_window[0] * (1 - xy_overlap[0]))
#     ny_pix_per_step = np.int(xy_window[1] * (1 - xy_overlap[1]))
#
#     # Compute the number of windows in x/y
#     nx_buffer = np.int(xy_window[0] * (xy_overlap[0]))
#     ny_buffer = np.int(xy_window[1] * (xy_overlap[1]))
#     nx_windows = np.int((xspan - nx_buffer) / nx_pix_per_step)
#     ny_windows = np.int((yspan - ny_buffer) / ny_pix_per_step)
#
#     # Initialize a list to append window positions to
#     window_list = []
#     # Loop through finding x and y window positions
#     # Note: you could vectorize this step, but in practice
#     # you'll be considering windows one by one with your
#     # classifier, so looping makes sense
#     for ys in range(ny_windows):
#         for xs in range(nx_windows):
#             # Calculate window position
#             startx = xs * nx_pix_per_step + x_start_stop[0]
#             endx = startx + xy_window[0]
#             starty = ys * ny_pix_per_step + y_start_stop[0]
#             endy = starty + xy_window[1]
#
#             # Append window position to list
#             window_list.append(((startx, starty), (endx, endy)))
#     # Return the list of windows
#     return window_list



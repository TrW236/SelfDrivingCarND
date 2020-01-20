from search_cars import *
from scipy.ndimage.measurements import label
from copy import deepcopy


class Pipeline:
    def __init__(self, vanish_p, overlap, clf, scaler, threshold, n_layers, w_expand, exp_labels=False):
        self.vanish_p = vanish_p
        self.overlap = overlap
        self.clf = clf
        self.scaler = scaler
        self.threshold = threshold
        self.n_layers = n_layers
        self.width_expand = w_expand
        self.expLabels = exp_labels

    def img_process(self, img):
        wins_all = gen_all_windows(img, vanish_point=self.vanish_p, overlap=self.overlap, n_layers=self.n_layers, width_expand=self.width_expand)
        on_wins = search_windows(img, wins_all, self.clf, self.scaler)  # bins_range = (0, 256) because the img is .jpg
        heatmap = np.zeros_like(img[:, :, 0]).astype(np.float)
        heatmap = add_heat(heatmap, on_wins)
        heatmap_filtered = apply_threshold(heatmap, self.threshold)
        labels = label(heatmap_filtered)
        # pdb.set_trace()

        if self.expLabels:
            return labels

        else:
            imcopy = draw_labeled_bboxes(img, labels)
            return imcopy

import pdb
class Pipeline_subsamp:
    def __init__(self, clf, scaler, threshold, cells_per_step, window=64, scale=1.5, exp_labels=False):
        self.clf = clf
        self.scaler = scaler
        self.threshold = threshold
        self.cells_per_step=cells_per_step
        self.expLabels = exp_labels
        self.window = window
        self.scale = scale

    def img_process(self, img):
        on_wins = find_cars(img, self.clf, self.scaler, ystart=400, ystop=528,
                            window=self.window, cells_per_step=1, scale=1.5)
        on_wins += find_cars(img, self.clf, self.scaler, ystart=528, ystop=656,
                            window=self.window, cells_per_step=1, scale=2)
        heatmap = np.zeros_like(img[:, :, 0]).astype(np.float)
        heatmap = add_heat(heatmap, on_wins)
        heatmap_filtered = apply_threshold(heatmap, self.threshold)
        labels = label(heatmap_filtered)
        # pdb.set_trace()

        if self.expLabels:
            return labels
        else:
            imcopy = draw_labeled_bboxes(img, labels)
            return imcopy

    def img_process_small(self, img, labels, offset=10):
        on_wins =[]
        for car_number in range(1, labels[1] + 1):
            nonzero = (labels[0] == car_number).nonzero()
            nonzeroy = np.array(nonzero[0])
            nonzerox = np.array(nonzero[1])
            xstart = max(np.min(nonzerox)-offset, 0)
            xend = min(np.max(nonzerox)+offset, img.shape[1]-1)
            ystart = max(np.min(nonzeroy)-offset, 0)
            yend = min(np.max(nonzeroy)+offset,img.shape[0]-1)
            # bbox = ((np.min(nonzerox), np.min(nonzeroy)), (np.max(nonzerox), np.max(nonzeroy)))
            on_wins += find_cars(img, self.clf, self.scaler, ystart=ystart, ystop=yend, xstart=xstart, xend=xend,
                                window=self.window, cells_per_step=1, scale=1.5)
        # pdb.set_trace()
        heatmap = np.zeros_like(img[:, :, 0]).astype(np.float)
        heatmap = add_heat(heatmap, on_wins)
        heatmap_filtered = apply_threshold(heatmap, self.threshold)
        newlabels = label(heatmap_filtered)
        return newlabels


class Organizer:
    def __init__(self, pipeline, offset=10):
        self.pipeline = pipeline
        self.pipeline.expLabels = True
        self.preLabels = None
        self.count = 0
        self.offset = offset

    def img_process(self, img):
        if self.count % 1 == 0:
            # print(self.count)
            self.count = 0
            cur_labels = self.pipeline.img_process(img)
            if self.preLabels is None:  # Video start
                self.preLabels = cur_labels
            else:
                # if np.absolute(cur_labels[1] - self.preLabels[1]) >= 2:  # or (cur_labels[1] > 3):
                #     pass
                # else:
                self.preLabels = cur_labels
        else:  # small search
            cur_labels = self.pipeline.img_process_small(img, self.preLabels, self.offset)
            # pdb.set_trace()
            # if np.absolute(cur_labels[1] - self.preLabels[1]) >= 2:  # or (cur_labels[1] > 3):
            #     pass
            # else:
            self.preLabels = cur_labels
        imcopy = draw_labeled_bboxes(img, self.preLabels)
        self.count += 1
        return imcopy

# import pickle
# with open('mymodel.p', 'rb') as f:
#     svc, scaler, data = pickle.load(f)
# threshold = 2
# cells_per_step = 1
# window = 64
# scale = 1.5
# p_subsam = Pipeline_subsamp(svc, scaler, threshold, cells_per_step=cells_per_step, window=window, scale=scale)
#
# p_subsam.expLabels=True
# raw_img = mpimg.imread('test_images/test3.jpg')
# labels = p_subsam.img_process(raw_img)
# newlabels = p_subsam.img_process_small(raw_img, labels)


# class LabelsBlock:
#     def __init__(self):
#         self.labels = None
#         self.nxtBlock = None
#
#     def set_labels(self, labels):
#         self.labels = labels
#
#     def set_next(self, labels_block):
#         self.nxtBlock = labels_block
#
# class LinkedLables:


# class Organizer:
#     def __init__(self, pipeline):
#         self.pipeLine = pipeline
#         self.pipeLine.expLabels = True
#         self.preLabels = None
#         self.preBools = None
#         # self.comLabels = None
#
#     def img_process(self, img):
#         cur_labels = self.pipeLine.img_process(img)
#         cur_bools = cur_labels[0].astype(np.bool)
#
#         if not np.any(cur_bools):  # not detection draw pre valid boxes
#             if self.preLabels is not None:
#                 return draw_labeled_bboxes(img, self.preLabels)
#             else:
#                 return img
#
#         if self.preLabels is not None:
#             valid_idx = []
#             for i in range(cur_labels[1]):
#                 idx = i+1
#                 bools_idx = cur_labels[0] == idx  # bool array
#                 points = np.argwhere(bools_idx)
#                 if np.max(points, axis=0)[1] >= img.shape[1]-10:  # 10 is the tolerant
#                     valid_idx.append(idx)
#                 else:  # com with the pre labels
#
#                     cur_bools = cur_labels[0].astype(np.bool)
#                     if np.any(np.logical_and(bools_idx, np.logical_and(cur_bools, self.preBools))):
#                         valid_idx.append(idx)
#             if len(valid_idx) == 0:  # no valid
#                 if self.preLabels is not None:
#                     return draw_labeled_bboxes(img, self.preLabels)
#                 else:
#                     return img
#             else:  # valid_idx
#                 draw_labeled_bboxes(img, cur_labels, valid_idx)
#                 self.preLabels = cur_labels
#                 self.preBools = cur_bools
#         else:
#             self.preLabels = cur_labels
#             self.preBools = cur_bools
#             # self.comLabels = cur_labels
#         if self.preLabels is not None:
#             return draw_labeled_bboxes(img, self.preLabels)
#         else:
#             return img
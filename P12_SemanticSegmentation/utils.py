import re
import random
import numpy as np
import os.path
import scipy.misc
import shutil
import time
import tensorflow as tf
from glob import glob


def gen_batch_function(data_folder, image_shape):
    """
    Generate function to create batches of training data
    :param data_folder: Path to folder that contains all the datasets
    :param image_shape: Tuple - Shape of image
    :return:
    """
    def get_batches_fn(batch_size):
        """
        Create batches of training data
        :param batch_size: Batch Size
        :return: Batches of training data
        """
        image_paths = glob(os.path.join(data_folder, 'image_2', '*.png'))
        label_paths = {
            re.sub(r'_(lane|road)_', '_', os.path.basename(path)): path
            for path in glob(os.path.join(data_folder, 'gt_image_2', '*_road_*.png'))}
        background_color = np.array([255, 0, 0])

        random.shuffle(image_paths)
        for batch_i in range(0, len(image_paths), batch_size):
            images = []
            gt_images = []
            for image_file in image_paths[batch_i:batch_i+batch_size]:
                gt_image_file = label_paths[os.path.basename(image_file)]

                image = scipy.misc.imresize(scipy.misc.imread(image_file), image_shape)
                gt_image = scipy.misc.imresize(scipy.misc.imread(gt_image_file), image_shape)

                gt_bg = np.all(gt_image == background_color, axis=2)
                gt_bg = gt_bg.reshape(*gt_bg.shape, 1)
                gt_image = np.concatenate((gt_bg, np.invert(gt_bg)), axis=2)

                images.append(image)
                gt_images.append(gt_image)

            yield np.array(images), np.array(gt_images)

    return get_batches_fn


def save_inference_samples(runs_dir, data_dir, sess, image_shape, logits, keep_prob, input_image):
    # Make folder for current run
    output_dir = os.path.join(runs_dir, str(time.time()))
    if os.path.exists(output_dir):
        shutil.rmtree(output_dir)
    os.makedirs(output_dir)

    # Run NN on test images and save them to HD
    print('Training Finished. Saving test images to: {}'.format(output_dir))
    image_outputs = gen_test_output(
        sess, logits, keep_prob, input_image, os.path.join(data_dir, 'data_road/testing'), image_shape)
    for name, image in image_outputs:
        scipy.misc.imsave(os.path.join(output_dir, name), image)


def gen_test_output(sess, logits, keep_prob, image_pl, data_folder, image_shape):
    """
    Generate test output using the test images
    :param sess: TF session
    :param logits: TF Tensor for the logits
    :param keep_prob: TF Placeholder for the dropout keep robability
    :param image_pl: TF Placeholder for the image placeholder
    :param data_folder: Path to the folder that contains the datasets
    :param image_shape: Tuple - Shape of image
    :return: Output for for each test image
    """
    for image_file in glob(os.path.join(data_folder, 'image_2', '*.png')):
        image = scipy.misc.imresize(scipy.misc.imread(image_file), image_shape)

        im_softmax = sess.run(
            [tf.nn.softmax(logits)],
            {keep_prob: 1.0, image_pl: [image]})
        im_softmax = im_softmax[0][:, 1].reshape(image_shape[0], image_shape[1])
        segmentation = (im_softmax > 0.5).reshape(image_shape[0], image_shape[1], 1)
        mask = np.dot(segmentation, np.array([[0, 255, 0, 127]]))
        mask = scipy.misc.toimage(mask, mode="RGBA")
        street_im = scipy.misc.toimage(image)
        street_im.paste(mask, box=None, mask=mask)

        yield os.path.basename(image_file), np.array(street_im)


def add_light(img, light):  # [0, 255]
    imcopy=np.copy(img)
    imcopy = imcopy.astype(np.float32) + light
    imcopy = np.maximum(imcopy, 0)
    imcopy = np.minimum(imcopy, 255)
    return imcopy.astype(np.uint8)


def augment_data(imgs, labels, light=30):
    res_imgs = []
    res_labels = []
    for img, label in zip(imgs, labels):
        # original
        res_imgs.append(img)
        res_labels.append(label)
        # flip
        res_imgs.append(np.fliplr(img))
        res_labels.append(np.fliplr(label))
        # add light orig
        res_imgs.append(add_light(img, light))
        res_labels.append(label)
        # add light flip
        res_imgs.append(add_light(np.fliplr(img), light))
        res_labels.append(np.fliplr(label))
        # reduce light orig
        res_imgs.append(add_light(img, -light))
        res_labels.append(label)
        # reduce light flip
        res_imgs.append(add_light(np.fliplr(img), -light))
        res_labels.append(np.fliplr(label))

    return np.array(res_imgs), np.array(res_labels)


def plot_imgs(imgs, labels):
    import matplotlib.pyplot as plt
    f, axs = plt.subplots(6, 2)
    plt.subplots_adjust(wspace=0, hspace=0)
    for i, (img, label) in enumerate(zip(imgs, labels)):
        axs[i][0].imshow(img)
        axs[i][0].axis('off')
        axs[i][1].imshow(label[:, :, 1], cmap='gray')
        axs[i][1].axis('off')
    plt.show()


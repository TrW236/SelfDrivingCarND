import os.path
import tensorflow as tf
import warnings
import numpy as np
import pickle
from utils import *


def load_vgg(sess, vgg_path):
    """
    Load Pretrained VGG Model into TensorFlow.
    :param sess: TensorFlow Session
    :param vgg_path: Path to vgg folder, containing "variables/" and "saved_model.pb"
    :return: Tuple of Tensors from VGG model (image_input, keep_prob, layer3_out, layer4_out, layer7_out)
    """
    vgg_tag = 'vgg16'
    vgg_input_tensor_name = 'image_input:0'
    vgg_keep_prob_tensor_name = 'keep_prob:0'
    vgg_layer3_out_tensor_name = 'layer3_out:0'
    vgg_layer4_out_tensor_name = 'layer4_out:0'
    vgg_layer7_out_tensor_name = 'layer7_out:0'

    tf.saved_model.loader.load(sess, [vgg_tag], vgg_path)
    graph = tf.get_default_graph()
    image_input = graph.get_tensor_by_name(vgg_input_tensor_name)
    keep_prob = graph.get_tensor_by_name(vgg_keep_prob_tensor_name)
    layer3_out = graph.get_tensor_by_name(vgg_layer3_out_tensor_name)
    layer4_out = graph.get_tensor_by_name(vgg_layer4_out_tensor_name)
    layer7_out = graph.get_tensor_by_name(vgg_layer7_out_tensor_name)

    return image_input, keep_prob, layer3_out, layer4_out, layer7_out


def layers(vgg_layer3_out, vgg_layer4_out, vgg_layer7_out, num_classes):
    """
    Create the layers for a fully convolutional network.  Build skip-layers using the vgg layers.
    :param vgg_layer7_out: TF Tensor for VGG Layer 3 output
    :param vgg_layer4_out: TF Tensor for VGG Layer 4 output
    :param vgg_layer3_out: TF Tensor for VGG Layer 7 output
    :param num_classes: Number of classes to classify
    :return: The Tensor for the last layer of output
    """

    conv_1x1_7 = tf.layers.conv2d(vgg_layer7_out, num_classes, 1, 1, padding='same',
                                kernel_initializer=tf.random_normal_initializer(stddev=0.01),
                                kernel_regularizer=tf.contrib.layers.l2_regularizer(1e-3))

    output = tf.layers.conv2d_transpose(conv_1x1_7, num_classes, 4, 2, padding='same',
                                        kernel_initializer=tf.random_normal_initializer(stddev=0.01),
                                        kernel_regularizer=tf.contrib.layers.l2_regularizer(1e-3))
    conv_1x1_4 = tf.layers.conv2d(vgg_layer4_out, num_classes, 1, 1, padding='same',
                                kernel_initializer=tf.random_normal_initializer(stddev=0.01),
                                kernel_regularizer=tf.contrib.layers.l2_regularizer(1e-3))
    output = tf.add(output, conv_1x1_4)

    output = tf.layers.conv2d_transpose(output, num_classes, 4, 2, padding='same',
                                        kernel_initializer=tf.random_normal_initializer(stddev=0.01),
                                        kernel_regularizer=tf.contrib.layers.l2_regularizer(1e-3))
    conv_1x1_3 = tf.layers.conv2d(vgg_layer3_out, num_classes, 1, 1, padding='same',
                                kernel_initializer=tf.random_normal_initializer(stddev=0.01),
                                kernel_regularizer=tf.contrib.layers.l2_regularizer(1e-3))
    output = tf.add(output, conv_1x1_3)

    output = tf.layers.conv2d_transpose(output, num_classes, 16, 8, padding='same',
                                        kernel_initializer=tf.random_normal_initializer(stddev=0.01),
                                        kernel_regularizer=tf.contrib.layers.l2_regularizer(1e-3))
    return output


def optimize(nn_last_layer, correct_label, learning_rate, num_classes):
    """
    Build the TensorFLow loss and optimizer operations.
    :param nn_last_layer: TF Tensor of the last layer in the neural network
    :param correct_label: TF Placeholder for the correct label image
    :param learning_rate: TF Placeholder for the learning rate
    :param num_classes: Number of classes to classify
    :return: Tuple of (logits, train_op, cross_entropy_loss)
    """
    logits = tf.reshape(nn_last_layer, (-1, num_classes))
    correct_label = tf.reshape(correct_label, (-1, num_classes))

    cross_entropy_loss = tf.reduce_mean(tf.nn.softmax_cross_entropy_with_logits(logits=logits, labels=correct_label))

    optimizer = tf.train.AdamOptimizer(learning_rate=learning_rate)  # create Optimizer
    train_op = optimizer.minimize(cross_entropy_loss)  # set optimizer to minimize a loss

    return logits, train_op, cross_entropy_loss


def train_nn(sess, epochs, batch_size, get_batches_fn, train_op, cross_entropy_loss, input_image,
             correct_label, keep_prob, learning_rate):
    """
    Train neural network and print out the loss during training.
    :param sess: TF Session
    :param epochs: Number of epochs
    :param batch_size: Batch size
    :param get_batches_fn: Function to get batches of training data.  Call using get_batches_fn(batch_size)
    :param train_op: TF Operation to train the neural network
    :param cross_entropy_loss: TF Tensor for the amount of loss
    :param input_image: TF Placeholder for input images
    :param correct_label: TF Placeholder for label images
    :param keep_prob: TF Placeholder for dropout keep probability
    :param learning_rate: TF Placeholder for learning rate
    """
    kp = 0.8  # keep prob  todo param
    lr = 1e-5  # learning rate  todo param

    losses = []
    for epoch in range(epochs):
        for imgs, labels in get_batches_fn(batch_size):
            # Augment Images for better results
            imgs, labels = augment_data(imgs, labels)

            sess.run(train_op, feed_dict={input_image: imgs, correct_label: labels, keep_prob: kp, learning_rate: lr})
            loss = sess.run(cross_entropy_loss, feed_dict={input_image: imgs, keep_prob: 1.0, correct_label: labels})
            losses.append(loss)
            print('Epoch: ', epoch, '; loss: ', loss)
    return losses


def run():
    num_classes = 2
    image_shape = (160, 576)
    epochs = 60
    batch_size = 1

    data_dir = '/data'  # todo I use Floydhub to train my model, if using local machine, the path must be changed appropriately
    runs_dir = '/output'  # todo I use Floydhub to train my model, if using local machine, the path must be changed appropriately

    with tf.Session() as sess:
        # Path to vgg model
        vgg_path = os.path.join(data_dir, 'vgg')
        # Create function to get batches
        get_batches_fn = gen_batch_function(os.path.join(data_dir, 'data_road/training'), image_shape)

        correct_label = tf.placeholder(tf.float32, [None, image_shape[0], image_shape[1], num_classes])
        learning_rate = tf.placeholder(tf.float32)

        # Build NN using load_vgg, layers, and optimize function
        input_image, keep_prob, vgg_layer3_out, vgg_layer4_out, vgg_layer7_out = load_vgg(sess, vgg_path)
        nn_last_layer = layers(vgg_layer3_out, vgg_layer4_out, vgg_layer7_out, num_classes)

        logits, train_op, cross_entropy_loss = optimize(nn_last_layer, correct_label, learning_rate, num_classes)
        # Train NN using the train_nn function
        sess.run(tf.global_variables_initializer())
        losses = train_nn(sess, epochs, batch_size, get_batches_fn, train_op, cross_entropy_loss, input_image,
                          correct_label, keep_prob, learning_rate)
        # Save inference data using helper.save_inference_samples
        save_inference_samples(runs_dir, data_dir, sess, image_shape, logits, keep_prob, input_image)

        pickle.dump(losses, open(os.path.join(runs_dir, "losses.p"), "wb"))

        # Apply the trained model to a video todo


if __name__ == '__main__':
    run()

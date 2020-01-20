from matplotlib import pyplot as plt
import cv2
import random
import matplotlib.image as mpimg


def draw_pics(imgs, titles=None, cmap=None, fontsize=15, axis='off'):
    n_pics = len(imgs)

    if cmap is None:
        cmap = [None]*n_pics

    if titles is None:
        titles = ['picture']*n_pics

    fig, axs = plt.subplots(1, n_pics, figsize=(15, 5), dpi=80)
    axs = axs.ravel()

    for i in range(n_pics):
        axs[i].imshow(imgs[i], cmap=cmap[i])
        axs[i].axis(axis)
        axs[i].set_title(titles[i], fontsize=fontsize)

    fig.tight_layout()


def get_rand_samples(imgs, num):
    outs = []
    for i in range(num):
        idx = random.randint(0, len(imgs))
        car = mpimg.imread(imgs[idx])
        outs.append(car)
    return outs


def plot_axes(data, fontsize=15, titles=None):
    n = len(data)
    if titles is None:
        titles = ["fig"]*n
    f, axs = plt.subplots(1, n, figsize=(15, 5), dpi=80)
    axs = axs.ravel()
    for i in range(n):
        axs[i].plot(data[i])
        axs[i].set_title(titles[i], fontsize=fontsize)
    f.tight_layout()


def draw_lines(img, vertices, color=(0,0,255), thickness=5):
    for i in range(len(vertices)-1):
        cv2.line(img, (vertices[i][0],vertices[i][1]), (vertices[i+1][0], vertices[i+1][1]), color, thickness)


def draw_closed_lines(img, vertices, color=(0,0,255), thickness=5):
    draw_lines(img, vertices, color, thickness)
    cv2.line(img, (vertices[-1][0],vertices[-1][1]), (vertices[0][0], vertices[0][1]), color, thickness)


def point_lin(p0, p1, rate):
    x = int(p0[0] + (p1[0] - p0[0]) * rate)
    y = int(p0[1] + (p1[1] - p0[1]) * rate)
    return [x, y]


def val_lin(val0, val1, rate):
    return val0 + (val1 - val0)*rate


from matplotlib import pyplot as plt
import cv2


def draw_pics(n_pics, imgs, titles=None, cmap=None, fontsize=15, axis='off'):
    if cmap is None:
        cmap = [None]*n_pics

    fig, axs = plt.subplots(1, n_pics, figsize=(15, 5), dpi=80)
    axs = axs.ravel()

    if titles is None:
        titles = ['picture']*n_pics

    for i in range(n_pics):
        axs[i].imshow(imgs[i], cmap=cmap[i])
        axs[i].axis(axis)
        axs[i].set_title(titles[i], fontsize=fontsize)

    fig.tight_layout()


def plot_2axes(feature1, feature2, fontsize=15, titles=None):
    if titles is None:
        titles = ["fig_1", "fig_2"]
    f, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 5), dpi=80)
    ax1.plot(feature1)
    ax1.set_title(titles[0], fontsize=fontsize)
    ax2.plot(feature2)
    ax2.set_title(titles[1], fontsize=fontsize)


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


#!/usr/bin/env python

import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d # For 3D plot
import matplotlib.gridspec as gridspec # For custom subplot grid
import numpy as np
import time
import sys

def load_data(filename):
    folder = './catkin_ws/src/uav_vision/data_storage/'
    data = np.load(folder + filename, allow_pickle=True)

    n = len(data)
    t_i = 0
    gt_i = 1
    yolo_i = gt_i + 6
    tcv_i = yolo_i + 6
    yolo_error_i = tcv_i + 6
    filtered_i = yolo_error_i + 6

    time = np.zeros(n)
    gt = np.zeros((n,6))
    yolo = np.zeros((n,6))
    tcv = np.zeros((n,6))
    yolo_error = np.zeros((n,6))
    filtered = np.zeros((n,6))
    for i, data_point in enumerate(data):
        time[i] = data_point[t_i]
        gt[i] = data_point[gt_i:gt_i+6]
        yolo[i] = data_point[yolo_i:yolo_i+6]
        tcv[i] = data_point[tcv_i:tcv_i+6]
        yolo_error[i] = data_point[yolo_error_i:yolo_error_i+6]
        filtered[i] = data_point[filtered_i:filtered_i+6]

    return time, gt, yolo, tcv, yolo_error, filtered


def plot_xyz(time, gt, est, savename):
    fig = plt.figure(figsize=(10,10))
    ax = fig.add_subplot(111,projection='3d')
    plt.title('Estimated trajectory xy')
    est, = plt.plot(est[:,0], est[:,1], est[:,2])
    gt, = plt.plot( gt[:,0], gt[:,1], gt[:,2])
    plt.legend([est, gt], ['yolo estimate x,y, z', 'ground truth x, y, z'])

    folder = './catkin_ws/src/uav_vision/data_storage/plots/'
    plt.savefig(folder+savename+'.svg')
    fig.tight_layout()
    fig.show()
    try:
        plt.waitforbuttonpress()
        plt.close()
    except Exception as e:
        pass

def plot_xy(time, gt, est, savename):
    fig = plt.figure(figsize=(10,10))
    plt.title('Estimated trajectory xy')
    e, = plt.plot(est[:,0], est[:,1])
    gt, = plt.plot(gt[:,0], gt[:,1])
    plt.legend([e, gt], ['yolo estimate x,y', 'ground truth x, y'])

    folder = './catkin_ws/src/uav_vision/data_storage/plots/'
    plt.savefig(folder+savename+'.svg')
    fig.tight_layout()
    fig.show()
    try:
        plt.waitforbuttonpress(0)
        plt.close()
    except Exception as e:
        pass


def est_plot(time, gt, est, savename):
    file_title = 'yolo_v4_tiny_estimate_vs_gt_hovering'
    variables = ['x', 'y', 'z', 'yaw']
    legend_values = ['est_x', 'est_y' ,'est_z', 'est_yaw']
    subtitles = variables
    fig = plt.figure(figsize=(12,8))
    plt.title('Yolo estimate while hovering')
    for i in range(4):
        k = i + 2 if i == 3 else i
        ax = plt.subplot(2,2,i+1)
        ax.set_xlabel('Time [s]')
        ax.set_ylabel('[deg]' if i == 3 else '[m]')
        ax.axhline(y=0, color='grey', linestyle='--')
        ax.legend(legend_values[i])
        plt.grid()
        plt.title(subtitles[i])
        data_line, = ax.plot(time,est[:,k], color='b')
        data_line.set_label('yolo_estimate')
        gt_line, = ax.plot(time,gt[:,k], color='r')
        plt.legend([data_line, gt_line],[legend_values[i],'ground truth'])

    folder = './catkin_ws/src/uav_vision/data_storage/plots/'
    plt.savefig(folder+savename+'.svg')

    fig.tight_layout()
    fig.show()

    try:
        plt.waitforbuttonpress(0)
        plt.close()
    except Exception as e:
        pass

def main():

    loadname = 'hovering_5m_yolov4t_256_fb_comb.npy'
    savename = loadname.split('.')[0]

    time, gt, yolo, tcv, yolo_error, filtered = load_data(loadname)
    est_plot(time, gt, yolo, savename +"_var_plot")
    plot_xy(time, gt, yolo, savename+"_xy_plot")
    plot_xyz(time, gt, yolo, savename+"_xyz_plot")


if __name__ == '__main__':
    main()

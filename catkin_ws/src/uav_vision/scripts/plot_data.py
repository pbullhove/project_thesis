#!/usr/bin/env python

import matplotlib.pyplot as plt
import numpy as np
import time


def plot_data(stored_array, methods_to_plot, variables_to_plot, plot_error=False):
    t_id = 0            # Time index
    g_id = 1            # Ground truth index
    e_id = g_id + 6     # Ellipse index
    a_id = e_id + 6     # Arrow index
    c_id = a_id + 6     # Corner index
    d_id = c_id + 6     # Dead reckoning index

    error_e_id = d_id + 6     # Ellipse error index
    error_a_id = error_e_id + 6     # Arrow error index
    error_c_id = error_a_id + 6     # Corner error index
    error_d_id = error_c_id + 6     # Dead reckoning error index

    time_stamps = stored_array[:, t_id]

    titles_variables = [
        "x", "y", "z", "None", "None", "yaw"
    ]
    titles_error_variables = [
        "x Position Error with ground truth z position", "y Position Error with ground truth z position", "z Position Error with ground truth z position",
        "none", "none", "yaw Rotation Error with ground truth z position"
    ]

    lables_variables = [
        "x-Position [m]", "y-Position [m]", "z-Position [m]", "none", "none", "yaw Rotation [deg]",
    ]
    lables_error_variables = [
        "x-Position Error [m]", "y-Position Error [m]", "z-Position Error [m]", "none", "none", "yaw Rotation Error [deg]",
    ]
    titles_methods = [
        "Ground truth",
        "Ellipse",
        "Arrow",
        "Corners",
        "Dead reckogning",
        "Ellipse error",
        "Arrow error",
        "Corners error",
        "Dead reckogning error",
    ]
    indices_methods = [g_id, e_id, a_id, c_id, d_id,
        error_e_id, error_a_id, error_c_id, error_d_id
    ]
    colors_methods = [
        "g",        # green:    "Ground truth"
        "b",        # blue:     "Ellipse"
        "r",        # red:      "Arrow"
        "y",        # yellow:   "Corners"
        "k",        # black:    "Dead reckogning"
        "b",        # blue:     "Ellipse error"
        "r",        # red:      "Arrow error"
        "y",        # yellow:   "Corners error"
        "k"         # black:    "Dead reckogning error"
    ]

    for variable in variables_to_plot:
        title = titles_variables[variable]

        if plot_error:
            y_label = lables_error_variables[variable]
        else:
            y_label = lables_variables[variable]

        # fig, ax = plt.subplots(figsize=(10,8))
        fig, ax = plt.subplots(figsize=(20,15))

        if plot_error:
            ax.axhline(y=0, color='grey', linestyle='--') # Plot the zero-line

            # Plot the z ground truth
            gt_method = 0
            z_variable = 2

            index = indices_methods[gt_method]
            data = stored_array[:, index:index+6][:, z_variable]

            time_stamps_local = time_stamps.copy()
            time_stamps_local[np.isnan(data)] = np.nan

            color = 'g'

            ax2 = ax.twinx()

            line, = ax2.plot(time_stamps_local, data)
            line.set_color(color)
            line.set_label("Ground truth z position")
            ax2.yaxis.grid()
            ax2.set_axisbelow(True)

            ax2.legend(loc='upper right', facecolor='white', framealpha=1)

            ax2.set_ylabel('z Position [m]', color=color)
            ax2.tick_params(axis='y', labelcolor=color)

        for method in methods_to_plot:
            legend_text = titles_methods[method]
            line_color = colors_methods[method]
            index = indices_methods[method]

            data = stored_array[:, index:index+6][:,variable]
            time_stamps_local = time_stamps.copy()
            time_stamps_local[np.isnan(data)] = np.nan
      
            line, = ax.plot(time_stamps_local, data)
            line.set_color(line_color)
            line.set_label(legend_text)

            ax.set_title(title)
            ax.set_xlabel('Time [s]')
            ax.set_ylabel(y_label)
            ax.xaxis.grid()
        
        ax.legend(loc='upper left', facecolor='white', framealpha=1)

        plt.xlim(time_stamps[0], time_stamps[-1])

        fig.tight_layout()
        # plt.draw()
        # plt.waitforbuttonpress(0)
        # plt.close(fig)
        # plt.grid()
        plt.show()


if __name__ == '__main__':
    # Settings
    test_number = 1
    plot_error = True

    # 0: x, 1: y, 2: z, 3: roll, 4: pitch, 5: yaw
    # variables_to_plot = [0, 1, 2, 5]
    variables_to_plot = [0]


    # 0: ground truth, 1: ellipse, 2: arrow, 3: corners, 4: dead reckoning
    # 5: ellipse_error, 6: arrow_error, 7: corners_error, 8: dead reckoning_error
    if plot_error:
        methods_to_plot = [5, 6, 7] #, 8]
    else:
        methods_to_plot = [0, 1, 2, 3, 4]

    # Load the data
    folder = './catkin_ws/src/uav_vision/data_storage/'
    filename = 'test_'+str(test_number)+'.npy'
    path = folder + filename
    stored_array = np.load(path, allow_pickle=True)

    plot_data(stored_array, methods_to_plot, variables_to_plot, plot_error)


import argparse
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import patches
import japanize_matplotlib

def align_yaxis(ax1, v1, ax2, v2):
    """adjust ax2 ylimit so that v2 in ax2 is aligned to v1 in ax1"""
    _, y1 = ax1.transData.transform((0, v1))
    _, y2 = ax2.transData.transform((0, v2))
    inv = ax2.transData.inverted()
    _, dy = inv.transform((0, 0)) - inv.transform((0, y1-y2))
    miny, maxy = ax2.get_ylim()
    ax2.set_ylim(miny+dy, maxy+dy)

parser = argparse.ArgumentParser()
parser.add_argument("--following_position_csv_path", default='', help="File path of following_position.csv")
parser.add_argument("--target_postion_odom_csv_path", default='', help="File path of target_postion_odom.csv")
parser.add_argument("--odom_csv_path", default='', help="File path of odom.csv")
parser.add_argument("--raw_cmd_vel_csv_path", default='', help="File path of raw_cmd_vel.csv")
parser.add_argument("--smooth_cmd_vel_csv_path", default='', help="File path of smooth_cmd_vel.csv")
parser.add_argument("--odom_velocity_csv_path", default='', help="File path of odom_velocity.csv")
parser.add_argument("--save_plot_path", default='', help="File path of Plot image")

options = parser.parse_args()

# Distance between robot and person
print("\tDistance between robot and person")
usecols = [ '%time', 'field.x', 'field.y']
target_from_robot_base = pd.read_csv( options.following_position_csv_path, encoding='shift-jis', usecols=usecols, dtype={ '%time': float, 'field.x': float, 'field.y': float } )
time = target_from_robot_base['%time'].values
time = time * (10 ** -9)
base_time = time[0]
time = time - base_time
target_x = target_from_robot_base['field.x'].values
target_y = target_from_robot_base['field.y'].values
distance = np.sqrt((target_x) ** 2 + (target_y) ** 2)
# idx = distance.size - int(distance.size * 0.03)
# distance = distance[:idx]
# time = time[:idx]
time_d = time

print(f"\t\tDistance average = {np.mean(distance[ distance > 0.0 ])}")
print(f"\t\tDistance max     = {np.max(distance[ distance > 0.0 ])}")
print(f"\t\tDistance min     = {np.min(distance[ distance > 0.0 ])}")

figsize_px = np.array([1920, 1080])
dpi = 100
figsize_inch = figsize_px / dpi
fig, ax = plt.subplots(figsize=figsize_inch, dpi=dpi)
ax.set_xlabel('t[sec]')  # x-axis label
ax.set_ylabel('Distance[m]')  # y-axis label
ax.set_title('Distance between robot and person') # Graph Title
# ax.set_aspect('equal', adjustable='box') 
ax.grid()            # Ruled lines
#ax.set_xlim([-10, 10]) # Specify drawing range in x-direction
#ax.set_ylim([0, 1])    # Specify drawing range in y-direction
ax.plot(time, distance, color="blue", label="Distance")

ax.legend(loc=0)    # Legend
fig.tight_layout()  # Layout settings
plt.savefig( options.save_plot_path + "_distance.png" ) #Saving Images
# plt.show()
plt.clf()

# odom velocity
print("\tOdometry Velocity")
usecols = [ '%time', 'field.linear.x', 'field.angular.z']
odom_velocity = pd.read_csv( options.odom_velocity_csv_path, encoding='shift-jis', usecols=usecols, dtype={ '%time':float, 'field.linear.x':float, 'field.angular.z':float } )
time = odom_velocity['%time'].values
time = time * (10 ** -9)
# base_time = time[0]
time = time - base_time
linear = odom_velocity['field.linear.x'].values
angular = odom_velocity['field.angular.z'].values
time_v = time
print(f"\t\tTranslational average = {np.mean(linear[linear > 0.0])}")
print(f"\t\tTranslational max     = {np.max(linear[linear > 0.0])}")
print(f"\t\tTranslational min     = {np.min(linear[linear > 0.0])}")
print(f"\t\tRotational average    = {np.mean(angular)}")
print(f"\t\tRotational max        = {np.max(angular)}")
print(f"\t\tRotational min        = {np.min(angular)}")

fig = plt.figure(figsize = (20, 20))
fig.suptitle('Odometry velocity')
ax1 = fig.add_subplot(2, 1, 1)
ax1.set_title("Translational")
ax1.set_xlabel('t[sec]')
ax1.set_ylabel('m/s')
ax1.plot(time, linear, color="blue", label="Translational")

# ax1.set_aspect('equal', adjustable='box')
ax1.grid()
ax2 = fig.add_subplot(2, 1, 2)
ax2.set_title("Rotational")
ax2.set_xlabel('t[sec]')
ax2.set_ylabel('rad/s')
ax2.plot(time, angular, color="green", label='Rotational')
# ax2.set_aspect('equal', adjustable='box')
ax2.grid()
plt.savefig( options.save_plot_path + "_odom_velocity.png" ) # Saving Images
# plt.show()
plt.clf()


print("\tDistance and Velocity")
# start = 8.0
# idx = time_d[ time_d < start ].size
# time_d = time_d[idx:]
# time_d = time_d - time_d[0]
# distance = distance[idx:]

# idx = time_v[ time_v < start ].size
# time_v = time_v[idx:]
# time_v = time_v - time_v[0]
# linear = linear[idx:]

figsize_px = np.array([1920, 1080])
dpi = 100
figsize_inch = figsize_px / dpi
fig, ax = plt.subplots(figsize=figsize_inch, dpi=dpi)
ax2 = ax.twinx()  # instantiate a second axes that shares the same x-axis

color="blue"
ax.plot(time_d, distance, color=color, label="人とロボットの距離", linestyle='-', linewidth=5)
font_size = 25
ax.set_xlabel('時間[sec]', size = font_size)  # x-axis label
ax.set_ylabel('人とロボットの距離[m]', size = font_size, color=color,) # y-axis label
# ax.set_title('Distance[m] & Velocity[m/s]', size = font_size) # Graph Title
ax.tick_params(axis='y', labelsize=font_size, labelcolor=color)
ax.tick_params(axis='x', labelsize=font_size)
# ax.set_aspect('equal', adjustable='box') # keep the same scale
# ax.grid()            # ruled line
# ax.set_xlim([0, max(time_v)]) # Specify drawing range in x-direction
ax.set_ylim([-0.6, 5.6])    # Specify the drawing range in the y-direction
# ax.legend(loc=1,fontsize=font_size)    # legend
ax.set_yticks([0.0, 1.0, 2.0, 3.0, 4.0, 5.0])
ax.set_yticklabels([0.0, 1.0, 2.0, 3.0, 4.0, 5.0],fontsize=font_size)

color="red"
ax2.plot(time_v, linear, color=color, label="ロボットの並進速度", linestyle='-', linewidth=5)
ax2.set_ylabel('ロボットの並進速度[m/s]', size = font_size, color=color) # y-axis label
ax2.tick_params(axis='y', labelsize=font_size, labelcolor=color)
ax2.tick_params(axis='x', labelsize=font_size)
# ax2.set_aspect('equal', adjustable='box') # keep the same scale
# ax2.set_xlim([0, max(time_v)]) # Specify drawing range in x-direction
ax2.set_ylim([-0.6, 2.6])    # Specify the drawing range in the y-direction
# ax2.legend(loc=1,fontsize=font_size)    # legend
# ax2.grid()            # ruled line
ax2.set_yticks([0.0, 0.4, 0.8, 1.2, 1.6, 2.0])
ax2.set_yticklabels([0.0, 0.4, 0.8, 1.2, 1.6, 2.0],fontsize=font_size)

align_yaxis(ax, 0, ax2, 0)
lines1, labels1 = ax.get_legend_handles_labels()
lines2, labels2 = ax2.get_legend_handles_labels()
ax2.legend(lines1+lines2, labels1+labels2, loc = (0.6, 0.85), fontsize=font_size)

ax.grid(True, which='both', linewidth = 3)
plt.xlim(0, max(time_v)) # Specify drawing range in x-direction
# plt.subplots_adjust(left=0.18, right=0.86, bottom=0.14, top=0.94)

fig.tight_layout()  # Layout Settings
plt.savefig( options.save_plot_path + "_distance_velocty.png" ) # Saving Images
# plt.show()
plt.clf()

# Tracking and Robot Trajectory
print("\tTracking and Robot Trajectory")
usecols = ['field.x', 'field.y']
target = pd.read_csv( options.target_postion_odom_csv_path, encoding='shift-jis', usecols=usecols, dtype={ 'field.x': float, 'field.y': float } )
robot = pd.read_csv( options.odom_csv_path, encoding='shift-jis', usecols=usecols, dtype={ 'field.x': float, 'field.y': float } )
target_x = target['field.x'].values
target_x = target_x+0.25
target_y = target['field.y'].values
target_y = -target_y
robot_x = robot['field.x'].values
robot_x = robot_x+0.25
robot_y = robot['field.y'].values
robot_y = -robot_y


figsize_px = np.array([1920, 1080])
dpi = 100
figsize_inch = figsize_px / dpi
fig, ax = plt.subplots(figsize=figsize_inch, dpi=dpi)
ax.set_xlabel("x[m]", size = 20)
ax.set_ylabel("y[m]", size = 20)
# ax.set_title('Tracking and Robot Trajectory') # Graph Title
ax.set_aspect('equal', adjustable='box') # keep the same scale
ax.grid()            # ruled line
ax.plot(target_y, target_x, color="deepskyblue", linestyle='-', linewidth=5, label="Person")
ax.plot(robot_y, robot_x, color="coral", linestyle='--', linewidth=5, label="Robot")
# obstacle
r = patches.Rectangle( (2.1,1.4) , 1.2, 0.9, fill=True, fc="Black", linewidth=0)
ax.add_patch(r)
r = patches.Rectangle( (4.2,0.0) , 1.2, 0.9, fill=True, fc="Black", linewidth=0)
ax.add_patch(r)
c = patches.Circle( xy=(7.05,1.85), radius=0.05,fill=True, fc="Black")
ax.add_patch(c)
ax.set_xlim([-1.0, 9.0]) # Specify drawing range in x-direction
ax.set_ylim([0, 2.3])    # Specify the drawing range in the y-direction

ax.legend(loc=0)    # legend
# fig.tight_layout()  # Layout Settings
# Set axis scale and display scale lines
plt.rcParams['font.size'] = 20
ax.set_xticks([0.0, 0.9, 2.1, 3.3, 4.2, 5.4, 7.05, 8.4, -1.0, 9.0])
ax.set_yticks([0.0, 0.9, 1.85, 1.15, 1.4, 0.0, 2.3])
xticklabels = ax.get_xticklabels()
yticklabels = ax.get_yticklabels()
# Set command to configure each element
ax.set_xticklabels([0.0, 0.9, 2.1, 3.3, 4.2, 5.4, 7.05, 8.4, -1.0, 9.0],fontsize=20)
ax.set_yticklabels([0.0, 0.9, 1.85, 1.15, 1.4, 0.0, 2.3],fontsize=20)
plt.savefig( options.save_plot_path + "_tracking_and_robot.png" ) # Saving Images
# plt.show()
plt.clf()

# smooth velocity
print("\tSmooth velocity")
usecols = [ '%time', 'field.linear.x', 'field.angular.z']
smooth_velocity = pd.read_csv( options.smooth_cmd_vel_csv_path, encoding='shift-jis', usecols=usecols, dtype={ '%time':float, 'field.linear.x':float, 'field.angular.z':float } )
time = smooth_velocity['%time'].values
time = time * (10 ** -9)
time = time - base_time
linear = smooth_velocity['field.linear.x'].values
angular = smooth_velocity['field.angular.z'].values
print(f"\t\tTranslational average = {np.mean(linear[linear > 0.0])}")
print(f"\t\tTranslational max     = {np.max(linear[linear > 0.0])}")
print(f"\t\tTranslational min     = {np.min(linear[linear > 0.0])}")
print(f"\t\tRotational average    = {np.mean(angular)}")
print(f"\t\tRotational max        = {np.max(angular)}")
print(f"\t\tRotational min        = {np.min(angular)}")

fig = plt.figure(figsize = (20, 20))
fig.suptitle('Smooth velocity')
ax1 = fig.add_subplot(2, 1, 1)
ax1.set_title("Translational")
ax1.set_xlabel('t[sec]')
ax1.set_ylabel('m/s')
ax1.plot(time, linear, color="blue", label="Translational")

# ax1.set_aspect('equal', adjustable='box')
ax1.grid()
ax2 = fig.add_subplot(2, 1, 2)
ax2.set_title("Rotational")
ax2.set_xlabel('t[sec]')
ax2.set_ylabel('rad/s')
ax2.plot(time, angular, color="green", label='Rotational')
# ax2.set_aspect('equal', adjustable='box')
ax2.grid()
plt.savefig( options.save_plot_path + "_smooth_velocity.png" ) # Saving Images
# plt.show()
plt.clf()



# velocity
print("\tVelocity")
velocity = pd.read_csv( options.raw_cmd_vel_csv_path, encoding='shift-jis', usecols=usecols, dtype={ '%time':float, 'field.linear.x':float, 'field.angular.z':float } )
time = velocity['%time'].values
time = time * (10 ** -9)
time = time - base_time
linear = velocity['field.linear.x'].values
angular = velocity['field.angular.z'].values
print(f"\t\tTranslational average = {np.mean(linear[linear > 0.0])}")
print(f"\t\tTranslational max     = {np.max(linear[linear > 0.0])}")
print(f"\t\tTranslational min     = {np.min(linear[linear > 0.0])}")
print(f"\t\tRotational average    = {np.mean(angular)}")
print(f"\t\tRotational max        = {np.max(angular)}")
print(f"\t\tRotational min        = {np.min(angular)}")

fig = plt.figure(figsize = (20, 20))
fig.suptitle('Velocity')
ax1 = fig.add_subplot(2, 1, 1)
ax1.set_title("Translational")
ax1.set_xlabel('t[sec]')
ax1.set_ylabel('m/s')
ax1.plot(time, linear, color="blue", label="Translational")

# ax1.set_aspect('equal', adjustable='box')
ax1.grid()
ax2 = fig.add_subplot(2, 1, 2)
ax2.set_title("Rotational")
ax2.set_xlabel('t[sec]')
ax2.set_ylabel('rad/s')
ax2.plot(time, angular, color="green", label='Rotational')
# ax2.set_aspect('equal', adjustable='box')
ax2.grid()
plt.savefig( options.save_plot_path + "_velocity.png" ) # Saving Images
# plt.show()
plt.clf()
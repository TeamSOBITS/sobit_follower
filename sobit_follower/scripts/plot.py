import argparse
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import patches

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
target_x = target_from_robot_base['field.x'].values
target_y = target_from_robot_base['field.y'].values
distance = np.sqrt((target_x) ** 2 + (target_y) ** 2)
idx = distance.size - int(distance.size * 0.03)
distance = distance[:idx]
time = time[:idx]

print(f"\t\tDistance average = {np.mean(distance[ distance > 0.0 ])}")
print(f"\t\tDistance max     = {np.max(distance[ distance > 0.0 ])}")
print(f"\t\tDistance min     = {np.min(distance[ distance > 0.0 ])}")

fig, ax = plt.subplots()
ax.set_xlabel('t')  # x軸ラベル
ax.set_ylabel('Distance[m]')  # y軸ラベル
ax.set_title('Distance between robot and person') # グラフタイトル
# ax.set_aspect('equal', adjustable='box') # スケールを揃える
ax.grid()            # 罫線
#ax.set_xlim([-10, 10]) # x方向の描画範囲を指定
#ax.set_ylim([0, 1])    # y方向の描画範囲を指定
ax.plot(time, distance, color="blue", label="Distance")

ax.legend(loc=0)    # 凡例
fig.tight_layout()  # レイアウトの設定
plt.savefig( options.save_plot_path + "_distance.png" ) # 画像の保存
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


fig, ax = plt.subplots()
ax.set_xlabel('x[m]')  # x軸ラベル
ax.set_ylabel('y[m]')  # y軸ラベル
ax.set_title('Tracking and Robot Trajectory') # グラフタイトル
ax.set_aspect('equal', adjustable='box') # スケールを揃える
ax.grid()            # 罫線
ax.plot(target_y, target_x, color="deepskyblue", linestyle='-', linewidth=3, label="Person")
ax.plot(robot_y, robot_x, color="coral", linestyle='--', linewidth=3, label="Robot")
# obstacle
r = patches.Rectangle( (2.1,1.4) , 1.2, 0.9, fill=True, fc="Black", linewidth=0)
ax.add_patch(r)
r = patches.Rectangle( (4.2,0.0) , 1.2, 0.9, fill=True, fc="Black", linewidth=0)
ax.add_patch(r)
c = patches.Circle( xy=(7.05,1.85), radius=0.05,fill=True, fc="Black")
ax.add_patch(c)
ax.set_xlim([-0.5, 9.0]) # x方向の描画範囲を指定
ax.set_ylim([0, 2.3])    # y方向の描画範囲を指定

ax.legend(loc=0)    # 凡例
fig.tight_layout()  # レイアウトの設定
plt.savefig( options.save_plot_path + "_tracking_and_robot.png" ) # 画像の保存
# plt.show()
plt.clf()

# smooth velocity
print("\tSmooth velocity")
usecols = [ '%time', 'field.linear.x', 'field.angular.z']
smooth_velocity = pd.read_csv( options.smooth_cmd_vel_csv_path, encoding='shift-jis', usecols=usecols, dtype={ '%time':float, 'field.linear.x':float, 'field.angular.z':float } )
time = smooth_velocity['%time'].values.tolist()
linear = smooth_velocity['field.linear.x'].values
angular = smooth_velocity['field.angular.z'].values
print(f"\t\tTranslational average = {np.mean(linear[linear > 0.0])}")
print(f"\t\tTranslational max     = {np.max(linear[linear > 0.0])}")
print(f"\t\tTranslational min     = {np.min(linear[linear > 0.0])}")
print(f"\t\tRotational average    = {np.mean(angular)}")
print(f"\t\tRotational max        = {np.max(angular)}")
print(f"\t\tRotational min        = {np.min(angular)}")

fig = plt.figure(figsize = (10, 10))
fig.suptitle('Smooth velocity')
ax1 = fig.add_subplot(2, 1, 1)
ax1.set_title("Translational")
ax1.set_xlabel('t')
ax1.set_ylabel('m/s')
ax1.plot(time, linear, color="blue", label="Translational")

# ax1.set_aspect('equal', adjustable='box')
ax1.grid()
ax2 = fig.add_subplot(2, 1, 2)
ax2.set_title("Rotational")
ax2.set_xlabel('t')
ax2.set_ylabel('rad/s')
ax2.plot(time, angular, color="green", label='Rotational')
# ax2.set_aspect('equal', adjustable='box')
ax2.grid()
plt.savefig( options.save_plot_path + "_smooth_velocity.png" ) # 画像の保存
# plt.show()
plt.clf()

# velocity
print("\tVelocity")
velocity = pd.read_csv( options.raw_cmd_vel_csv_path, encoding='shift-jis', usecols=usecols, dtype={ '%time':float, 'field.linear.x':float, 'field.angular.z':float } )
time = velocity['%time'].values.tolist()
linear = velocity['field.linear.x'].values
angular = velocity['field.angular.z'].values
print(f"\t\tTranslational average = {np.mean(linear[linear > 0.0])}")
print(f"\t\tTranslational max     = {np.max(linear[linear > 0.0])}")
print(f"\t\tTranslational min     = {np.min(linear[linear > 0.0])}")
print(f"\t\tRotational average    = {np.mean(angular)}")
print(f"\t\tRotational max        = {np.max(angular)}")
print(f"\t\tRotational min        = {np.min(angular)}")

fig = plt.figure(figsize = (10, 10))
fig.suptitle('Velocity')
ax1 = fig.add_subplot(2, 1, 1)
ax1.set_title("Translational")
ax1.set_xlabel('t')
ax1.set_ylabel('m/s')
ax1.plot(time, linear, color="blue", label="Translational")

# ax1.set_aspect('equal', adjustable='box')
ax1.grid()
ax2 = fig.add_subplot(2, 1, 2)
ax2.set_title("Rotational")
ax2.set_xlabel('t')
ax2.set_ylabel('rad/s')
ax2.plot(time, angular, color="green", label='Rotational')
# ax2.set_aspect('equal', adjustable='box')
ax2.grid()
plt.savefig( options.save_plot_path + "_velocity.png" ) # 画像の保存
# plt.show()
plt.clf()

# odom velocity
print("\tOdometry Velocity")
odom_velocity = pd.read_csv( options.odom_velocity_csv_path, encoding='shift-jis', usecols=usecols, dtype={ '%time':float, 'field.linear.x':float, 'field.angular.z':float } )
time = odom_velocity['%time'].values.tolist()
linear = odom_velocity['field.linear.x'].values
angular = odom_velocity['field.angular.z'].values
print(f"\t\tTranslational average = {np.mean(linear[linear > 0.0])}")
print(f"\t\tTranslational max     = {np.max(linear[linear > 0.0])}")
print(f"\t\tTranslational min     = {np.min(linear[linear > 0.0])}")
print(f"\t\tRotational average    = {np.mean(angular)}")
print(f"\t\tRotational max        = {np.max(angular)}")
print(f"\t\tRotational min        = {np.min(angular)}")

fig = plt.figure(figsize = (10, 10))
fig.suptitle('Odometry velocity')
ax1 = fig.add_subplot(2, 1, 1)
ax1.set_title("Translational")
ax1.set_xlabel('t')
ax1.set_ylabel('m/s')
ax1.plot(time, linear, color="blue", label="Translational")

# ax1.set_aspect('equal', adjustable='box')
ax1.grid()
ax2 = fig.add_subplot(2, 1, 2)
ax2.set_title("Rotational")
ax2.set_xlabel('t')
ax2.set_ylabel('rad/s')
ax2.plot(time, angular, color="green", label='Rotational')
# ax2.set_aspect('equal', adjustable='box')
ax2.grid()
plt.savefig( options.save_plot_path + "_odom_velocity.png" ) # 画像の保存
# plt.show()
plt.clf()
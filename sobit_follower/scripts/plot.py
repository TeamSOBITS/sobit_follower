import argparse
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

parser = argparse.ArgumentParser()
parser.add_argument("--following_position_csv_path", default='', help="File path of following_position.csv")
parser.add_argument("--target_postion_odom_csv_path", default='', help="File path of target_postion_odom.csv")
parser.add_argument("--odom_csv_path", default='', help="File path of odom.csv")
parser.add_argument("--raw_cmd_vel_csv_path", default='', help="File path of raw_cmd_vel.csv")
parser.add_argument("--smooth_cmd_vel_csv_path", default='', help="File path of smooth_cmd_vel.csv")
parser.add_argument("--odom_velocity_csv_path", default='', help="File path of odom_velocity.csv")
parser.add_argument("--save_plot_path", default='', help="File path of Plot image")

options = parser.parse_args()

# Targets and Robot Trajectory
print("\tTargets and Robot Trajectory")
usecols = ['field.x', 'field.y']
target = pd.read_csv( options.target_postion_odom_csv_path, encoding='shift-jis', usecols=usecols, dtype={ 'field.x': float, 'field.y': float } )
robot = pd.read_csv( options.odom_csv_path, encoding='shift-jis', usecols=usecols, dtype={ 'field.x': float, 'field.y': float } )
target_x = target['field.x'].values
target_y = target['field.y'].values
target_y = -target_y
robot_x = robot['field.x'].values
robot_y = robot['field.y'].values
robot_y = -robot_y

fig, ax = plt.subplots()
ax.set_xlabel('y')  # x軸ラベル
ax.set_ylabel('x')  # y軸ラベル
ax.set_title('Targets and Robot Trajectory') # グラフタイトル
ax.set_aspect('equal', adjustable='box') # スケールを揃える
ax.grid()            # 罫線
#ax.set_xlim([-10, 10]) # x方向の描画範囲を指定
#ax.set_ylim([0, 1])    # y方向の描画範囲を指定
ax.plot(target_y, target_x, color="blue", label="Targets")
ax.plot(robot_y, robot_x, color="green", label="Robot")

ax.legend(loc=0)    # 凡例
fig.tight_layout()  # レイアウトの設定
plt.savefig( options.save_plot_path + "_targets_and_robot.png" ) # 画像の保存
# plt.show()
plt.clf()

# smooth velocity
print("\tSmooth velocity")
usecols = [ '%time', 'field.linear.x', 'field.angular.z']
smooth_velocity = pd.read_csv( options.smooth_cmd_vel_csv_path, encoding='shift-jis', usecols=usecols, dtype={ '%time':float, 'field.linear.x':float, 'field.angular.z':float } )
time = smooth_velocity['%time'].values.tolist()
linear = smooth_velocity['field.linear.x'].values.tolist()
angular = smooth_velocity['field.angular.z'].values.tolist()
fig = plt.figure(figsize = (10, 10))
fig.suptitle('Smooth velocity')
ax1 = fig.add_subplot(2, 1, 1)
ax1.set_title("Linear")
ax1.set_xlabel('t')
ax1.set_ylabel('m/s')
ax1.plot(time, linear, color="blue", label="linear")
# ax1.set_aspect('equal', adjustable='box')
ax1.grid()
ax2 = fig.add_subplot(2, 1, 2)
ax2.set_title("Angular")
ax2.set_xlabel('t')
ax2.set_ylabel('rad/s')
ax2.plot(time, angular, color="green", label='angular')
# ax2.set_aspect('equal', adjustable='box')
ax2.grid()
plt.savefig( options.save_plot_path + "_smooth_velocity.png" ) # 画像の保存
# plt.show()
plt.clf()

# velocity
print("\tVelocity")
velocity = pd.read_csv( options.raw_cmd_vel_csv_path, encoding='shift-jis', usecols=usecols, dtype={ '%time':float, 'field.linear.x':float, 'field.angular.z':float } )
time = velocity['%time'].values.tolist()
linear = velocity['field.linear.x'].values.tolist()
angular = velocity['field.angular.z'].values.tolist()
fig = plt.figure(figsize = (10, 10))
fig.suptitle('Velocity')
ax1 = fig.add_subplot(2, 1, 1)
ax1.set_title("Linear")
ax1.set_xlabel('t')
ax1.set_ylabel('m/s')
ax1.plot(time, linear, color="blue", label="linear")
# ax1.set_aspect('equal', adjustable='box')
ax1.grid()
ax2 = fig.add_subplot(2, 1, 2)
ax2.set_title("Angular")
ax2.set_xlabel('t')
ax2.set_ylabel('rad/s')
ax2.plot(time, angular, color="green", label='angular')
# ax2.set_aspect('equal', adjustable='box')
ax2.grid()
plt.savefig( options.save_plot_path + "_velocity.png" ) # 画像の保存
# plt.show()
plt.clf()

# odom velocity
print("\tOdometry Velocity")
odom_velocity = pd.read_csv( options.odom_velocity_csv_path, encoding='shift-jis', usecols=usecols, dtype={ '%time':float, 'field.linear.x':float, 'field.angular.z':float } )
time = odom_velocity['%time'].values.tolist()
linear = odom_velocity['field.linear.x'].values.tolist()
angular = odom_velocity['field.angular.z'].values.tolist()
fig = plt.figure(figsize = (10, 10))
fig.suptitle('odom velocity')
ax1 = fig.add_subplot(2, 1, 1)
ax1.set_title("Linear")
ax1.set_xlabel('t')
ax1.set_ylabel('m/s')
ax1.plot(time, linear, color="blue", label="linear")
# ax1.set_aspect('equal', adjustable='box')
ax1.grid()
ax2 = fig.add_subplot(2, 1, 2)
ax2.set_title("Angular")
ax2.set_xlabel('t')
ax2.set_ylabel('rad/s')
ax2.plot(time, angular, color="green", label='angular')
# ax2.set_aspect('equal', adjustable='box')
ax2.grid()
plt.savefig( options.save_plot_path + "_odom_velocity.png" ) # 画像の保存
# plt.show()
plt.clf()
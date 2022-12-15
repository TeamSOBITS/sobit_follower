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

# # Distance between robot and person
# print("\tDistance between robot and person")
# usecols = [ '%time', 'field.x', 'field.y']
# target_from_robot_base = pd.read_csv( options.following_position_csv_path, encoding='shift-jis', usecols=usecols, dtype={ '%time': float, 'field.x': float, 'field.y': float } )
# time = target_from_robot_base['%time'].values
# target_x = target_from_robot_base['field.x'].values
# target_y = target_from_robot_base['field.y'].values
# distance = np.sqrt((target_x) ** 2 + (target_y) ** 2)
# idx = distance.size - int(distance.size * 0.03)
# distance = distance[:idx]
# time = time[:idx]

# print(f"\t\tDistance average = {np.mean(distance[ distance > 0.0 ])}")
# print(f"\t\tDistance max     = {np.max(distance[ distance > 0.0 ])}")
# print(f"\t\tDistance min     = {np.min(distance[ distance > 0.0 ])}")

# fig, ax = plt.subplots()
# ax.set_xlabel('t')  # x軸ラベル
# ax.set_ylabel('Distance[m]')  # y軸ラベル
# ax.set_title('Distance between robot and person') # グラフタイトル
# # ax.set_aspect('equal', adjustable='box') # スケールを揃える
# ax.grid()            # 罫線
# #ax.set_xlim([-10, 10]) # x方向の描画範囲を指定
# #ax.set_ylim([0, 1])    # y方向の描画範囲を指定
# ax.plot(time, distance, color="blue", label="Distance")

# ax.legend(loc=0)    # 凡例
# fig.tight_layout()  # レイアウトの設定
# plt.savefig( options.save_plot_path + "_distance.png" ) # 画像の保存
# # plt.show()
# plt.clf()

# Tracking and Robot Trajectory
# print("\tTracking and Robot Trajectory")
# usecols = ['field.x', 'field.y']
# target = pd.read_csv( options.target_postion_odom_csv_path, encoding='shift-jis', usecols=usecols, dtype={ 'field.x': float, 'field.y': float } )
# robot = pd.read_csv( options.odom_csv_path, encoding='shift-jis', usecols=usecols, dtype={ 'field.x': float, 'field.y': float } )
# target_x = target['field.x'].values
# target_x = target_x+0.25
# target_y = target['field.y'].values
# target_y = -target_y
# robot_x = robot['field.x'].values
# robot_x = robot_x+0.25
# robot_y = robot['field.y'].values
# robot_y = -robot_y


# fig, ax = plt.subplots()
# ax.set_xlabel("x[m]", size = 20)
# ax.set_ylabel("y[m]", size = 20)
# # ax.set_title('Tracking and Robot Trajectory') # グラフタイトル
# ax.set_aspect('equal', adjustable='box') # スケールを揃える
# ax.grid()            # 罫線
# ax.plot(target_y, target_x, color="deepskyblue", linestyle='-', linewidth=5, label="Person")
# ax.plot(robot_y, robot_x, color="coral", linestyle='--', linewidth=5, label="Robot")
# # obstacle
# r = patches.Rectangle( (2.1,1.4) , 1.2, 0.9, fill=True, fc="Black", linewidth=0)
# ax.add_patch(r)
# r = patches.Rectangle( (4.2,0.0) , 1.2, 0.9, fill=True, fc="Black", linewidth=0)
# ax.add_patch(r)
# c = patches.Circle( xy=(7.05,1.85), radius=0.05,fill=True, fc="Black")
# ax.add_patch(c)
# ax.set_xlim([-1.0, 9.0]) # x方向の描画範囲を指定
# ax.set_ylim([0, 2.3])    # y方向の描画範囲を指定

# ax.legend(loc=0)    # 凡例
# # fig.tight_layout()  # レイアウトの設定
# # 軸目盛の設定と目盛り線の表示
# plt.rcParams['font.size'] = 20
# ax.set_xticks([0.0, 0.9, 2.1, 3.3, 4.2, 5.4, 7.05, 8.4, -1.0, 9.0])
# ax.set_yticks([0.0, 0.9, 1.85, 1.15, 1.4, 0.0, 2.3])
# xticklabels = ax.get_xticklabels()
# yticklabels = ax.get_yticklabels()
# # 各要素の設定をおこなうsetコマンド
# ax.set_xticklabels([0.0, 0.9, 2.1, 3.3, 4.2, 5.4, 7.05, 8.4, -1.0, 9.0],fontsize=20)
# ax.set_yticklabels([0.0, 0.9, 1.85, 1.15, 1.4, 0.0, 2.3],fontsize=20)
# # plt.savefig( options.save_plot_path + "_tracking_and_robot.png" ) # 画像の保存
# plt.show()
# plt.clf()

# # smooth velocity
# print("\tSmooth velocity")
# usecols = [ '%time', 'field.linear.x', 'field.angular.z']
# smooth_velocity = pd.read_csv( options.smooth_cmd_vel_csv_path, encoding='shift-jis', usecols=usecols, dtype={ '%time':float, 'field.linear.x':float, 'field.angular.z':float } )
# time = smooth_velocity['%time'].values.tolist()
# linear = smooth_velocity['field.linear.x'].values
# angular = smooth_velocity['field.angular.z'].values
# print(f"\t\tTranslational average = {np.mean(linear[linear > 0.0])}")
# print(f"\t\tTranslational max     = {np.max(linear[linear > 0.0])}")
# print(f"\t\tTranslational min     = {np.min(linear[linear > 0.0])}")
# print(f"\t\tRotational average    = {np.mean(angular)}")
# print(f"\t\tRotational max        = {np.max(angular)}")
# print(f"\t\tRotational min        = {np.min(angular)}")

# fig = plt.figure(figsize = (10, 10))
# fig.suptitle('Smooth velocity')
# ax1 = fig.add_subplot(2, 1, 1)
# ax1.set_title("Translational")
# ax1.set_xlabel('t')
# ax1.set_ylabel('m/s')
# ax1.plot(time, linear, color="blue", label="Translational")

# # ax1.set_aspect('equal', adjustable='box')
# ax1.grid()
# ax2 = fig.add_subplot(2, 1, 2)
# ax2.set_title("Rotational")
# ax2.set_xlabel('t')
# ax2.set_ylabel('rad/s')
# ax2.plot(time, angular, color="green", label='Rotational')
# # ax2.set_aspect('equal', adjustable='box')
# ax2.grid()
# plt.savefig( options.save_plot_path + "_smooth_velocity.png" ) # 画像の保存
# # plt.show()
# plt.clf()

# # velocity
# print("\tVelocity")
# velocity = pd.read_csv( options.raw_cmd_vel_csv_path, encoding='shift-jis', usecols=usecols, dtype={ '%time':float, 'field.linear.x':float, 'field.angular.z':float } )
# time = velocity['%time'].values.tolist()
# linear = velocity['field.linear.x'].values
# angular = velocity['field.angular.z'].values
# print(f"\t\tTranslational average = {np.mean(linear[linear > 0.0])}")
# print(f"\t\tTranslational max     = {np.max(linear[linear > 0.0])}")
# print(f"\t\tTranslational min     = {np.min(linear[linear > 0.0])}")
# print(f"\t\tRotational average    = {np.mean(angular)}")
# print(f"\t\tRotational max        = {np.max(angular)}")
# print(f"\t\tRotational min        = {np.min(angular)}")

# fig = plt.figure(figsize = (10, 10))
# fig.suptitle('Velocity')
# ax1 = fig.add_subplot(2, 1, 1)
# ax1.set_title("Translational")
# ax1.set_xlabel('t')
# ax1.set_ylabel('m/s')
# ax1.plot(time, linear, color="blue", label="Translational")

# # ax1.set_aspect('equal', adjustable='box')
# ax1.grid()
# ax2 = fig.add_subplot(2, 1, 2)
# ax2.set_title("Rotational")
# ax2.set_xlabel('t')
# ax2.set_ylabel('rad/s')
# ax2.plot(time, angular, color="green", label='Rotational')
# # ax2.set_aspect('equal', adjustable='box')
# ax2.grid()
# plt.savefig( options.save_plot_path + "_velocity.png" ) # 画像の保存
# # plt.show()
# plt.clf()

# # odom velocity
# print("\tOdometry Velocity")
# odom_velocity = pd.read_csv( options.odom_velocity_csv_path, encoding='shift-jis', usecols=usecols, dtype={ '%time':float, 'field.linear.x':float, 'field.angular.z':float } )
# time = odom_velocity['%time'].values.tolist()
# linear = odom_velocity['field.linear.x'].values
# angular = odom_velocity['field.angular.z'].values
# print(f"\t\tTranslational average = {np.mean(linear[linear > 0.0])}")
# print(f"\t\tTranslational max     = {np.max(linear[linear > 0.0])}")
# print(f"\t\tTranslational min     = {np.min(linear[linear > 0.0])}")
# print(f"\t\tRotational average    = {np.mean(angular)}")
# print(f"\t\tRotational max        = {np.max(angular)}")
# print(f"\t\tRotational min        = {np.min(angular)}")

# fig = plt.figure(figsize = (10, 10))
# fig.suptitle('Odometry velocity')
# ax1 = fig.add_subplot(2, 1, 1)
# ax1.set_title("Translational")
# ax1.set_xlabel('t')
# ax1.set_ylabel('m/s')
# ax1.plot(time, linear, color="blue", label="Translational")

# # ax1.set_aspect('equal', adjustable='box')
# ax1.grid()
# ax2 = fig.add_subplot(2, 1, 2)
# ax2.set_title("Rotational")
# ax2.set_xlabel('t')
# ax2.set_ylabel('rad/s')
# ax2.plot(time, angular, color="green", label='Rotational')
# # ax2.set_aspect('equal', adjustable='box')
# ax2.grid()
# plt.savefig( options.save_plot_path + "_odom_velocity.png" ) # 画像の保存
# # plt.show()
# plt.clf()

fig = plt.figure(figsize = (20, 15))
ax1 = fig.add_subplot(2, 1, 1) 
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

# ax1.set_title("data", color="blue")
ax1.set_xlabel("x[m]", size = 15)
ax1.set_ylabel("y[m]", size = 15)
# ax.set_title('Tracking and Robot Trajectory') # グラフタイトル
ax1.set_aspect('equal', adjustable='box') # スケールを揃える
ax1.grid()            # 罫線
ax1.plot(target_y, target_x, color="deepskyblue", linestyle='-', linewidth=5, label="Person")
ax1.plot(robot_y, robot_x, color="coral", linestyle='--', linewidth=5, label="Robot")
# obstacle
r = patches.Rectangle( (2.1,1.4) , 1.2, 0.9, fill=True, fc="Black", linewidth=0)
ax1.add_patch(r)
r = patches.Rectangle( (4.2,0.0) , 1.2, 0.9, fill=True, fc="Black", linewidth=0)
ax1.add_patch(r)
c = patches.Circle( xy=(7.05,1.85), radius=0.05,fill=True, fc="Black")
ax1.add_patch(c)
ax1.set_xlim([-1.0, 9.0]) # x方向の描画範囲を指定
ax1.set_ylim([0, 2.3])    # y方向の描画範囲を指定

ax1.legend(loc=0)    # 凡例
# fig.tight_layout()  # レイアウトの設定
# 軸目盛の設定と目盛り線の表示
plt.rcParams['font.size'] = 15
ax1.set_xticks([0.0, 0.9, 2.1, 3.3, 4.2, 5.4, 7.05, 8.4, -1.0, 9.0])
ax1.set_yticks([0.0, 0.9, 1.85, 1.15, 1.4, 0.0, 2.3])
xticklabels = ax1.get_xticklabels()
yticklabels = ax1.get_yticklabels()
# 各要素の設定をおこなうsetコマンド
ax1.set_xticklabels([0.0, 0.9, 2.1, 3.3, 4.2, 5.4, 7.05, 8.4, -1.0, 9.0],fontsize=15)
ax1.set_yticklabels([0.0, 0.9, 1.85, 1.15, 1.4, 0.0, 2.3],fontsize=15)
# plt.savefig( options.save_plot_path + "_tracking_and_robot.png" ) # 画像の保存

ax2 = fig.add_subplot(2, 1, 2) 
# ax2.set_title("env", color="blue")
# 軸目盛の設定と目盛り線の表示
plt.rcParams['font.size'] = 15
ax2.set_xticks([0.0, 0.9, 2.1, 3.3, 4.2, 5.4, 7.05, 8.4, -1.0, 9.0])
ax2.set_yticks([0.0, 0.9, 1.85, 1.15, 1.4, 0.0, 2.3])
xticklabels = ax2.get_xticklabels()
yticklabels = ax2.get_yticklabels()
# 各要素の設定をおこなうsetコマンド
ax2.set_xticklabels([0.0, 0.9, 2.1, 3.3, 4.2, 5.4, 7.05, 8.4, -1.0, 9.0],fontsize=15)
ax2.set_yticklabels([0.0, 0.9, 1.85, 1.15, 1.4, 0.0, 2.3],fontsize=15)
ax2.set_xlim([-1.0, 9.0]) # x方向の描画範囲を指定
ax2.set_ylim([0, 2.3])    # y方向の描画範囲を指定

# 軸ラベルの設定
ax2.set_xlabel("x[m]", size = 15)
ax2.set_ylabel("y[m]", size = 15)
# ax2.set_xlabel('x[m]')  # x軸ラベル
# ax2.set_ylabel('y[m]')  # y軸ラベル
# ax2.tick_params(direction = "inout", length = 5, colors = "#2a5caa")
ax2.grid()

# obstacle
r = patches.Rectangle( (2.1,1.4) , 1.2, 0.9, fill=True, fc="Black", linewidth=0, label="obstacle")
ax2.add_patch(r)
#ax2.text(2.7,1.85, "障害物", size=30, horizontalalignment="center", verticalalignment="center", color="White", fontfamily='TakaoGothic')
r = patches.Rectangle( (4.2,0.0) , 1.2, 0.9, fill=True, fc="Black", linewidth=0, label="obstacle")
ax2.add_patch(r)
#ax2.text(4.8,0.45, "障害物", size=30, horizontalalignment="center", verticalalignment="center", color="White", fontfamily='TakaoGothic')
# Person
c = patches.Circle( xy=(7.05,1.85), radius=0.05,fill=True, fc="Black")
ax2.add_patch(c)
# Robot
c = patches.Circle( xy=(0.0,0.2), radius=0.2,fill=True, fc="#2a5caa")
ax2.add_patch(c)
c = patches.Circle( xy=(0.0,0.475), radius=0.075,fill=True, fc="Black")
ax2.add_patch(c)
r = patches.Rectangle( (-0.125,0.15) , 0.25, 0.1, fill=True, fc="Black", linewidth=0, label="obstacle")
ax2.add_patch(r)
#ax2.text(0.0,0.68, "ロボット", size=30, horizontalalignment="center", verticalalignment="center", color="Black", fontfamily='TakaoGothic')
# Point
c = patches.Circle( xy=(0.9,1.15), radius=0.1,fill=True, fc="Green")
ax2.add_patch(c)
#ax2.text(0.9,0.9, "A地点", size=30, horizontalalignment="center", verticalalignment="center", color="Black", fontfamily='TakaoGothic')
c = patches.Circle( xy=(8.4,1.15), radius=0.1,fill=True, fc="Green")
ax2.add_patch(c)
#ax2.text(8.4,0.9, "B地点", size=30, horizontalalignment="center", verticalalignment="center", color="Black", fontfamily='TakaoGothic')
ax2.set_aspect('equal', adjustable='box') # スケールを揃える


plt.show()


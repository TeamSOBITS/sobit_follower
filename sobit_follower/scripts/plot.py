import pandas as pd
import matplotlib.pyplot as plt

# df = pd.read_csv('../csv/following_position.csv', encoding='shift-jis', index_col = 0 )
# x = df['field.x'].values.tolist()
# y = df['field.y'].values.tolist()

# # プロット
# plt.plot(y, x, label="test")
# plt.legend()
# plt.xlim(0, 2)
# plt.ylim(0, 2)
# plt.gca().set_aspect('equal', adjustable='box')
# plt.show()

df = pd.read_csv('../csv/velocity.csv', encoding='shift-jis' )
print(df)
time = df['%time'].values.tolist()
linear = df['field.linear.x'].values.tolist()
angular = df['field.angular.z'].values.tolist()

# プロット
plt.plot(time, linear, label="linear")
plt.plot(time, angular, label="angular")
plt.legend()
# plt.xlim(-2, 2)
# plt.ylim(-2, 2)
# plt.gca().set_aspect('equal', adjustable='box')
plt.show()
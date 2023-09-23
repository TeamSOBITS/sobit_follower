import matplotlib.pyplot as plt
from matplotlib import patches
import numpy as np
import matplotlib.font_manager
# # フォントを全て読み込み
# fonts = set([f.name for f in matplotlib.font_manager.fontManager.ttflist])
# # 描画領域のサイズ調整
# plt.figure(figsize=(10,len(fonts)/4))
# # フォントの表示
# for i, font in enumerate(fonts):
#     plt.text(0, i, f"日本語：{font}", fontname=font)
# # 見やすいように軸を消す
# plt.ylim(0, len(fonts))
# plt.axis("off")
# plt.show()
figsize_px = np.array([1920, 1080])
dpi = 100
figsize_inch = figsize_px / dpi

# figとaxの生成（グラフサイズを引数で設定）
fig, ax = plt.subplots(figsize=figsize_inch, dpi=dpi)
# 軸ラベルの設定
ax.set_xlabel("x[m]", size = 40)
ax.set_ylabel("y[m]", size = 40)
ax.set_aspect('equal', adjustable='box') # スケールを揃える
ax.grid()     

# obstacle
ob1 = patches.Rectangle( (0.85,1.4) , 0.9, 0.9, fill=True, fc="Black", linewidth=0)
ax.add_patch(ob1)
ob2 = patches.Rectangle( (2.4,0.0) , 0.9, 0.9, fill=True, fc="Black", linewidth=0)
ax.add_patch(ob2)
robot = patches.Circle( xy=(0.0,0.575), radius=0.05,fill=True, fc="Black")
ax.add_patch(robot)
target = patches.Circle( xy=(1.3,0.575), radius=0.05,fill=True, fc="Red")
ax.add_patch(target)
nt1 = patches.Circle( xy=(3.75,1.725), radius=0.05,fill=True, fc="Blue")
ax.add_patch(nt1)
nt2 = patches.Circle( xy=(4.75,0.575), radius=0.05,fill=True, fc="Blue")
ax.add_patch(nt2)
robot = patches.Circle( xy=(5.75, 1.15), radius=0.05,fill=True, fc="Black")
ax.add_patch(robot)

ax.legend(loc=0)    # 凡例

ax.set_xlim([0.0, 5.75]) # x方向の描画範囲を指定
ax.set_ylim([0, 2.3])    # y方向の描画範囲を指定

# 軸目盛の設定と目盛り線の表示
plt.rcParams['font.size'] = 20
ax.set_xticks([0.0, 0.85, 1.3, 1.75, 2.4, 3.3, 3.75, 4.75, 5.75])
ax.set_yticks([0.0, 0.575, 0.9, 1.15, 1.15, 1.4, 1.725, 2.3])
xticklabels = ax.get_xticklabels()
yticklabels = ax.get_yticklabels()
# 各要素の設定をおこなうsetコマンド
ax.set_xticklabels([0.0, 0.85, 1.3, 1.75, 2.4, 3.3, 3.75, 4.75, 5.75],fontsize=40)
ax.set_yticklabels([0.0, 0.575, 0.9, 1.15, 1.15, 1.4, 1.725, 2.3],fontsize=40)

plt.savefig("experiment_env.png" ) #

plt.clf()

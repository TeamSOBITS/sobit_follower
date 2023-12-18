import matplotlib.pyplot as plt
from matplotlib import patches
import numpy as np
import matplotlib.font_manager

# Generate fig and ax (set graph size by argument)
fig, ax = plt.subplots()

# Set axis scale and display scale lines
plt.rcParams['font.size'] = 20
ax.set_xticks([0.0, 0.9, 2.1, 3.3, 4.2, 5.4, 7.05, 8.4, -1.0, 9.0])
ax.set_yticks([0.0, 0.9, 1.85, 1.15, 1.4, 0.0, 2.3])
xticklabels = ax.get_xticklabels()
yticklabels = ax.get_yticklabels()
# set command to set each element
ax.set_xticklabels([0.0, 0.9, 2.1, 3.3, 4.2, 5.4, 7.05, 8.4, -1.0, 9.0],fontsize=20)
ax.set_yticklabels([0.0, 0.9, 1.85, 1.15, 1.4, 0.0, 2.3],fontsize=20)
ax.set_xlim([-1.0, 9.0]) # Specify drawing range in x-direction
ax.set_ylim([0, 2.3])    # Specify drawing range in y direction

# Set axis labels
ax.set_xlabel("x[m]", size = 20)
ax.set_ylabel("y[m]", size = 20)
# ax.set_xlabel('x[m]')  # x-axis label
# ax.set_ylabel('y[m]')  # y-axis label
# ax.tick_params(direction = "inout", length = 5, colors = "#2a5caa")
ax.grid()

# obstacle
r = patches.Rectangle( (2.1,1.4) , 1.2, 0.9, fill=True, fc="Black", linewidth=0, label="obstacle")
ax.add_patch(r)
#ax.text(2.7,1.85, "障害物", size=30, horizontalalignment="center", verticalalignment="center", color="White", fontfamily='TakaoGothic')
r = patches.Rectangle( (4.2,0.0) , 1.2, 0.9, fill=True, fc="Black", linewidth=0, label="obstacle")
ax.add_patch(r)
#ax.text(4.8,0.45, "障害物", size=30, horizontalalignment="center", verticalalignment="center", color="White", fontfamily='TakaoGothic')
# Person
c = patches.Circle( xy=(7.05,1.85), radius=0.05,fill=True, fc="Black")
ax.add_patch(c)
# Robot
c = patches.Circle( xy=(0.0,0.2), radius=0.2,fill=True, fc="#2a5caa")
ax.add_patch(c)
c = patches.Circle( xy=(0.0,0.475), radius=0.075,fill=True, fc="Black")
ax.add_patch(c)
r = patches.Rectangle( (-0.125,0.15) , 0.25, 0.1, fill=True, fc="Black", linewidth=0, label="obstacle")
ax.add_patch(r)
#ax.text(0.0,0.68, "ロボット", size=30, horizontalalignment="center", verticalalignment="center", color="Black", fontfamily='TakaoGothic')
# Point
c = patches.Circle( xy=(0.9,1.15), radius=0.1,fill=True, fc="Green")
ax.add_patch(c)
#ax.text(0.9,0.9, "A地点", size=30, horizontalalignment="center", verticalalignment="center", color="Black", fontfamily='TakaoGothic')
c = patches.Circle( xy=(8.4,1.15), radius=0.1,fill=True, fc="Green")
ax.add_patch(c)
#ax.text(8.4,0.9, "B地点", size=30, horizontalalignment="center", verticalalignment="center", color="Black", fontfamily='TakaoGothic')
ax.set_aspect('equal', adjustable='box') # Align scale
fig.tight_layout()  # Set the layout
# plt.legend()
plt.show()
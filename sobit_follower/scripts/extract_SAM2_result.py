import os
import rosbag
from cv_bridge import CvBridge
import cv2
import matplotlib.pyplot as plt
import argparse

# Initialize CvBridge
bridge = CvBridge()

# Argument Parsing
parser = argparse.ArgumentParser()
parser.add_argument("--plate_bag_path", default='', help="File path for plate_top_camera bag file")
parser.add_argument("--mask_bag_path", default='', help="File path for sam2_nontravelable_region_mask bag file")
parser.add_argument("--save_plot_path", default='', help="Directory path to save side-by-side visualizations")
options = parser.parse_args()

# Output directory for visualizations
plot_output_dir = os.path.join(options.save_plot_path, 'sam2_side_by_side')
os.makedirs(plot_output_dir, exist_ok=True)

# Extract Plate Top Camera Frames with Timestamps
print("Extracting frames with timestamps from /plate_top_camera/color/image_raw...")
plate_frames = []
plate_timestamps = []
with rosbag.Bag(options.plate_bag_path, 'r') as bag:
    for topic, msg, t in bag.read_messages(topics=['/plate_top_camera/color/image_raw']):
        plate_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        plate_frames.append(plate_image)
        plate_timestamps.append(t.to_sec())  # Store timestamp in seconds

# Extract SAM2 Mask Frames with Timestamps
print("Extracting frames with timestamps from /sam2_nontravelable_region_mask...")
mask_frames = []
mask_timestamps = []
with rosbag.Bag(options.mask_bag_path, 'r') as bag:
    for topic, msg, t in bag.read_messages(topics=['/sam2_nontravelable_region_mask']):
        mask_image = bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        mask_frames.append(mask_image)
        mask_timestamps.append(t.to_sec())  # Store timestamp in seconds

# Synchronize based on mask timestamps (only consider timestamps where mask is available)
print("Matching /sam2_nontravelable_region_mask frames with closest /plate_top_camera/color/image_raw frames...")
synchronized_pairs = []
for mask_time, mask_img in zip(mask_timestamps, mask_frames):
    closest_index = min(range(len(plate_timestamps)), key=lambda i: abs(plate_timestamps[i] - mask_time))
    plate_img = plate_frames[closest_index]
    synchronized_pairs.append((plate_img, mask_img))

# Plot each pair of synchronized frames side-by-side
print("Generating side-by-side plots for synchronized frames...")
for i, (plate_img, mask_img) in enumerate(synchronized_pairs):
    # Resize mask image to match plate image size if needed
    mask_img = cv2.resize(mask_img, (plate_img.shape[1], plate_img.shape[0]))

    # Plot both images side-by-side
    fig, axs = plt.subplots(1, 2, figsize=(12, 6))
    
    # Left: Plate Camera Image
    axs[0].imshow(cv2.cvtColor(plate_img, cv2.COLOR_BGR2RGB))
    axs[0].set_title('RGB Input', fontsize=16)
    axs[0].axis('off')

    # Right: SAM2 Mask Image
    axs[1].imshow(mask_img, cmap='gray')
    axs[1].set_title('SAM 2 Mask', fontsize=16)
    axs[1].axis('off')

    # Overall Title
    fig.suptitle(f'Non-travelable Region by SAM 2 - Frame {i}', fontsize=20)

    # Save only side-by-side plot
    plot_file = os.path.join(plot_output_dir, f"frame_{i:04d}.png")
    plt.tight_layout()
    plt.savefig(plot_file)
    plt.close()

print(f"All side-by-side plots have been saved in:\n- {plot_output_dir}")

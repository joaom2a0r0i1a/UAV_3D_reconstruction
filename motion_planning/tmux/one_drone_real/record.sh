#!/bin/bash

path="/home/\$(optenv USER mrs)/bag_files/latest/"

# By default, we record everything.
# Except for this list of EXCLUDED topics:
exclude=(

/camera/align_to_color/parameter_descriptions
/camera/align_to_color/parameter_updates
/camera/aligned_depth_to_color/camera_info
#/camera/aligned_depth_to_color/image_raw
/camera/aligned_depth_to_color/image_raw/compressed
/camera/aligned_depth_to_color/image_raw/compressed/parameter_descriptions
/camera/aligned_depth_to_color/image_raw/compressed/parameter_updates
/camera/aligned_depth_to_color/image_raw/compressedDepth
/camera/aligned_depth_to_color/image_raw/compressedDepth/parameter_descriptions
/camera/aligned_depth_to_color/image_raw/compressedDepth/parameter_updates
/camera/aligned_depth_to_color/image_raw/theora
/camera/aligned_depth_to_color/image_raw/theora/parameter_descriptions
/camera/aligned_depth_to_color/image_raw/theora/parameter_updates
#/camera/color/camera_info
#/camera/color/image_raw
/camera/color/image_raw/compressed
/camera/color/image_raw/compressed/parameter_descriptions
/camera/color/image_raw/compressed/parameter_updates
/camera/color/image_raw/compressedDepth/parameter_descriptions
/camera/color/image_raw/compressedDepth/parameter_updates
/camera/color/image_raw/theora
/camera/color/image_raw/theora/parameter_descriptions
/camera/color/image_raw/theora/parameter_updates
/camera/color/metadata   
/camera/decimation/parameter_descriptions
/camera/decimation/parameter_updates
/camera/depth/camera_info
/camera/depth/image_rect_raw
/camera/depth/image_rect_raw/compressed
/camera/depth/image_rect_raw/compressed/parameter_descriptions
/camera/depth/image_rect_raw/compressed/parameter_updates
/camera/depth/image_rect_raw/compressedDepth
/camera/depth/image_rect_raw/compressedDepth/parameter_descriptions
/camera/depth/image_rect_raw/compressedDepth/parameter_updates
/camera/depth/image_rect_raw/theora
/camera/depth/image_rect_raw/theora/parameter_descriptions
/camera/depth/image_rect_raw/theora/parameter_updates
/camera/depth/metadata
/camera/extrinsics/depth_to_color
/camera/motion_module/parameter_descriptions
/camera/motion_module/parameter_updates
/camera/realsense2_camera_manager/bond
/camera/rgb_camera/auto_exposure_roi/parameter_descriptions
/camera/rgb_camera/auto_exposure_roi/parameter_updates
/camera/rgb_camera/parameter_descriptions
/camera/rgb_camera/parameter_updates
/camera/spatial/parameter_descriptions
/camera/spatial/parameter_updates
/camera/stereo_module/auto_exposure_roi/parameter_descriptions
/camera/stereo_module/auto_exposure_roi/parameter_updates
/camera/stereo_module/parameter_descriptions
/camera/stereo_module/parameter_updates
/camera/temporal/parameter_descriptions
/camera/temporal/parameter_updates

#/uav2/planner_node/surface_pointcloud
#/uav2/planner_node/tsdf_pointcloud
#/uav2/planner_node/occupied_nodes
#/uav2/planner_node/tsdf_slice
#/uav2/planner_node/mesh
#/uav2/planner_node/tsdf_map_out
#/uav2/planner_node/esdf_pointcloud
#/uav2/planner_node/esdf_slice
#/uav2/planner_node/traversable
#/uav2/planner_node/esdf_map_out
#/uav2/visualization_marker_out
#/uav2/simulation_ready
#/uav2/frustum_out
#/uav2/unknown_voxels_out

#(.*)voxblox_node(.*)
         

# IN GENERAL, DON'T RECORD CAMERAS
#
# If you want to record cameras, create a copy of this script
# and place it at your tmux session.
#
# Please, seek an advice of a senior researcher of MRS about
# what can be recorded. Recording too much data can lead to
# ROS communication hiccups, which can lead to eland, failsafe
# or just a CRASH.

# Every topic containing "compressed"
# '(.*)compressed(.*)'
# Every topic containing "image_raw"
# '(.*)image_raw(.*)'
# Every topic containing "theora"
# '(.*)theora(.*)'
# Every topic containing "h264"
# '(.*)h264(.*)'

)

# file's header
filename=`mktemp`
echo "<launch>" > "$filename"
echo "<arg name=\"UAV_NAME\" default=\"\$(env UAV_NAME)\" />" >> "$filename"
echo "<group ns=\"\$(arg UAV_NAME)\">" >> "$filename"

echo -n "<node pkg=\"rosbag\" type=\"record\" name=\"rosbag_record\" output=\"screen\" args=\"--lz4 -o $path -a" >> "$filename"

# if there is anything to exclude
if [ "${#exclude[*]}" -gt 0 ]; then

  echo -n " -x " >> "$filename"

  # list all the string and separate the with |
  for ((i=0; i < ${#exclude[*]}; i++));
  do
    echo -n "${exclude[$i]}" >> "$filename"
    if [ "$i" -lt "$( expr ${#exclude[*]} - 1)" ]; then
      echo -n "|" >> "$filename"
    fi
  done

fi

echo "\">" >> "$filename"

echo "<remap from=\"~status_msg_out\" to=\"mrs_uav_status/display_string\" />" >> "$filename"
echo "<remap from=\"~data_rate_out\" to=\"~data_rate_MB_per_s\" />" >> "$filename"

# file's footer
echo "</node>" >> "$filename"
echo "</group>" >> "$filename"
echo "</launch>" >> "$filename"

cat $filename
roslaunch $filename

#! /bin/sh
#
# commit be5f6cca6b3963af69c18b84e7c7e064a2910c9c
# Date:   Wed Jun 1 21:26:58 2016 +0900
#

if [ $# -gt 1 -o "$1" = "-h" -o "$1" = "--help" ]; then
  echo "Usage: $0 [data_path]"
  exit 1
fi

DIR=my_launch
if [ -d "$DIR" ]; then
  BKDIR="${DIR}.$(date +%Y%m%d%H%M%S)"
  echo "$DIR is already exists, move to $BKDIR"
  mv -f $DIR $BKDIR
fi
mkdir $DIR

DATA_DIR=$HOME/.autoware/data
[ $# -eq 1 ] && DATA_DIR="$1"

#
# MAP
#
LAUNCH=$DIR/my_map.launch

PMAP_PRM=noupdate
PCDS=$(find $DATA_DIR/map/pointcloud_map -name '*.pcd' | paste -d ' ' -s)
CSVS=$(find $DATA_DIR/map/vector_map -name '*.csv' | paste -d ' ' -s)

cat > $LAUNCH <<EOF
<launch>

  <!-- TF -->
  <include file="$DATA_DIR/tf/tf.launch"/>

  <!-- Point Cloud -->
  <node pkg="map_file" type="points_map_loader" name="points_map_loader" args="$PMAP_PRM $PCDS"/>

  <!-- Vector Map -->
  <node pkg="map_file" type="vector_map_loader" name="vector_map_loader" args="$CSVS"/>

</launch>
EOF

#chmod a+x $LAUNCH
ls -l $LAUNCH

#
# Sensing
#
LAUNCH=$DIR/my_sensing.launch

cat > $LAUNCH <<EOF
<launch>

  <!-- calibration file path -->
  <arg name="velodyne_calib" default="\$(find velodyne_pointcloud)/params/32db.yaml"/>
  <arg name="camera_calib" default="$DATA_DIR/calibration/camera_lidar_3d/prius/nic-150407.yml"/>

  <!-- HDL-32e -->
  <include file="\$(find velodyne_pointcloud)/launch/velodyne_hdl32e.launch">
    <arg name="calibration" value="\$(arg velodyne_calib)"/>
  </include>

  <!-- Javad Delta 3 -->
  <!-- <node pkg="javad_navsat_driver" type="gnss.sh" name="javad_driver"/> -->

  <!-- PointGrey Grasshopper3 -->
  <include file="\$(find pointgrey)/scripts/grasshopper3.launch">
    <arg name="CalibrationFile" value="\$(arg camera_calib)"/>
  </include>

</launch>
EOF

#chmod a+x $LAUNCH
ls -l $LAUNCH

#
# Localization
#
LAUNCH=$DIR/my_localization.launch

cat > $LAUNCH <<EOF
<launch>

  <!-- setting path parameter -->
  <arg name="use_openmp" value="false" />
  <arg name="get_height" value="true" />

  <!-- Setup -->
  <include file="\$(find runtime_manager)/scripts/setup_tf.launch">
    <arg name="x" value="1.2" />
    <arg name="y" value="0.0" />
    <arg name="z" value="2.0" />
    <arg name="yaw" value="0.0" />
    <arg name="pitch" value="0.0" />
    <arg name="roll" value="0.0" />
    <arg name="frame_id" value="/base_link" />
    <arg name="child_frame_id" value="/velodyne" />
    <arg name="period_in_ms" value="10"/>
  </include>
  <include file="\$(find model_publisher)/launch/vehicle_model.launch" />

  <!-- points downsampler -->
  <include file="\$(find points_downsampler)/launch/points_downsample.launch" />

  <!-- nmea2tfpose -->
  <include file="\$(find gnss_localizer)/launch/nmea2tfpose.launch"/>

  <!-- ndt_matching -->
  <include file="\$(find ndt_localizer)/launch/ndt_matching.launch">
    <arg name="use_openmp" value="\$(arg use_openmp)" />
    <arg name="get_height" value="\$(arg get_height)" />
  </include>

</launch>
EOF

#chmod a+x $LAUNCH
ls -l $LAUNCH

#
# Detection
#
LAUNCH=$DIR/my_detection.launch

cat > $LAUNCH <<EOF
<launch>

  <!-- setting of this launch file -->
  <arg name="car_detection" default="true" />
  <arg name="pedestrian_detection" default="false" />
  <arg name="is_use_gpu" default="true" />
  <arg name="is_register_lidar2camera_tf" default="true" />
  <arg name="is_publish_projection_matrix" default="true" />
  <arg name="is_publish_camera_info" default="true" />
  <arg name="camera_calib" default="$DATA_DIR/calibration/camera_lidar_3d/prius/nic-150407.yml"/>

  <!-- calibration_publisher -->
  <include file="\$(find runtime_manager)/scripts/calibration_publisher.launch">
    <arg name="file" value="\$(arg camera_calib)" />
    <arg name="register_lidar2camera_tf" value="\$(arg is_register_lidar2camera_tf)" />
    <arg name="publish_extrinsic_mat" value="\$(arg is_publish_projection_matrix)" />
    <arg name="publish_camera_info" value="\$(arg is_publish_camera_info)" />
  </include>

  <!-- points2image -->
  <node pkg="points2image" type="points2image" name="points2image" />

  <!-- car and pedestrian detection -->
  <!-- dpm_XXX -->
  <include file="\$(find cv_tracker)/launch/dpm_ttic.launch">
    <arg name="car" value="\$(arg car_detection)" />
    <arg name="pedestrian" value="\$(arg pedestrian_detection)" />
    <arg name="use_gpu" value="\$(arg is_use_gpu)" />
  </include>

  <!-- range_fusion -->
  <include file="\$(find cv_tracker)/launch/ranging.launch">
    <arg name="car" value="\$(arg car_detection)" />
    <arg name="pedestrian" value="\$(arg pedestrian_detection)" />
  </include>

  <!-- XXX_track -->
  <include file="\$(find cv_tracker)/launch/klt_tracking.launch">
    <arg name="car" value="\$(arg car_detection)" />
    <arg name="pedestrian" value="\$(arg pedestrian_detection)" />
  </include>

  <!-- obj_reproj -->
  <include file="\$(find cv_tracker)/launch/reprojection.launch">
    <arg name="car" value="\$(arg car_detection)" />
    <arg name="pedestrian" value="\$(arg pedestrian_detection)" />
  </include>

  <!-- euclidean_cluster -->
  <include file="\$(find lidar_tracker)/launch/euclidean_clustering.launch">
  </include>

  <!-- obj_fusion -->
  <include file="\$(find lidar_tracker)/launch/obj_fusion.launch">
    <arg name="car" value="\$(arg car_detection)" />
    <arg name="pedestrian" value="\$(arg pedestrian_detection)" />
  </include>


  <!-- traffic light recognition -->
  <!-- feat_proj -->
  <!--
  <node pkg="road_wizard" type="feat_proj" name="feat_proj" />
  -->

  <!-- region_tlr -->
  <!--
  <include file="\$(find road_wizard)/launch/traffic_light_recognition.launch" />
  -->

</launch>
EOF

#chmod a+x $LAUNCH
ls -l $LAUNCH

#
# Mission Planning
#
LAUNCH=$DIR/my_mission_planning.launch

cat > $LAUNCH <<EOF
<launch>

  <!-- setting path parameter -->
  <arg name="multi_lane_csv" default="$DATA_DIR/path/moriyama_path.txt" />
  <arg name="topic_pose_stamped" default="/ndt_pose" />
  <arg name="topic_twist_stamped" default="/estimate_twist" />

  <!-- Tablet UI -->
  <!--
  <include file="\$(find runtime_manager)/scripts/tablet_socket.launch"/>
  -->

  <!-- vel_pose_mux -->
  <include file="\$(find autoware_connector)/launch/vel_pose_connect.launch">
    <arg name="topic_pose_stamped" value="\$(arg topic_pose_stamped)" />
    <arg name="topic_twist_stamped" value="\$(arg topic_twist_stamped)" />
  </include>

  <!-- waypoint_loader -->
  <include file="\$(find waypoint_maker)/launch/waypoint_loader.launch">
    <arg name="multi_lane_csv" value="\$(arg multi_lane_csv)"/>
  </include>

  <!-- lane_navi -->
  <!--
  <node pkg="lane_planner" type="lane_navi" name="lane_navi" />
  -->

  <!-- lane_rule -->
  <node pkg="lane_planner" type="lane_rule" name="lane_rule" />

  <!-- lane_stop -->
  <node pkg="lane_planner" type="lane_stop" name="lane_stop" />

  <!-- lane_select -->
  <node pkg="lane_planner" type="lane_select" name="lane_select" />

</launch>
EOF

#chmod a+x $LAUNCH
ls -l $LAUNCH

#
# Motion Planning
#
LAUNCH=$DIR/my_motion_planning.launch

cat > $LAUNCH <<EOF
<launch>

  <!-- Vehicle Contorl -->
  <include file="\$(find runtime_manager)/scripts/vehicle_socket.launch"/>

  <!-- obstacle_avoid -->
  <include file="\$(find astar_planner)/launch/obstacle_avoid.launch"/>

  <!-- velocity_set -->
  <include file="\$(find astar_planner)/launch/velocity_set.launch"/>

  <!-- pure_pursuit -->
  <node pkg="rostopic" type="rostopic" name="config_waypoint_follower_rostopic"
        args="pub -l /config/waypoint_follower autoware_msgs/ConfigWaypointFollower
        '{ header: auto, param_flag: 1, velocity: 5.0, lookahead_distance: 4.0, lookahead_ratio: 2.0, minimum_lookahead_distance: 6.0, displacement_threshold: 0.0, relative_angle_threshold: 0.0 }' "
  />
  <include file="\$(find waypoint_follower)/launch/pure_pursuit.launch"/>

  <!-- twist_filter -->
  <include file="\$(find waypoint_follower)/launch/twist_filter.launch"/>

</launch>
EOF

#chmod a+x $LAUNCH
ls -l $LAUNCH

# EOF
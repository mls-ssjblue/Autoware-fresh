^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package lidar_tracker
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.6.3 (2018-03-06)
------------------

1.6.2 (2018-02-27)
------------------
* Update CHANGELOG
* Contributors: Yusuke FUJII

1.6.1 (2018-01-20)
------------------
* update CHANGELOG
* fix build error with OpenCV 3.3
* Contributors: Yamato ANDO, Yusuke FUJII

1.6.0 (2017-12-11)
------------------
* Prepare release for 1.6.0
* use qt5_wrap_ui instead of autouic
* - Test Tracking
  - Add missing parameter to lunch file and runtime manager
* - Add new Node for object polygon representation and tracking (kf_contour_tracker)
  - Add launch file and tune tracking parameters
  - Test with Moriyama rosbag
* refactor CMakeLists.txt. use automoc, autouic and autorcc
* Contributors: Yamato ANDO, hatem-darweesh

1.5.1 (2017-09-25)
------------------
* Release/1.5.1 (`#816 <https://github.com/cpfl/autoware/issues/816>`_)
  * fix a build error by gcc version
  * fix build error for older indigo version
  * update changelog for v1.5.1
  * 1.5.1
* Contributors: Yusuke FUJII

1.5.0 (2017-09-21)
------------------
* Update changelog
* [hotfix] fixes to lidar_tracker package(`#787 <https://github.com/cpfl/autoware/issues/787>`_)
  -Fixed a typo in the ground_filter launch file from points_preprocessor
  -Fixed ID duplication in kf_lidar_tracker
  Tested on Ubuntu 14.04 and 16.04
* ROS_WARN_ONCE " VectorMap Server Call failed"
* Add vector_map_server to RuntimeManager  `#722 <https://github.com/cpfl/autoware/issues/722>`_
* Contributors: Abraham Monrroy, Yusuke FUJII, andoh104

1.4.0 (2017-08-04)
------------------
* Add an enum type for object labels
* Refactored by pullreq feedbacks and roscpp-code-format
* version number must equal current release number so we can start releasing in the future
* added changelogs
* Fix coodinate bug and small refactor
* Change time-sync, flag/mutex -> sync_policies::ApproximateTime
* Add visualization/rosparam and some refactoring
* Publishing unique IDs (`#740 <https://github.com/cpfl/autoware/issues/740>`_)
  Checked against 14.04 and 16.04
* Add axis and adjust the direction.
* Contributors: Abraham Monrroy, Akihito OHSATO, Dejan Pangercic, USUDA Hisashi

1.3.1 (2017-07-16)
------------------

1.3.0 (2017-07-14)
------------------
* fix build issues due to autoware_msgs
* Remove timers
* Add ROS_INFO that print execution time of euclidean clustering
* replace ',' into && at for sentence
* Add selecter of GPU for euclidean clustering
* Fixes for GPU Euclidean clustering
* convert to autoware_msgs
* Removed the chance of a deadlock on extreme cases when points are far from each other
* Removed the chance of a deadlock on extreme cases when points are far from each other
* Added GPU support for Euclidean Clustering
* Contributors: AMC, YamatoAndo, Yusuke FUJII, yukitsuji

1.2.0 (2017-06-07)
------------------
* fixed build issues
* Add OpenPlanner to Develop Branch, add OpenPlanner to Runtime Manager, and modify rviz default config file
  fix map loading options
  automatic replanning simulation and traffic light stop and go
  add performance logging
  behavior state for traffic light and stop signs fixed
  fix logging shift, fix euclidean clusters problem
  visualize dp steps
  detection config for robot vel16
  tune ff path follower for simulation
  tune ff path follower for simulation
  HMI update
  simulated obstacle bounding box representation
  HMI Update
  HMI Successful Demo
  improve detection accuracy to < 10 cm
  HMI Tested. More runtime manager options.
  HMI Tested. More runtime manager options.
  fix dp plan build issue
  Controller - Steering Delay auto calibration
  Multi-Traffic Behavior Simulation on Rviz using OpenPlanner
  change node names to match ROS naming standards
  change node names to match ROS naming standards
  - Add OpenPlanner Vehicle Simulator
  - Integrate with Autoware's pure pursut
  - Revised local planning
  - Unit-Test usig playback based simulation
  update simulation launch files
  More Unit Testing
  Improve Object Tracking
  CAN info message handle!
  rviz config
  visualization changes
  add option to select velocities source
  RS Planner Test
* Fixed compatibility issues with indigo
* Removed unnecesary code
* KF Lidar Tracker
* fix circular-dependency
* Fix library dependency
* ROS Kinectic Upgrade tested on Ubuntu 16.04 and OpenCV 3.2.0
  Modules not included:
  -orb_localizer
  -dpm_ocv node and lib
  Everything else working
  Added some libraries for Gazebo on src/extras
* Update for kinetic
* Kalman Filter files
  MOT files
* Kf Added
  Euclidean Cluster improved
* Fixes
* svm probilities added
* Added SVm Classifier
* changes
* Added FPFH descriptor calculation
* Time measurement for VectorMap Server
* Added VectorMap Server support
* Switching to VectorMap Service
* Lidar tracker restructuration
* VectorMap
* Cleaning
* Fixed setting frame order for pcl
* Added output_frame param
  Allows to transform output coordinate frame of the bounding boxes and CloudClusterArray messages
* Publishind PCA's Eigen vectors/values for each cluster
* Changed to PointStamped
* Added CloudCluster and CloudClusterArray Message
  Clusters and its feats can now be accessed from outside the clustering node.
  Refer to the messages definition
* Added Ground removal as optional.
  Removed unused params
* Parametrization of Clustering params
* Fix on var name
* Added Cluster class
* Added Cluster class
* Integrated Vscan tracker with Euclidean clustering
* Update .gitignore to ignore automatic generated file
* Add launch file to boot vehicle-tracker
* Switch tracking input from obj_pose to obj_label
  obj_pose : fused data between reprojection result and clustered vscan
  obj_label: reprojection result
* Get init position of object from subscribed obj_pose topic
* Make moc file dependencies explicit
  And no need to specify "arch=compute_XXX" option for NVCC
* changed frame name to velodyne
* -Now it compiles
  -Modified CMakeFile to use only CMake commands instead of custom commands
* changes
* cmake update
* testing
* Port vehicle_tracker into Autoware
  This porting is incomplete
* Defined new message for vscan tracking's result
* Update .gitignore to ignore automatic generated file
* Add launch file to boot vehicle-tracker
* Switch tracking input from obj_pose to obj_label
  obj_pose : fused data between reprojection result and clustered vscan
  obj_label: reprojection result
* Get init position of object from subscribed obj_pose topic
* Make moc file dependencies explicit
  And no need to specify "arch=compute_XXX" option for NVCC
* changed frame name to velodyne
* -Now it compiles
  -Modified CMakeFile to use only CMake commands instead of custom commands
* changes
* cmake update
* testing
* Port vehicle_tracker into Autoware
  This porting is incomplete
* Defined new message for vscan tracking's result
* Contributors: AMC, Manato Hirabayashi, Shohei Fujii, Yukihiro Saito, Yusuke FUJII, Yusuke Fujii, amc-nu, hatem-darweesh

1.1.2 (2017-02-27 23:10)
------------------------

1.1.1 (2017-02-27 22:25)
------------------------

1.1.0 (2017-02-24)
------------------
* Change topic name
* Fixed a bug. It is don't publish when subscribing topic data is empty.
* Add euclidean lidar track
* Change to use cloud_cluster node in obj fusion
* Contributors: Yukihiro Saito

1.0.1 (2017-01-14)
------------------

1.0.0 (2016-12-22)
------------------
* Defaults for unset CUDA_CAPABILITY_VERSION
  If CUDA_CAPABILITY_VERSION is empty, then make gets input of arch `sm\_`
  which causes a failure in compilation
  - This allows for a fallback that is similar to rest of the file.
  Closes `#536 <https://github.com/cpfl/autoware/issues/536>`_
* Fix .gitignore in each packages
* Removing "UpTo" points from all the point cloud topics
* Added param to ignore points closer than a threshold
* Lidar segmentation (`#499 <https://github.com/cpfl/autoware/issues/499>`_)
  * Lidar tracker restructuration
  * Added points_preprocessor package, including; ground filtering and space filtering.
* Lidar segmentation (`#490 <https://github.com/cpfl/autoware/issues/490>`_)
  Fixed setting frame order for pc
  Cleaning
* Added output_frame param
  Allows to transform output coordinate frame of the bounding boxes and CloudClusterArray messages
* Lidar segmentation (`#486 <https://github.com/cpfl/autoware/issues/486>`_)
  Added CloudCluster and CloudClusterArray Message
  Clusters and its feats can now be accessed from outside the clustering node.
  Refer to the messages definition
* Lidar segmentation (`#482 <https://github.com/cpfl/autoware/issues/482>`_)
  * Added Cluster class
  * Parametrization of Clustering params
* Added params for Cloud clipping
  fixed bug in segment by distance
* Added
  RuntimeManager control for Euclidean clustering
  Distance based threshold for clusteringd
* Added BoundingBox angle estimation
* Added params to Launch file
* Difference of Normals Segmentation added to the pipeline
* Code cleaning
* Added BoundingBox angle estimation
* Added params to Launch file
* Difference of Normals Segmentation added to the pipeline
* Code cleaning
* Code cleaning
* Accelerated obj_fusion
* Add module graph tool
* Add a text label with a object pose
* modify obj_fusion andobj_reproj in order to use tracking ID
* Don't publish non message object
  This causes build error on debug-building.
* Fix for rosjava installed platform
  Some packages don't declare package dependencies correctly.
  This makes message jar files built failure.
* Add sleep command to decrease CPU occupancy
* modify launch files in perception to add a pedestrian mode in the sync packege
* modify correct timestamp and timing to publish
* Runtime Manager Computing tab, add Synchronization button
* Add timestamp topic to obj_fusion
* Add topic publishing function to obj_fusion
  This function is called immediately
  when both of source topics of obj_pose are subscribed
* Add flags to confirm multiple topics are subscribed
  - When topic's callback is called, corresponding flag is turned true
  - Result topic is published only when all flags are true
* Some fix
* Accelerate euclidean_cluster
  - Add paramter for precision and throughput tuning
  - Add down sampling process (selectable from paramater)
  - Pass filtered pointcloud to clustering process
* Use c++11 option instead of c++0x
  We can use newer compilers which support 'c++11' option
* Update euclidean_clustering.launch
* as pointed by Yosh
  regarding the topics name, I'm just following the convention used in the file. (ie. "/points_cluster")
* -Modified euclidean clustering to:
  1. publish new topic "/points_ground" of the type sensor_msgs::PointCloud2, outputs the planar points in the ground
  2. publish new topic "/points_filtered" of the type sensor_msgs::PointCloud2, removes the planar points from points_raw
  Both of the added features feed from the customizable 'points_node' argument.
  Please check the launch file for details.
  The idea is to generate different pointcloud messages to be projected using the new points2image.
  Example:
  1. Generate the PC messages
  % roslaunch lidar_tracker euclidean_clustering.launch
  This will publish 3 topics, /euclidean_clustering, /points_filtered, /points_ground
  2. Launch calibration_publisher
  3. Launch points2image to show the projected result from the desired PC message
  For instance:
  % rosrun points2image points2image _points_node:=/points_filtered
  or
  % rosrun points2image points2image _points_node:=/points_ground
  etc...
  4. finally :
  % rosrun viewers points_image_viewer
  Any PointCloud2 Message --->  Points2Image --->  Viewer
* Initial commit for public release
* Contributors: AMC, Abraham, Abraham Monrroy, Hiroki Ohta, Manato Hirabayashi, Shinpei Kato, Syohei YOSHIDA, Tushar Dadlani, USUDA Hisashi, Yukihiro Saito, h_ohta, kondoh, pdsljp

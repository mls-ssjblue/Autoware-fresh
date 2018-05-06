^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package points2image
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.6.3 (2018-03-06)
------------------

1.6.2 (2018-02-27)
------------------
* Update CHANGELOG
* Contributors: Yusuke FUJII

1.6.1 (2018-01-20)
------------------
* update CHANGELOG
* Contributors: Yusuke FUJII

1.6.0 (2017-12-11)
------------------
* Prepare release for 1.6.0
* use qt5_wrap_ui instead of autouic
* Fix feature/points2image bug multicam support (`#886 <https://github.com/cpfl/autoware/issues/886>`_)
  * pointgrey
  * Added New Calibration node
  * Added parameters, plane fitting
  * added mirror node, etc
  * Points2Image
  Calibration Publisher
  now works with multiple cameras using ros namespaces
  * Including only points2image
  * Added Launch file for points2 image specific for the ladybug camera
* refactor CMakeLists.txt. use automoc, autouic and autorcc
* Contributors: Abraham Monrroy, Yamato ANDO

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
* Contributors: Yusuke FUJII

1.4.0 (2017-08-04)
------------------
* version number must equal current release number so we can start releasing in the future
* added changelogs
* Contributors: Dejan Pangercic

1.3.1 (2017-07-16)
------------------

1.3.0 (2017-07-14)
------------------
* convert to autoware_msgs
* Contributors: YamatoAndo

1.2.0 (2017-06-07)
------------------
* fix circular-dependency
* Fixes
* Contributors: AMC, Shohei Fujii

1.1.2 (2017-02-27 23:10)
------------------------

1.1.1 (2017-02-27 22:25)
------------------------

1.1.0 (2017-02-24)
------------------

1.0.1 (2017-01-14)
------------------

1.0.0 (2016-12-22)
------------------
* Add module graph tool
* Add missing dependencies
* Remove needless compiling flags
* Fix for rosjava installed platform
  Some packages don't declare package dependencies correctly.
  This makes message jar files built failure.
* Change a subscribing topic  in points2image and vscan2points when synchronization is enabled
* Correct calibration_camera_lidar dependnecy about message header
* Correct dependency name
* Delete image size fixing
* Ros-parameterize all defined value in points2vscan
  Now we can specify parameters value from launch file
* Update threshold
  So that vscan can work for close-range obstacles when velodyne 32 is
  used
* Make projection matrix source selectable
  I modified nodes that subscribe /projection_matrix
  so that we can specify the topic name from launch file
* Make camera_info source selectable
  I modified nodes that subscribe /camera/camera_info
  so that we can specify the topic name from launch file
* Clean up Qt5 configuration
  Use pkg-config as possible instead of absolute pathes.
* Use c++11 option instead of c++0x
  We can use newer compilers which support 'c++11' option
* Updated point2image to set minh in the message
* updated fusion to optionally read from any points to image projected topic via argument points_node.
  default  topic vscan_image (not changed)
  updated points2image topic to optionally project any pointcloud2 topic via argu
  ment point_node.
  default topic: points_raw (not changed)
* Initial commit for public release
* Contributors: AMC, Manato Hirabayashi, Shinpei Kato, Syohei YOSHIDA, USUDA Hisashi, Yukihiro Saito

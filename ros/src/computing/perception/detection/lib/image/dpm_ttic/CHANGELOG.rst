^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package libdpm_ttic
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* Contributors: Shohei Fujii

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
* Remove needless compiling flags
* Fix CUDA_ERROR_LAUNCH_FAILED error occurred in dpm_ttic
  * Correct memory allocate & copy size
  * Correct address calculation on device side
* Improve cmake CUDA configuration
  Use cmake CUDA feature as possible for removing absolute paths.
* Use c++11 option instead of c++0x
  We can use newer compilers which support 'c++11' option
* Initial commit for public release
* Contributors: Manato Hirabayashi, Shinpei Kato, Syohei YOSHIDA

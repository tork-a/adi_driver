^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package adi_driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.0 (2018-01-21)
------------------
* note that you need to restart after addgroup (`#3 <https://github.com/tork-a/adi_driver/issues/3>`_)
* Add publish_tf argument for launch file
* Change to load model only if use rviz
* Update index.rst
* Fix schematics, add documents
  - Schematics of ADIS16470 cable was wrong
  - Add ADXL345 cable schematics
* Add docbuild (`#2 <https://github.com/tork-a/adi_driver/issues/2>`_)
  * add circle.yml
  * add docbuild command to CMakeLists.txt
  * Update index.rst contents
  * Put travis badge.
* Contributors: Ryosuke Tajima, Tokyo Opensource Robotics Developer 534, Y. Suzuki

0.0.1 (2017-12-14)
------------------
* Add doc/index.rst
* Fix build errors
* Remove all error of roslint
* Add roslint settings
* Adjust header inclusion
* Add loop rate parameter
* Refactor adis16470 code
* Add and change copyrights
* Change copyright representation
* Change test more practical
* add .travis.yml (`#1 <https://github.com/7675t/adi_driver/issues/1>`)
* add xacro to pakcage.xml
* add rviz, imu_filter_madgwick to pakcage.xml
* fix layout in package.xml
* add rqt_plot to package.xml
* add .travis.yml, using generate_prerelease_script.py
* Contributors: Ryosuke Tajima, Tokyo Opensource Robotics Developer 534

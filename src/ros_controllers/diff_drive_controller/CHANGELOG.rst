^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package diff_drive_controller
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.8 (2017-02-17)
------------------
* updated changelog
* Contributors: Hilario Tome

0.2.7 (2016-11-10)
------------------

0.2.6 (2016-11-09)
------------------
* Updated changelogs
* Contributors: Hilario Tome

0.3.0 (2017-10-10)
------------------

0.2.9 (2017-10-10)
------------------
* update test wheel_data to new wheel_data_stamped
* fix warning: comparison between signed and unsigned
* replace msg WheelData -> WheelDataStamped
* add msg WheelDataStamped.msg
* Allow multiple Tiagos on a single Gazebo
* 0.2.8
* Updated changelog
* updated changelog
* 0.2.7
* Updated changelog
* 0.2.6
* Updated changelog
* Updated changelogs
* Contributors: Hilario Tome, Jeremie Deray, davidfernandez

0.2.5 (2016-10-13)
------------------

0.2.4 (2016-03-07)
------------------

0.2.3 (2015-10-08)
------------------
* Address -Wunused-parameter warnings
* Contributors: Adolfo Rodriguez Tsouroukdissian

0.2.2 (2015-07-01)
------------------

0.2.1 (2015-03-19)
------------------
* Fix Preserve Turning Radius
  This will re-check the limit factor for linear/angular if the min is from angular/linear.
  Thanks to @jordiadell
* Fix limit factors when preserving turning radius
* Reset old wheel pos for odometry
* Contributors: Bence Magyar, Enrique Fernandez

0.2.0 (2015-03-12)
------------------
* update test/diffbot_multipliers.yaml
* per wheel radius multiplier
* moved check on dt on top of update
* Add test for wheel data (actual vs. reference)
* Add wheel data with actual + reference
* Publish limited velocity if publish_cmd param
* Add tests for preserve turning radius
* Add jerk limits tests
* Limit jerk
* Add preserve_turning_radius param
* Add tests for preserve turning radius
* Add jerk limits tests
* Limit jerk
* Use expected speed variable on test check
* Check traveled distance in XY plane
* Check twist (not position) in linear velocity test
* Add velocity rolling window size param
* Contributors: Bence Magyar, Enrique Fernandez, Jeremie Deray

0.1.0 (2014-10-29)
------------------
* First release of PAL's hydro backport

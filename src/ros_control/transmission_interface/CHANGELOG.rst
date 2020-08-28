^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package transmission_interface
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.5 (2017-02-17)
------------------
* added pointer getters for torque sensor and absoute position encoders
* Contributors: Hilario Tome

0.2.4 (2016-11-09)
------------------
* Fixed bug hasTorqueSensor and hasAbsoluteSensor, added std run_time exception support to controller manager
* Contributors: Hilario Tome

0.2.3 (2016-10-11)
------------------
* Added has torque sensor and absolute encoder support to transmissions
* Added backcompatibility
* Absolute position and torque sensor working
* Modified structures to have absolute encoder and torque sensor parameters
* Contributors: Hilario Tome

0.2.2 (2016-03-31)
------------------
* Changed private members of transmission parser from private to protected in order to implement transmission parser with blacklist
* Contributors: Hilario Tome

0.2.1 (2016-01-28)
------------------

0.2.0 (2016-01-28)
------------------
* Fix various catkin lint warnings
* Allow loading from TransmissionInfo list
* Contributors: Adolfo Rodriguez Tsouroukdissian, Victor Lopez

0.1.2 (2015-10-08)
------------------
* Address -Wunused-parameter warnings
* Contributors: Adolfo Rodriguez Tsouroukdissian

0.1.1 (2014-10-29)
------------------

0.1.0 (2014-10-28)
------------------
* First release of PAL's hydro backport

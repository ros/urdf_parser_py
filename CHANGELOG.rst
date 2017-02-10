^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package urdfdom_py
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.3 (2017-02-10)
------------------
* Made Chris and Shane the maintainers
* Added python-lxml to the travis build.
* Reverted line break (`ros/urdfdom#77 <https://github.com/ros/urdfdom/pull/77>`_) now that there is a more generic solution. (`#5 <https://github.com/ros/urdf_parser_py/issues/5>`_)
* Added line break to make errors easier to read. (`#4 <https://github.com/ros/urdf_parser_py/issues/4>`_)
* Contributors: Chris Lalancette, Isaac I.Y. Saito

0.3.1 (2016-02-22)
------------------
* Add travis
* Add package.xml for ROS release
* Tweak CMakeLists to reflect migration from urdfdom
* Contributors: Jackie Kay

0.3.0 (2016-02-16)
------------------
* [urdf_parser_py] Add missing newline
* color and texture in link material are optional
* fixed transmission parser to match specification
  http://wiki.ros.org/urdf/XML/Transmission#A.3Ctransmission.3E_Elements
  proper unittest for urdf_parser_py
  - added transmission parsing tests
* Merge pull request `#59 <https://github.com/ros/urdf_parser_py/issues/59>`_ from goretkin/goretkin-optional-calibration
  joint calibration is optional
* made hardware interface optional in both transmission actuator and joint
* joint calibration is optional
  according to http://wiki.ros.org/urdf/XML/joint
* set limit(upper/lower), safety_controller(soft_upper/lower_limit, k_position) is optional
* urdf_parser_py.xml_reflection : python 3 compatibility
  Substitute 'Exception, e' with `Exception as e`. This ensure compatibility with Python 3 and Python 2.6,2.7 .
  Check https://docs.python.org/3/howto/pyporting.html#capturing-the-currently-raised-exception for more info.
* urdf_parser_py: Confirmed that both new transmissions are parseable in the same robot.
  test: Added in specific test for tranmsissions. Have it print the type.
  @todo: Make this a unittest.
* urdf_parser_py: Added attribute preservation for xmlr.RawType (i.e., <gazebo reference="..."/>)
* urdf_parser_py: Parsing new transmission format works. Need to add in test for old PR2 transmission types.
  test/calvin: Added in generated Calvin URDF provided by v4hn
  test/romeo: Reformatted to match Calvin test
* urdf_parser_py.urdf: Adding duck-typed parsing for ros_control and PR2 transmissions.
* [urdf_parser_py] Python3 compatibility
* Contributors: Antonio El Khoury, Doug Sievers, Eric Cousineau, Gustavo Goretkin, Ioan A Sucan, Ioan Sucan, Jackie Kay, Kei Okada, Kelsey Hawkins, Séverin Lemaignan, Tamaki Nishino, Thomas Moulard, VahidAminZ, Vincent Rabaud, eacousineau, isucan

0.2.9 (2013-09-10)
------------------
* add xml_reflection to the install rule
* use install-layout deb
* install urdf_python_py with plain cmake
* xml_reflection: Moved into urdf_parser_py
* urdf_parser_py, xml_reflection: Removed catkin build stuff
* urdf_parser_py, xml_reflection: Remove runtime dependencies on rospy, roslib.
  Will remove catkin from setup.py, along with CMakeLists.txt files
* README.md: Moving into urdf_parser_py
* urdf/Transmission: Uncommented line. Need to figure out how to make transmission types more flexible (for usage with ros_control)
* urdf: Commented out transmissions -- need to address more generalized plugins (see ros_control packages)
* urdf: Made sure that geometry children constraint does not count comments
  Removed .pyc files
  Conflicts:
  .gitignore
* Added in some compatibility methods / accessors for calibration stack
* CMakeLists.txt: Reflected path change {examples => scripts}/display_urdf
  package.xml's: Removed python export path (does not apply)
* test: Reorganized.
  TODO Make actual unit test?
* xml_reflection: Refactoring into catkin package. Trying to figure out how to manage source setup.sh for development...
* urdf: Changed ordering of tags to match that of romeo.urdf
  test: Added in romeo test (fuerte branch)
* Corrected wrong import for xml_reflection.basics module
* Added in test for pelican_urdf
* xml_reflection: Made separate package, moved out of urdf_parser_py
* urdf.Transmission: Needed to add tag
* urdf.Transmission: Small bufix for type
* urdf: Changed Mesh.scale attribute to not be required (thanks to Carlos De La Guardia)
* Refactored variable names to match PEP 8 - http://www.python.org/dev/peps/pep-0008/#introduction
* Finished refactoring. Instance methods: to_xml => write_xml, load_xml => read_xml. Class methods: from\_{xml,string}
* Renamed to_xml to dump_xml, but that is inconsistent with pyyaml. Will rename to {read,write}_xml for clarity.
* Fixed bug in Robot.add_joint() (thanks to Carlos De La Guardia). Fixed some SDF stuff
* Starting to develop SDF stuff
* Touched up some namespace stuff a little, fixed example script.
* Updated API, added back in static from_xml and generalized it. Updated example file.
* Going to separate out xml and python variables
* Starting to do some baic refactoring
* Have Gazebo stuff working a little better. Yaml dumps work as well
* Added in Gazebo tags, but it looks ugly... Need to just extra all children and insert those normally
* Did that refactoring stuff
* Refactoring loading design
* Realizing that maybe I should make a factory-type object for aggregate types
* Changing to accomadate aggregate types
* Added in check_valid() for warnings / assertions
* Things are updated now. Might run slower, but meh
* Can at least run urdf.py
* Things coming together code wise, but have not yet tested. Need to
* Have basic reflection setup, continuing on
* Modifying reflection setup
* There's a problem if an element is supposed to be unique and there's multiple instances of it, it will be overwritten by later occurences. In what seems to be the policy of the urdfdom parser, it uses the first instance, whereas this implementation uses the last instance.
* Tried to add in Gazebo stuff
* Trying a few more complicated things
* Seems to be working now
* More reworking
* Trying out a couple of methods
* Doing some more rework
* Adding transition, but want to set value easily... Going to try etree
* Messing around with the yaml stuff
* Bugfixed color stuff
* Update version to 0.3.0
* Add setup.py, remove comments
* Catkinize package
* Fix inertia matrix constructor.
  * ixy was set to iyy by mistake.
  * Reported by Joseph Salini.
* Do not strip trailing zeros in float display.
  * Stripping trailing zeros causes the removal of exponents if value
  is expressed in scientific notation (e.g. 1.2e-10 -> 1.2e-1).
* Fix float display while writing xml document.
  * The older method caused a loss of float precision when writing urdf
  file.
* Added verbose flag to parsing methods.  If false, the parser will not throw warnings if
  it encounters unknown elements.
* Changed Joint names to strings (was causing a bug), added get_root which detects the link which is top in the tree (URDF's should guarantee a unique root/tree structure), and added an option to get_chain which allows one to not include fixed joints in the chain.
* Load link inertial origin when parsing URDF.
* Do not display XML output in display_urdf.
* Add tree structure pretty printing.
* Allow the user to choose from which parameter the model is loaded.
* Enhance naming following Kelsey Hawkins's suggestions.
  - make load/parse methods static
  - rename load into load_xml_file, parse into parse_xml_string
  - rename loadFromParameterServer into load_from_paremeter_server
* Ignore compiled Python files.
* Support model loading from the parameter server. Make test generic.
* Update manifest.xml to use epydoc.
* Update manifest.xml.
* Replace urdf_python by urdf_parser_py everywhere.
* Remove unwanted packages.

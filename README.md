# urdf_parser_py

## Development Setup

You must manually run `setup.py`. For catkin development, you can install to $ws/../build/lib/pythonX.Y/dist-packages via

	devel_prefix=$(cd $(catkin_find --first-only)/.. && pwd)
	cd ../urdf_parser_py
	python setup.py install --install-layout deb --prefix $devel_prefix

## Authors

*	Thomas Moulard - `urdfpy` implementation, integration
*	David Lu - `urdf_python` implementation, integration
*	Kelsey Hawkins - `urdf_parser_python` implementation, integration
*	Antonio El Khoury - bugfixes
*	Eric Cousineau - reflection update

## Reflection

This an attempt to generalize the structure of the URDF via reflection to make it easier to extend. This concept is taken from Gazebo's SDF structure, and was done with SDF in mind to a) make an SDF parser and b) make a simple converter between URDF and SDF.

### Changes

*	Features:
	*	Transmission and basic Gazebo nodes.
	*	General aggregate types, preserving order
	*	Dumping to YAML, used for printing to string (dictionaries do not preserve attribute ordering)
*	XML Parsing: minidom has been swapped out with lxml.etree, but it should not be hard to change that back. Maybe Sax could be used for event-driven parsing.
*	API:
	*	Loading methods rely primarily on instance methods rather than static methods, mirroring Gazebo's SDF construct-then-load method
	*	Renamed static `parse_xml()` to `from_xml()`, and renamed `load_*` methods to `from_*` if they are static

### Todo

1.	Support additional formats (SDF, drakeURDF, etc.)
	*	Parse Gazebo's SDF definition files at some point? For speed's sake, parse it and have it generate code to use?
	*	Consider auto-generating modules from schemas such as [urdf.xsd](https://github.com/ros/urdfdom/blob/master/xsd/urdf.xsd). This can extend to [SDF](http://sdformat.org/schemas/model.xsd), [drakeURDF](https://github.com/RobotLocomotion/drake/blob/master/drake/doc/drakeURDF.xsd).
2.	Make a direct, two-way URDF <-> SDF converter.
	*	Gazebo has the ability to load URDFs and save SDFs, but it lumps everything together
3.	Consider a cleaner implementation for reflection.
	*	Make the names a little clearer, especially the fact that `from_xml` and `to_xml` write to a node, but do not create a new one.
	*	Abstraction layer is not clear. Should explicitly use abstract classes, and try to really clarify the dispatch order (`xmlr.Element`, `xmlr.Param`, `xmlr.Object`, etc.)
4.	Figure out good policy for handling default methods. If saving to XML, write out default values, or leave them out for brevity (and to leave it open for change)? Might be best to add that as an option.
5.	Find a lightweight package that can handle the reflection aspect more elegantly. Enthought traits? IPython's spinoff of traits?

# urdf_parser_py

## Authors

*   Thomas Moulard - `urdfpy` implementation, integration
*   David Lu - `urdf_python` implementation, integration
*   Kelsey Hawkins - `urdf_parser_python` implementation, integration
*   Antonio El Khoury - bugfixes
*   Eric Cousineau - reflection
*   Ioan Sucan
*   Jackie Kay
*   Chris LaLancette (maintainer)
*   Shane Loretz (maintainer)

## Features

*   URDF
*   SDF (very basic coverage)
*   XML Saving / Loading
    * Some attempts to preserve original ordering; comments are stripped out,
    however.

## Todo

1.  Deprecate public access to `xml_reflection`.
2.  Make a direct, two-way URDF <-> SDF converter when kinematics are not an
    issue.

## Development Setup

For ease of developing, you may manually run `setup.py`.
For catkin development, you can install to
`${ws}/../build/lib/pythonX.Y/dist-packages` via:

    devel_prefix=$(cd $(catkin_find --first-only)/.. && pwd)
    cd ../urdf_parser_py
    python setup.py install --install-layout deb --prefix ${devel_prefix}

#!/bin/bash
set -eux -o pipefail

dir=$(dirname $BASH_SOURCE)
scripts="$dir/../../scripts"

name=romeo
# TODO(eacousineau): This is an older version, directly generated from package.
# Update this to use version-controlled URDF:
# https://github.com/ros-aldebaran/romeo_robot/blob/e53c8f3/romeo_description/urdf/romeo_generated_urdf/romeo.urdf
orig="$dir/romeo.urdf"
gen="/tmp/$name.urdf"
patch="/tmp/$name.patch"

$scripts/display_urdf "$orig" -o "$gen"
diff -u "$orig" "$gen" | tee "$patch"

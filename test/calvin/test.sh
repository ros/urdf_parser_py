#!/bin/bash
set -eux -o pipefail

# TODO(eacousineau): Errors out with 'no hardwareInterface defined'. Update
# URDF and fix.

dir=$(dirname $BASH_SOURCE)
scripts="$dir/../../scripts"

name=calvin
orig="$dir/$name.urdf"
gen="/tmp/$name.urdf"
patch="/tmp/$name.patch"

$scripts/display_urdf "$orig" -o "$gen"
diff -u "$orig" "$gen" > "$patch"

#!/usr/bin/env bash
echo "Running CI build locally."
export ros_release_name=melodic
export ubuntu_version_name=bionic
curl -s "https://raw.githubusercontent.com/shadow-robot/sr-build-tools/master/bin/sr-run-ci-build.sh" | bash -x /dev/stdin master local check_cache,check_build,check_install,code_coverage

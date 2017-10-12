#!/usr/bin/env bash

echo "starting root_finish_bootstrap"

ROSDEP_RULES=/etc/ros/rosdep/sources.list.d
CUSTOM_ROSDEP_RULES=19-custom.list
echo "CONTENTS OF ROSDEP_RULES:"
ls ${ROSDEP_RULES}
cp ${ROSDEP_RULES}/20-default.list ${ROSDEP_RULES}/${CUSTOM_ROSDEP_RULES}
sed -i '/python.yaml/c\yaml file:\/\/\/home\/ubuntu\/src\/rosdistro\/rosdep\/python.yaml' ${ROSDEP_RULES}/${CUSTOM_ROSDEP_RULES}

echo "CONTENTS OF ROSDEP_RULES AFTER NEW FILE:"
ls ${ROSDEP_RULES}

echo "finishing root_finish_bootstrap"

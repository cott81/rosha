#! /bin/bash

echo "set up diagnosis for GenSystem"
#pkgPath=rospack find my_diagnostic_aggregator
pkgPath=/home/dominik/work/impera/cn-care-ros-pkg/my_diagnostic_aggregator
echo $pkgPath
rosparam load $pkgPath/launch/GenSystem_load.yaml /my_diagnostic_aggregator

$pkgPath/bin/my_aggregator_node
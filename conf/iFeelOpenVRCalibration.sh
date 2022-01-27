usage() {
cat << EOF
************************************************************************************************************************************
Script for calibrating OpenVR world and iFeel world with the virtualizer world.
The script should be launched expecting the subject inside the virtualizer in T-Pose.

OPTION: <vertical distance from the subject belly to the subject eyes in meters>

EXAMPLE USAGE: ./iFeelOpenVRCalibration.sh 0.5
************************************************************************************************************************************
EOF
}


################################################################################
# "MAIN" FUNCTION:                                                             #
################################################################################
if [ $# -lt 1 ]; then
    echo "Error in options passed!"
    echo ""
    usage
    exit 1
fi

CHEST_TO_HEAD_DISTANCE_M=$1

echo "OpenVR: resetting seated position"
echo "resetSeatedPosition" | yarp rpc /OpenVRTrackersModule/rpc

echo "Virtualizer: resetting player orientation"
echo "resetPlayerOrientation" | yarp rpc /virtualizer/rpc

echo "Virtualizer: resetting player height"
echo "resetPlayerHeight" | yarp rpc /virtualizer/rpc 

echo "iFeel: removing IMUs orientation offset"
echo "removeIMUsAbsoluteRotationOffset" | yarp rpc  /iFeelSuit/calibrator/rpc:i

echo "Creating static transform: virtualizer_root -> openVR_origin"
echo "set_static_transform_rad virtualizer_root openVR_origin 0.1 0 $CHEST_TO_HEAD_DISTANCE_M 1.5708 0 -1.5708" | yarp rpc /transformServer/rpc
echo "set_static_transform_rad virtualizer_frame root_link_desired 0.0 0 0 0.0 -0.2 0.0" | yarp rpc /transformServer/rpc

exit 0

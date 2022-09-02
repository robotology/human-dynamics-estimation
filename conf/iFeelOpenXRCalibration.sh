usage() {
cat << EOF
************************************************************************************************************************************
Script for calibrating OpenVR world and iFeel world with the virtualizer world.
The script should be launched expecting the subject inside the virtualizer in T-Pose.

OPTION: <vertical distance from the root link to the subject eyes in meters>

EXAMPLE USAGE: ./iFeelOpenXRCalibration.sh 0.5
************************************************************************************************************************************

EOF
}


################################################################################
# "MAIN" FUNCTION:                                                             #
################################################################################
usage
if [ $# -lt 1 ]; then
    HEIGHT_CORRECTION_M=-0.1
    echo "[WARNING] No option passed, using default HEIGHT_CORRECTION_M "$HEIGHT_CORRECTION_M
    echo ""
else
    HEIGHT_CORRECTION_M=$1
fi

# echo "OpenVR: resetting seated position"
# echo "resetSeatedPosition" | yarp rpc /OpenVRTrackersModule/rpc

echo "OpenXR: resetting seated position"
echo "alignRootFrameToHeadset" | yarp rpc /joypadDevice/Oculus/rpc

echo "Creating static transform: vive_tracker_waist_pose -> world"
echo "set_static_transform_rad vive_tracker_waist_pose world 0.0 $HEIGHT_CORRECTION_M 0.1 0.0 -1.5708 -1.5708" | yarp rpc /transformServer/rpc
echo "Creating static transform: world -> root_link_desired"
echo "set_static_transform_rad world root_link_desired 0.0 0 0 0.0 -0.2 0.0" | yarp rpc /transformServer/rpc

exit 0

namespace yarp xsens

struct FrameReferece {
    1: string frameReference;
    2: string frameName;
}

/**
 * Methods definition for the XsensDriver Wrapper service
 */
service XsensDriverService {

    /**
     * Calibrate the Xsens device with the default calibration procedure
     * \note this returns immediately
     */
    oneway void calibrateAsync();

    /**
     * Calibrate the Xsens device with the calibration procedure identified
     * by the specified parameter
     *
     *
     * \note this returns immediately
     * @param calibrationType name of the calibration to execute
     * @return true if the calibration is successful, false otherwise
     */
    oneway void calibrateAsyncWithType(1: string calibrationType);

    /**
     * Calibrate the Xsens device with the default calibration procedure
     * @return true if the calibration is successful, false otherwise
     */
    bool calibrate();

    /**
     * Calibrate the Xsens device with the calibration procedure identified
     * by the specified parameter
     *
     * @param calibrationType name of the calibration to execute
     * @return true if the calibration is successful, false otherwise
     */
    bool calibrateWithType(1: string calibrationType);

    /**
     * Abort the calibration procedure
     */
    oneway void abortCalibration();

    /**
     * Start acquiring data from the Xsens suit
     */
    oneway void startAcquisition();

    /**
     * Stop acquiring data from the suit
     */
    oneway void stopAcquisition();

    /** return the segments defined in the Xsens model
     *
     * \note the order of the segments is the same of the
     * one used to output the data
     * @return the list of segment names
     */
    list<FrameReferece> segments();

    /** returns the body dimensions currently used in the Xsens suit
     *
     * \note these are the dimensions currently used by the Xsens,
     * comprising the one estimated by the device
     * @return all the body dimensions
     */
    map<string, double> bodyDimensions();

    /** set the body dimension specified by the key-value pair
     *
     * @param dimensionKey key specifying the dimension to set
     * @param dimensionValue the corresponding dimension value
     * @return true if the set is successful. False otherwise
     */
    bool setBodyDimension(1:string dimensionKey, 2:double dimensionValue);

    /** set the body dimension specified by the key-value pairs
     *
     * @param dimensions key-value pairs specifying the new dimensions to set
     * @return true if the set is successful. False otherwise
     */
    bool setBodyDimensions(map<string, double> dimensions);

}
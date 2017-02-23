// This is an automatically-generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#ifndef YARP_THRIFT_GENERATOR_XsensDriverService
#define YARP_THRIFT_GENERATOR_XsensDriverService

#include <yarp/os/Wire.h>
#include <yarp/os/idl/WireTypes.h>

namespace xsens {
  class XsensDriverService;
}


/**
 * Methods definition for the XsensDriver Wrapper service
 */
class xsens::XsensDriverService : public yarp::os::Wire {
public:
  XsensDriverService();
  /**
   * Calibrate the Xsens device with the default calibration procedure
   * @return true if the calibration is successful, false otherwise
   */
  virtual bool calibrate();
  /**
   * Calibrate the Xsens device with the calibration procedure identified
   * by the specified parameter
   * @param calibrationType name of the calibration to execute
   * @return true if the calibration is successful, false otherwise
   */
  virtual bool calibrateWithType(const std::string& calibrationType);
  /**
   * Start acquiring data from the Xsens suit
   */
  virtual void startAcquisition();
  /**
   * Stop acquiring data from the suit
   */
  virtual void stopAcquisition();
  /**
   * return the segments defined in the Xsens model
   * \note the order of the segments is the same of the
   * one used to output the data
   * @return the list of segment names
   */
  virtual std::vector<std::string>  segments();
  /**
   * returns the body dimensions currently used in the Xsens suit
   * \note these are the dimensions currently used by the Xsens,
   * comprising the one estimated by the device
   * @return all the body dimensions
   */
  virtual std::map<std::string, double>  bodyDimensions();
  /**
   * set the body dimension specified by the key-value pair
   * @param dimensionKey key specifying the dimension to set
   * @param dimensionValue the corresponding dimension value
   * @return true if the set is successful. False otherwise
   */
  virtual bool setBodyDimension(const std::string& dimensionKey, const double dimensionValue);
  /**
   * set the body dimension specified by the key-value pairs
   * @param dimensions key-value pairs specifying the new dimensions to set
   * @return true if the set is successful. False otherwise
   */
  virtual bool setBodyDimensions(const std::map<std::string, double> & dimensions);
  virtual bool read(yarp::os::ConnectionReader& connection);
  virtual std::vector<std::string> help(const std::string& functionName="--all");
};

#endif

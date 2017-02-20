/*
* Copyright(C) 2016 iCub Facility
* Authors: Francesco Romano
* CopyPolicy : Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
*/

#ifndef YARP_DEV_IHUMAN_H
#define YARP_DEV_IHUMAN_H

#include <map>
#include <string>
#include <vector>

namespace yarp {
    namespace dev {
        class IHumanSkeleton;
    }

    namespace sig {
        class Vector;
    }
}

class yarp::dev::IHumanSkeleton {
public:

    virtual ~IHumanSkeleton();

    virtual unsigned getSegmentCount() const;
    virtual std::string segmentNameAtIndex(unsigned segmentIndex) const;
    virtual std::vector<std::string> segmentNames() const;
    virtual int segmentIndexForName(const std::string& name) const;

    //Configuration
    virtual bool setBodyDimensions(const std::map<std::string, double>& dimensions);
    virtual bool setBodyDimension(const std::string& bodyPart, const double dimension);
    virtual std::map<std::string, double> bodyDimensions() const = 0;

    // Calibration methods
    virtual bool calibrate(const std::string &calibrationType = "") = 0;
    virtual bool abortCalibration() = 0;

    //Acquisition methods
    virtual bool startAcquisition() = 0;
    virtual bool stopAcquisition() = 0;

    // Get Data
    virtual bool getSegmentPoses(std::vector<yarp::sig::Vector>& segmentPoses) = 0;
    virtual bool getSegmentVelocities(std::vector<yarp::sig::Vector>& segmentVelocities);
    virtual bool getSegmentAccelerations(std::vector<yarp::sig::Vector>& segmentAccelerations);
    virtual bool getSegmentInformation(std::vector<yarp::sig::Vector>& segmentPoses,
                                       std::vector<yarp::sig::Vector>& segmentVelocities,
                                       std::vector<yarp::sig::Vector>& segmentAccelerations);

};


#endif /* End of YARP_DEV_IHUMAN_H */

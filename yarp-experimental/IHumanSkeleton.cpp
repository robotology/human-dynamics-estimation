#include "IHumanSkeleton.h"


namespace yarp {
    namespace dev {

        IHumanSkeleton::~IHumanSkeleton() {}

        unsigned IHumanSkeleton::getSegmentCount() const { return 0;  }
        std::string IHumanSkeleton::segmentNameAtIndex(unsigned segmentIndex) const { return ""; }
        std::vector<std::string> IHumanSkeleton::segmentNames() const { return std::vector<std::string >(); }
        int IHumanSkeleton::segmentIndexForName(const std::string& name) const { return -1; }

        //Configuration
        bool IHumanSkeleton::setBodyDimensions(const std::map<std::string, double>&) { return false; }
        bool IHumanSkeleton::setBodyDimension(const std::string& bodyPart, const double) { return false; }

        bool IHumanSkeleton::getSegmentVelocities(std::vector<yarp::sig::Vector>&) { return false; }
        bool IHumanSkeleton::getSegmentAccelerations(std::vector<yarp::sig::Vector>&) { return false; }
        bool IHumanSkeleton::getSegmentInformation(std::vector<yarp::sig::Vector>& segmentPoses,
            std::vector<yarp::sig::Vector>& segmentVelocities,
            std::vector<yarp::sig::Vector>& segmentAccelerations)
        {
            return getSegmentPoses(segmentPoses)
                && getSegmentVelocities(segmentVelocities)
                && getSegmentAccelerations(segmentAccelerations);
        }

    }
}

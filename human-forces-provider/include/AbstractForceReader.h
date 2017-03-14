/*!
 * @file AbstractForceReader.h
 * @author Claudia Latella
 * @date 2017
 * @copyright iCub Facility - Istituto Italiano di Tecnologia
 */


#ifndef HUMAN_ABSTRACTFORCEREADER_H
#define HUMAN_ABSTRACTFORCEREADER_H

#include "ForceReader.h"

#include <yarp/sig/Vector.h>


namespace human
{
    class AbstractForceReader;
    class Force6D;
    class FrameTransformer;
}

namespace yarp
{
    namespace sig
    {
        class Vector;
    }
}


/*! @brief Abstract class for implementing code required from its derived classes. */
class human::AbstractForceReader : public human::ForceReader
{
private:
    FrameTransformer *m_frameTransformer;
    std::string m_attachedLink;
    std::string m_referenceFrame;
    yarp::sig::Vector m_packedForce;
    
protected:
    /*!
     * Read the force external to the module.
     * @param[in] force external force to be read
     * @return true if read successfull, false otherwise
     *
     * @note At this stage the external force can come from a YARP port (the robot case)
     * or from a device (the force plates case).
     */
    virtual bool lowLevelRead(yarp::sig::Vector& force) = 0;
    
public:
    /*!
     * Constructor from a link and frame.
     */
    AbstractForceReader(std::string attachedLink,
                 std::string referenceFrame);
    /*!
     * Set the frame transform.
     */
    virtual void setTransformer(FrameTransformer *frameTransformer);
    /*!
     * Return the frame transform.
     */
    virtual FrameTransformer *getTransformer();

    virtual bool readForce(Force6D &readForce);
};

#endif /* HUMAN_ABSTRACTFORCEREADER_H */

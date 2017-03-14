/*!
 * @file AbstractForceReader.cpp
 * @author Claudia Latella
 * @date 2017
 * @copyright iCub Facility - Istituto Italiano di Tecnologia
 */


#include "AbstractForceReader.h"
#include "FrameTransformer.h"

#include <yarp/os/LogStream.h>

#include <thrifts/Force6D.h>


namespace human
{
    AbstractForceReader::AbstractForceReader(std::string attachedLink,
                                             std::string referenceFrame)
    : m_frameTransformer(0)
    , m_attachedLink(attachedLink)
    , m_referenceFrame(referenceFrame)
    , m_packedForce(6, 0.0) {}
    
    
    void AbstractForceReader::setTransformer(FrameTransformer *frameTransformer)
    {
        m_frameTransformer = frameTransformer;
    }

    
    FrameTransformer* AbstractForceReader::getTransformer()
    {
        return m_frameTransformer;
    }

    
    bool AbstractForceReader::readForce(Force6D &packedForce)
    {
        packedForce.appliedLink = m_attachedLink;
        packedForce.expressedFrame = m_referenceFrame;
        
        if (!this->lowLevelRead(m_packedForce))
        {
            yError("Something wrong in the forces reading!");
            return false;
        }

        if (m_frameTransformer)
        {
            //TODO: Check if the input and output variable can be the same
            if (!m_frameTransformer->transformForceFrame(m_packedForce,
                                                         m_packedForce,
                                                         packedForce.appliedLink))
            {
                yError("Something wrong in the forces transformation!");
                return false;
            }
        }
        
        packedForce.fx = m_packedForce(0);
        packedForce.fy = m_packedForce(1);
        packedForce.fz = m_packedForce(2);
        packedForce.ux = m_packedForce(3);
        packedForce.uy = m_packedForce(4);
        packedForce.uz = m_packedForce(5);
        
        return true;
    }
}

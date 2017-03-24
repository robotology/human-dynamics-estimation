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
    , m_packedForce(6, 0.0)
    , m_forceBuffer(6, 0.0) {}
    
    
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
            packedForce.fx = 0;
            packedForce.fy = 0;
            packedForce.fz = 0;
            packedForce.ux = 0;
            packedForce.uy = 0;
            packedForce.uz = 0;
            return false;
        }

        if (m_frameTransformer)
        {
            m_forceBuffer = m_packedForce;
            if (!m_frameTransformer->transformForceFrame(m_forceBuffer,
                                                         m_packedForce,
                                                         packedForce.expressedFrame))
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

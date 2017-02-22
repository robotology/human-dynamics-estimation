//
//  FrameTransformer.h
//  HumanDynamicsEstimation
//
//  Created by Claudia Latella on 22/02/17.
//
//

/* FRAMETRANSFORMER.H is an interface for transforming forces from a defined 
 * frame with respect to another given one.
 */

#ifndef HUMAN_FRAMETRANSFORMER_H
#define HUMAN_FRAMETRANSFORMER_H

namespace human
{
    class ForceReader;
    class Force6D;
}


class human::ForceReader
{
    
public:
    virtual bool readForce(Force6D &readForce) = 0;
    virtual ~ForceReader();
    
};

#endif /* HUMAN_FRAMETRANSFORMER_H */

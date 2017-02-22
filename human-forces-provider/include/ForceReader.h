//
//  ForceReader.h
//  HumanDynamicsEstimation
//
//  Created by Claudia Latella on 15/02/17.
//
//

/* FORCEREADER.H is an interface for reading forces.
 */

#ifndef HUMAN_FORCEREADER_H
#define HUMAN_FORCEREADER_H

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

#endif /* HUMAN_FORCEREADER_H */

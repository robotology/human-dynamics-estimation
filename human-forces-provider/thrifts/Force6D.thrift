namespace yarp human

struct Force6D {
    /**
     * link in which the 6D Force is applied
     */
    1: string appliedLink;
    
    /**
     * frame on which the 6D force is expressed
     */
    2: string expressedFrame;
    
    3: double fx;
    4: double fy;
    5: double fz;

    6: double ux;
    7: double uy;
    8: double uz;

}


struct HumanForces {
    1 : list<Force6D> forces;
}

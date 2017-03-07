/*!
 * @file ForceReader.h
 * @author Claudia Latella
 * @date 2017
 * @copyright iCub Facility - Istituto Italiano di Tecnologia
 */


#ifndef HUMAN_FORCEREADER_H
#define HUMAN_FORCEREADER_H


namespace human
{
    class ForceReader;
    class Force6D;
}


/*! @brief Interface for reading forces from device and sensors. */
class human::ForceReader
{
public:
    /*!
     * Read the force and return an object ready to be transmitted.
     *
     * @code
     * f = readForce(a)
     * @endcode
     * @param[out] readForce the force read
     * @return true if read successfull, false otherwise
     */
    virtual bool readForce(Force6D &readForce) = 0;
    /*!
     * Destructor.
     */
    virtual ~ForceReader();
};

#endif /* HUMAN_FORCEREADER_H */
namespace yarp human

service HumanStateProviderService {
    
    /*! Return the list of joints and their serialization order
     *
     * the order is consistent with the joint serialization provided
     * by this module streaming port
     *
     * @return the list of joints of the model
     */
    list<string> joints();

}

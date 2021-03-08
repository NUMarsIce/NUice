#pragma once

/**
 *
 * Base driver class to be used by future drivers. 
 * Author: Ian Burwell 3/5/2021
 */

#include <ros.h>
#include <NU32Hardware.h>

class NUDriver {
    
public:
    //basic driver constructor initializes the Node Handle and a namespce
    NUDriver(ros::NodeHandle_<NU32Hardware> nh, String ns) : nh_(nh), namespace_(ns) {};

    /*
     * Setup function to be overwritten by the driver. This will be called once at the beginning of the program 
     */
    virtual void setup() = 0;

    /*
     * Update function. This will be called within the loop function 
     * This function should not block for too long.
     */
    virtual void update() = 0;

protected:
    ros::NodeHandle_<NU32Hardware> nh_;
    String namespace_;
};


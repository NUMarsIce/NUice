#pragma once

/**
 *
 * Base driver class to be used by future drivers. 
 * Author: Ian Burwell 3/5/2021
 */

#include <ros.h>

class NUDriver {
    
public:
    /*
     * Basic driver constructor initializes the Node Handle and a namespce
     */
    NUDriver(ros::NodeHandle& nh, const char* ns) : nh_(nh), namespace_(ns) {}

    /*
     * Setup function to be overwritten by the driver (ensuring to call the parent function). This will be called once at the beginning of the program 
     */
    virtual void setup() = 0;

    /*
     * Update function  to be overridden(ensuring to call the parent function). This will be called within the loop function 
     * This function should not block for too long.
     */
    virtual void update() = 0;

protected:
    ros::NodeHandle& nh_;
    const char* namespace_;

    /*
     * Appends the current namespace to the beginning of the string
     */
    const char* appendNamespace(const char* cstr){
        const char* buff = malloc(strlen(cstr)+strlen(namespace_)+1);
        strcpy(buff, namespace_);
        strcpy(buff+strlen(namespace_), cstr);
        return buff;
    }

};


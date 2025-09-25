#ifndef __INET_IEEE1901PRSTYPES_H
#define __INET_IEEE1901PRSTYPES_H

#include "omnetpp.h"

namespace inet {

struct PrsWindowResult {
    bool detected = false;
    bool otherPresent = false;
    bool selfSent = false;
    omnetpp::simtime_t startTime = omnetpp::SimTime::ZERO;
    omnetpp::simtime_t endTime = omnetpp::SimTime::ZERO;
};

} // namespace inet

#endif


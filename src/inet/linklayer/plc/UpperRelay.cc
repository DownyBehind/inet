#include <omnetpp.h>
#include <string>

namespace inet {
using namespace omnetpp;

class UpperRelay : public cSimpleModule {
  protected:
    virtual void handleMessage(cMessage *msg) override {
        cGate *outGate = gate("out");
        cGate *nextGate = outGate ? outGate->getNextGate() : nullptr;
        cModule *destOwner = nextGate ? nextGate->getOwnerModule() : nullptr;
        std::string destPathStr = destOwner ? destOwner->getFullPath() : std::string("<null>");
        const char *destGateName = nextGate ? nextGate->getName() : "<null>";
        EV_INFO << "[RELAY] forwarding upper frame name=" << msg->getName()
                << " t=" << simTime()
                << " to=" << destPathStr << ":" << destGateName
                << endl;
        send(msg, "out");
    }
};

Define_Module(UpperRelay);
} // namespace inet



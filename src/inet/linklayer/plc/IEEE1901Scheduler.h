#ifndef __INET_IEEE1901SCHEDULER_H
#define __INET_IEEE1901SCHEDULER_H

#include <map>
#include <set>
#include <vector>
#include "omnetpp.h"
#include "inet/linklayer/plc/IEEE1901PrsTypes.h"

namespace inet {

class IEEE1901Mac;

class IEEE1901GlobalScheduler
{
  public:
    static IEEE1901GlobalScheduler& getInstance();

    void registerMac(IEEE1901Mac *mac);
    void unregisterMac(IEEE1901Mac *mac);

    int requestPrsWindow(IEEE1901Mac *mac, int priority, omnetpp::simtime_t prs0Duration, omnetpp::simtime_t prs1Duration);
    void recordPrsTransmission(IEEE1901Mac *mac, int slot);

    PrsWindowResult queryPrsResult(IEEE1901Mac *mac, int slot) const;
    const std::set<IEEE1901Mac*>& getPrsParticipants(int slot) const { return prsParticipants[slot]; }

    void handleSchedulerMessage(omnetpp::cMessage *msg);

    bool isPrsActive() const { return prsActive; }

  private:
    IEEE1901GlobalScheduler() = default;

    enum PrsEventType {
        PRS0_END = 1,
        PRS1_END = 2
    };

    struct MacEntry {
        IEEE1901Mac *mac = nullptr;
    };

    struct PrsRequest {
        int priority = 0;
        bool participating = false;
    };

    omnetpp::cMessage *prs0EndMsg = nullptr;
    omnetpp::cMessage *prs1EndMsg = nullptr;
    IEEE1901Mac *coordinator = nullptr;

    bool prsActive = false;
    int prsGeneration = 0;
    omnetpp::simtime_t storedPrs0Duration = 0;
    omnetpp::simtime_t storedPrs1Duration = 0;

    std::vector<MacEntry> macs;
    std::map<IEEE1901Mac*, PrsRequest> activeRequests;
    std::set<IEEE1901Mac*> prsParticipants[2];

    void ensureCoordinator();
    void startPrsCycle(omnetpp::simtime_t startTime, omnetpp::simtime_t prs0Duration, omnetpp::simtime_t prs1Duration);
    void concludePrsSlot(int slot);
    void concludePrsCycle();
    void notifyPrsPhaseStart(int slot, omnetpp::simtime_t startTime, omnetpp::simtime_t endTime);
    void notifyPrsPhaseEnd(int slot);
};

} // namespace inet

#endif


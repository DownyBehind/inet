#include "inet/linklayer/plc/IEEE1901Scheduler.h"
#include "inet/linklayer/plc/IEEE1901Mac.h"

namespace inet {

IEEE1901GlobalScheduler& IEEE1901GlobalScheduler::getInstance()
{
    static IEEE1901GlobalScheduler instance;
    return instance;
}

void IEEE1901GlobalScheduler::registerMac(IEEE1901Mac *mac)
{
    macs.push_back(MacEntry{mac});
}

void IEEE1901GlobalScheduler::unregisterMac(IEEE1901Mac *mac)
{
    macs.erase(std::remove_if(macs.begin(), macs.end(), [&](const MacEntry &entry){ return entry.mac == mac; }), macs.end());
    activeRequests.erase(mac);
    prsParticipants[0].erase(mac);
    prsParticipants[1].erase(mac);
}

int IEEE1901GlobalScheduler::requestPrsWindow(IEEE1901Mac *mac, int priority, omnetpp::simtime_t prs0Duration, omnetpp::simtime_t prs1Duration)
{
    ensureCoordinator();
    activeRequests[mac] = PrsRequest{priority, true};

    if (!prsActive) {
        storedPrs0Duration = prs0Duration;
        storedPrs1Duration = prs1Duration;
        startPrsCycle(omnetpp::simTime(), storedPrs0Duration, storedPrs1Duration);
    }
    return prsGeneration;
}

void IEEE1901GlobalScheduler::recordPrsTransmission(IEEE1901Mac *mac, int slot)
{
    if (slot >= 0 && slot < 2)
        prsParticipants[slot].insert(mac);
}

IEEE1901GlobalScheduler::PrsWindowResult IEEE1901GlobalScheduler::queryPrsResult(IEEE1901Mac *mac, int slot) const
{
    PrsWindowResult result;
    if (slot < 0 || slot > 1)
        return result;
    result.selfSent = prsParticipants[slot].count(mac) > 0;
    result.detected = !prsParticipants[slot].empty();
    if (result.detected) {
        result.otherPresent = prsParticipants[slot].size() > (result.selfSent ? 1 : 0);
    }
    return result;
}

void IEEE1901GlobalScheduler::handleSchedulerMessage(omnetpp::cMessage *msg)
{
    if (msg == prs0EndMsg) {
        notifyPrsPhaseEnd(0);
        concludePrsSlot(0);
        notifyPrsPhaseStart(1, omnetpp::simTime(), omnetpp::simTime() + storedPrs1Duration);
        omnetpp::scheduleAt(omnetpp::simTime() + storedPrs1Duration, prs1EndMsg);
    }
    else if (msg == prs1EndMsg) {
        notifyPrsPhaseEnd(1);
        concludePrsSlot(1);
        concludePrsCycle();
    }
}

void IEEE1901GlobalScheduler::ensureCoordinator()
{
    if (!coordinator) {
        if (macs.empty())
            throw omnetpp::cRuntimeError("IEEE1901GlobalScheduler has no MAC instances registered");
        coordinator = macs.front().mac;
        if (!prs0EndMsg) {
            prs0EndMsg = new omnetpp::cMessage("prs0End");
            prs0EndMsg->setContextPointer(this);
        }
        if (!prs1EndMsg) {
            prs1EndMsg = new omnetpp::cMessage("prs1End");
            prs1EndMsg->setContextPointer(this);
        }
    }
}

void IEEE1901GlobalScheduler::startPrsCycle(omnetpp::simtime_t startTime, omnetpp::simtime_t prs0Duration, omnetpp::simtime_t prs1Duration)
{
    prsActive = true;
    prsGeneration += 1;
    prsParticipants[0].clear();
    prsParticipants[1].clear();
    notifyPrsPhaseStart(0, startTime, startTime + prs0Duration);
    if (!prs0EndMsg)
        prs0EndMsg = new omnetpp::cMessage("prs0End");
    if (!prs1EndMsg)
        prs1EndMsg = new omnetpp::cMessage("prs1End");
    omnetpp::scheduleAt(startTime + prs0Duration, prs0EndMsg);
    storedPrs1Duration = prs1Duration;
}

void IEEE1901GlobalScheduler::concludePrsSlot(int slot)
{
    // nothing yet; placeholder for analytics/logging
}

void IEEE1901GlobalScheduler::concludePrsCycle()
{
    prsActive = false;
    activeRequests.clear();
}

void IEEE1901GlobalScheduler::notifyPrsPhaseStart(int slot, omnetpp::simtime_t startTime, omnetpp::simtime_t endTime)
{
    for (auto &entry : macs) {
        if (entry.mac)
            entry.mac->onPrsPhaseStart(slot, prsGeneration, startTime, endTime);
    }
}

void IEEE1901GlobalScheduler::notifyPrsPhaseEnd(int slot)
{
    for (auto &entry : macs) {
        if (entry.mac)
            entry.mac->onPrsPhaseEnd(slot, prsGeneration);
    }
}

} // namespace inet


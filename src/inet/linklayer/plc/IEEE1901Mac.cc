//
// Copyright (C) 2025 INET Framework
//
// SPDX-License-Identifier: LGPL-3.0-or-later
//

#include "inet/linklayer/plc/IEEE1901Mac.h"
#include "inet/linklayer/plc/IEEE1901Scheduler.h"
#include <algorithm>
// Shared PRS bus state definitions
std::vector<IEEE1901Mac*> IEEE1901Mac::s_macInstances;
bool IEEE1901Mac::s_channelBusy = false;
omnetpp::simtime_t IEEE1901Mac::s_busyUntil = SIMTIME_ZERO;
int IEEE1901Mac::s_activeTxCount = 0;
#include "inet/common/ModuleAccess.h"
#include "inet/common/ProtocolTag_m.h"
#include "inet/common/lifecycle/ModuleOperations.h"

namespace inet {

Define_Module(IEEE1901Mac);

// Signal definitions
simsignal_t IEEE1901Mac::framesSentSignal = cComponent::registerSignal("framesSent");
simsignal_t IEEE1901Mac::framesReceivedSignal = cComponent::registerSignal("framesReceived");
simsignal_t IEEE1901Mac::framesDroppedSignal = cComponent::registerSignal("framesDropped");
simsignal_t IEEE1901Mac::collisionsSignal = cComponent::registerSignal("collisions");
simsignal_t IEEE1901Mac::backoffsSignal = cComponent::registerSignal("backoffs");

IEEE1901Mac::~IEEE1901Mac()
{
    EV_DEBUG << "IEEE1901Mac destructor called" << endl;
    
    cancelAndDelete(backoffTimer);
    cancelAndDelete(txTimer);
    if (sifsTimer) { if (sifsTimer->isScheduled()) cancelEvent(sifsTimer); delete sifsTimer; sifsTimer=nullptr; }
    if (difsTimer) { if (difsTimer->isScheduled()) cancelEvent(difsTimer); delete difsTimer; difsTimer=nullptr; }
    cancelAndDelete(prs0Timer);
    cancelAndDelete(prs1Timer);
    cancelAndDelete(retryTimer);
    delete currentFrame;
    // Dispose any pending upper-layer frames left in queue
    while (!txQueue.empty()) { delete txQueue.front(); txQueue.pop_front(); }
    auto it = std::find(s_macInstances.begin(), s_macInstances.end(), this);
    if (it != s_macInstances.end())
        s_macInstances.erase(it);
    IEEE1901GlobalScheduler::getInstance().unregisterMac(this);
}

void IEEE1901Mac::initialize(int stage)
{
    EV_DEBUG << "IEEE1901Mac::initialize() stage " << stage << endl;
    // IMPORTANT: Initialize base class to bind standard gates and lifecycle
    MacProtocolBase::initialize(stage);
    
    if (stage == INITSTAGE_LOCAL) {
        // Read parameters from NED file
        txPower = par("txPower").doubleValue();
        bitrate = par("bitrate").doubleValue();
        maxRetries = par("maxRetries").intValue();
        slotTime = par("slotTime").doubleValue();
        sifsTime = par("sifsTime").doubleValue();
        difsTime = par("difsTime").doubleValue();
        cwMin = par("cwMin").intValue();
        cwMax = par("cwMax").intValue();
        
        // Priority Resolution Signal parameters
        prs0Duration = par("prs0Duration").doubleValue();
        prs1Duration = par("prs1Duration").doubleValue();
        
        EV_INFO << "IEEE1901Mac initialized with parameters:" << endl;
        EV_INFO << "  txPower: " << txPower << " dBm" << endl;
        EV_INFO << "  bitrate: " << bitrate << " bps" << endl;
        EV_INFO << "  maxRetries: " << maxRetries << endl;
        EV_INFO << "  slotTime: " << slotTime << " s" << endl;
        EV_INFO << "  sifsTime: " << sifsTime << " s" << endl;
        EV_INFO << "  difsTime: " << difsTime << " s" << endl;
        EV_INFO << "  cwMin: " << cwMin << endl;
        EV_INFO << "  cwMax: " << cwMax << endl;
        EV_INFO << "  prs0Duration: " << prs0Duration << " s" << endl;
        EV_INFO << "  prs1Duration: " << prs1Duration << " s" << endl;
        
        // Initialize state variables
        isTransmitting = false;
        isReceiving = false;
        channelBusy = false;
        backoffCounter = 0;
        currentCW = cwMin;
        retryCounter = 0;
        currentFrame = nullptr;
        
        // Initialize HomePlug 1.0 Backoff Algorithm variables
        backoffProcedureCounter = 0;
        deferralCounter = 0;
        backoffCounterBC = 0;
        
        // Initialize Priority Resolution state
        inPriorityResolution = false;
        prs0SignalDetected = false;
        prs1SignalDetected = false;
        wonPriorityResolution = false;
        currentFramePriority = 0;
        prsSlotGeneration[0] = prsSlotGeneration[1] = 0;
        prsWindowEnd[0] = prsWindowEnd[1] = SIMTIME_ZERO;
        s_prsBus.resetSlot(0);
        s_prsBus.resetSlot(1);
        
        // Initialize statistics
        numFramesSent = 0;
        numFramesReceived = 0;
        numFramesDropped = 0;
        numCollisions = 0;
        numBackoffs = 0;
        numTxAttempts = 0;
        numBpcIncrements = 0;
        numDcZeroBusyIncrements = 0;
        numTxFailCollisions = 0;
        
        // Create timer messages
        backoffTimer = new cMessage("backoffTimer"); take(backoffTimer);
        txTimer = new cMessage("txTimer"); take(txTimer);
        sifsTimer = new cMessage("sifsTimer"); take(sifsTimer);
        difsTimer = new cMessage("difsTimer"); take(difsTimer);
        prs0Timer = new cMessage("prs0Timer"); take(prs0Timer);
        prs1Timer = new cMessage("prs1Timer"); take(prs1Timer);
        retryTimer = new cMessage("retryTimer"); take(retryTimer);
        s_macInstances.push_back(this);
        IEEE1901GlobalScheduler::getInstance().registerMac(this);
        
        EV_DEBUG << "IEEE1901Mac initialization stage " << stage << " completed" << endl;
    }
    else if (stage == INITSTAGE_LINK_LAYER) {
        EV_DEBUG << "IEEE1901Mac link layer initialization" << endl;
        // cache lower layer out gate if available
        lowerOutGate = hasGate("lowerLayerOut") ? gate("lowerLayerOut") : nullptr;
        
        // Register signals for statistics collection
        WATCH(numFramesSent);
        WATCH(numFramesReceived);
        WATCH(numFramesDropped);
        WATCH(numCollisions);
        WATCH(numBackoffs);
        WATCH(isTransmitting);
        WATCH(isReceiving);
        WATCH(channelBusy);
        WATCH(backoffCounter);
        WATCH(backoffProcedureCounter);
        WATCH(deferralCounter);
        WATCH(backoffCounterBC);
        WATCH(inPriorityResolution);
        WATCH(prs0SignalDetected);
        WATCH(prs1SignalDetected);
        WATCH(wonPriorityResolution);
        WATCH(currentFramePriority);
        WATCH(numBpcIncrements);
        WATCH(numDcZeroBusyIncrements);
        WATCH(numTxFailCollisions);
        
        updateDisplayString();
    }
}

void IEEE1901Mac::configureNetworkInterface()
{
    EV_DEBUG << "IEEE1901Mac::configureNetworkInterface()" << endl;
    
    // Configure the network interface for PLC communication
    // Set interface properties specific to IEEE 1901
    if (networkInterface) {
        networkInterface->setMtu(MTU);
        networkInterface->setMulticast(true);
        networkInterface->setBroadcast(true);
    }
}

void IEEE1901Mac::handleMessage(cMessage *msg)
{
    EV_DEBUG << "IEEE1901Mac::handleMessage() - received message: " << msg->getName() << endl;
    
    if (msg->isSelfMessage()) {
        if (msg == retryTimer) {
            onRetryTimer();
            return;
        }
        handleSelfMessage(msg);
    }
    else if (msg->getArrivalGate()->isName("upperLayerIn")) {
        EV_DEBUG << "Handling frame from upper layer" << endl;
        handleUpperLayerFrame(msg);
    }
    else if (msg->getArrivalGate()->isName("lowerLayerIn")) {
        EV_DEBUG << "Handling frame from lower layer (PHY)" << endl;
        handleLowerLayerFrame(msg);
    }
    else {
        EV_ERROR << "Unknown arrival gate: " << msg->getArrivalGate()->getName() << endl;
        delete msg;
    }
}

void IEEE1901Mac::startTransmission(PLCFrame *frame)
{
    EV_DEBUG << "IEEE1901Mac::startTransmission() - frame type: " << frame->getFrameType() << endl;
    
    if (isTransmitting) {
        EV_WARN << "Already transmitting, dropping frame" << endl;
        delete frame;
        numFramesDropped++;
        emit(framesDroppedSignal, numFramesDropped);
        return;
    }
    
    EV_INFO << "Starting transmission of frame with:" << endl;
    EV_INFO << "  Frame Type: " << frame->getFrameType() << endl;
    EV_INFO << "  Priority: " << frame->getPriority() << endl;
    EV_INFO << "  Source: " << frame->getSrcAddr() << endl;
    EV_INFO << "  Destination: " << frame->getDestAddr() << endl;
    EV_INFO << "  Payload Length: " << frame->getPayloadLength() << endl;
    
    // Store frame and its priority
    currentFrame = frame;
    currentFramePriority = frame->getPriority();
    
    // Start priority resolution process first
    startPriorityResolution(currentFramePriority);
    updateDisplayString();
}

int IEEE1901Mac::getMacNumericId() const
{
    int macId = getId();
    return macId;
}

void IEEE1901Mac::beginPrsWindow(int)
{
    prsWindowResult[0] = PrsWindowResult();
    prsWindowResult[1] = PrsWindowResult();
    sentPrs0 = false;
    sentPrs1 = false;
}

void IEEE1901Mac::notePrsTransmission(int prsSlot)
{
    IEEE1901GlobalScheduler::getInstance().recordPrsTransmission(this, prsSlot);
    if (prsSlot == 0)
        sentPrs0 = true;
    else
        sentPrs1 = true;
}

IEEE1901Mac::PrsWindowResult IEEE1901Mac::concludePrsWindow(int prsSlot)
{
    auto result = IEEE1901GlobalScheduler::getInstance().queryPrsResult(this, prsSlot);
    prsWindowResult[prsSlot] = result;
    return result;
}

bool IEEE1901Mac::shouldDeferPriority(int framePriority) const
{
    const PrsWindowResult &prs0 = prsWindowResult[0];
    const PrsWindowResult &prs1 = prsWindowResult[1];
    switch (framePriority) {
        case 0:
            return prs0.otherPresent || prs1.otherPresent;
        case 1:
            return prs1.otherPresent || prs0.otherPresent;
        case 2:
            return prs0.otherPresent && prs1.otherPresent;
        case 3:
        default:
            return false;
    }
}

void IEEE1901Mac::handlePriorityLoss()
{
    EV_INFO << "Lost priority resolution, deferring transmission" << endl;
    EV_ALWAYS << "[PRS_DEFER] node=" << getFullPath() << " t=" << simTime() << endl;
    if (currentFrame) {
        txQueue.push_front(currentFrame);
        currentFrame = nullptr;
    }
    if (backoffTimer->isScheduled()) cancelEvent(backoffTimer);
    if (txTimer->isScheduled()) cancelEvent(txTimer);
    resetBackoffProcedure();
    scheduleRetry(slotTime);
}

void IEEE1901Mac::handlePriorityWin()
{
    EV_INFO << "Won priority resolution, proceeding to HomePlug 1.0 backoff stage" << endl;
    scheduleBackoff();
}

void IEEE1901Mac::scheduleRetry(simtime_t delay)
{
    if (!retryTimer)
        retryTimer = new cMessage("retryTimer");
    if (!retryTimer->isScheduled())
        scheduleAt(simTime() + delay, retryTimer);
}

void IEEE1901Mac::onRetryTimer()
{
    if (!txQueue.empty() && !currentFrame) {
        PLCFrame *frame = txQueue.front();
        txQueue.pop_front();
        startTransmission(frame);
    }
}

void IEEE1901Mac::processSlotResult(bool busy)
{
    if (busy)
        updateCountersOnBusySlot();
    else
        updateCountersOnIdleSlot();
}

void IEEE1901Mac::handlePhyControl(cMessage *msg)
{
    const char *name = msg->getName();
    if (!strcmp(name, "CTRL_PHY_BUSY_ON")) {
        processPhyBusyNotification();
    }
    else if (!strcmp(name, "CTRL_PHY_BUSY_OFF")) {
        processPhyIdleNotification();
    }
    else if (!strcmp(name, "CTRL_TX_FAIL")) {
        handleTxFailure();
    }
    else {
        EV_WARN << "Unknown PHY control message: " << name << endl;
    }
    delete msg;
}

void IEEE1901Mac::processPhyBusyNotification()
{
    s_activeTxCount = std::max(0, s_activeTxCount + 1);
    s_channelBusy = (s_activeTxCount > 0);
}

void IEEE1901Mac::processPhyIdleNotification()
{
    s_activeTxCount = std::max(0, s_activeTxCount - 1);
    s_channelBusy = (s_activeTxCount > 0);
}

void IEEE1901Mac::handleTxFailure()
{
    EV_WARN << "Received CTRL_TX_FAIL from PHY" << endl;
    if (retryBackup) {
        txQueue.push_front(retryBackup);
        retryBackup = nullptr;
    }
    else if (currentFrame) {
        txQueue.push_front(currentFrame->dup());
    }
    backoffProcedureCounter++;
    numBpcIncrements++;
    numTxFailCollisions++;
    handlePriorityLoss();
}

void IEEE1901Mac::scheduleBackoff()
{
    EV_DEBUG << "IEEE1901Mac::scheduleBackoff() - HomePlug 1.0 algorithm" << endl;
    
    if (backoffTimer->isScheduled()) {
        EV_DEBUG << "Backoff timer already scheduled" << endl;
        return;
    }
    
    // Initialize backoff counters according to HomePlug 1.0
    // Per Table I: start at BPC=0 parameters, then immediately advance BPC bookkeeping
    initializeBackoffCounters(currentFramePriority, backoffProcedureCounter);
    // After BC and DC are updated, BPC immediately increases by one (paper spec)
    backoffProcedureCounter++;
    numBpcIncrements++;
    
    EV_INFO << "HomePlug 1.0 Backoff initialized:" << endl;
    EV_INFO << "  Priority: CA" << currentFramePriority << endl;
    EV_INFO << "  BPC: " << backoffProcedureCounter << endl;
    EV_INFO << "  DC: " << deferralCounter << endl;
    EV_INFO << "  CW: " << currentCW << endl;
    EV_INFO << "  BC: " << backoffCounterBC << endl;
    
    numBackoffs++;
    emit(backoffsSignal, numBackoffs);
    
    // Schedule first slot check
    scheduleAt(simTime() + slotTime, backoffTimer);
    
    updateDisplayString();
}

void IEEE1901Mac::updateCountersOnBusySlot()
{
    EV_DEBUG << "IEEE1901Mac::updateCountersOnBusySlot() - HomePlug 1.0 algorithm" << endl;
    EV_INFO << "[BUSY_SLOT] node=" << getFullPath()
            << " slotIndex=" << backoffCounterBC
            << " DC=" << deferralCounter
            << " t=" << simTime() << endl;
    
    EV_INFO << "Channel detected as busy during backoff slot" << endl;
    EV_INFO << "  Before update: BC=" << backoffCounterBC << ", DC=" << deferralCounter << endl;
    
    // HomePlug 1.0: When slot is busy, both BC and DC decrement
    if (backoffCounterBC > 0) {
        backoffCounterBC--;
    }
    
    if (deferralCounter > 0) {
        deferralCounter--;
    }
    
    EV_INFO << "  After update: BC=" << backoffCounterBC << ", DC=" << deferralCounter << endl;
    
    // Check if DC reached 0 while medium is busy
    if (deferralCounter == 0) {
        EV_INFO << "Deferral Counter reached 0 with busy medium - increasing BPC" << endl;
        
        // Increase BPC and reset counters
        backoffProcedureCounter++;
        numBpcIncrements++;
        numDcZeroBusyIncrements++;
        initializeBackoffCounters(currentFramePriority, backoffProcedureCounter);
        EV_ALWAYS << "[BUSY_SLOT] escalate node=" << getFullPath()
                << " newBPC=" << backoffProcedureCounter
                << " newDC=" << deferralCounter
                << " newCW=" << currentCW
                << " t=" << simTime() << endl;
        
        EV_INFO << "  Updated BPC: " << backoffProcedureCounter << endl;
        EV_INFO << "  New DC: " << deferralCounter << endl;
        EV_INFO << "  New CW: " << currentCW << endl;
        EV_INFO << "  New BC: " << backoffCounterBC << endl;
    }
    
    // Continue backoff if BC > 0
    if (backoffCounterBC > 0) {
        // Freeze backoff counter when channel is busy, continue monitoring
        scheduleAt(simTime() + slotTime, backoffTimer);
        EV_DEBUG << "Scheduled next slot check (busy channel)" << endl;
    } else {
        EV_INFO << "BC reached 0 - ready to transmit after busy period ends" << endl;
        EV_INFO << "OBS MAC_BC0 node=" << getFullPath() << " t=" << simTime() << endl;
    }
    
    updateDisplayString();
}

void IEEE1901Mac::updateCountersOnIdleSlot()
{
    EV_DEBUG << "IEEE1901Mac::updateCountersOnIdleSlot() - HomePlug 1.0 algorithm" << endl;
    EV_ALWAYS << "[IDLE_SLOT] node=" << getFullPath()
            << " slotIndex=" << backoffCounterBC
            << " DC=" << deferralCounter
            << " t=" << simTime() << endl;
    
    EV_INFO << "Channel detected as idle during backoff slot" << endl;
    EV_INFO << "  Before update: BC=" << backoffCounterBC << ", DC=" << deferralCounter << endl;
    
    // HomePlug 1.0: When slot is idle, only BC decrements
    if (backoffCounterBC > 0) {
        backoffCounterBC--;
        EV_INFO << "  Decremented BC to: " << backoffCounterBC << endl;
        
        if (backoffCounterBC > 0) {
            // Schedule next slot check
            scheduleAt(simTime() + slotTime, backoffTimer);
            EV_DEBUG << "Scheduled next backoff slot check" << endl;
        } else {
            EV_INFO << "BC reached 0 - ready to transmit!" << endl;
            handleSlotTimeout();
        }
    } else {
        EV_INFO << "BC already 0 - ready to transmit!" << endl;
        EV_ALWAYS << "[IDLE_SLOT] immediate_tx node=" << getFullPath() << " t=" << simTime() << endl;
        EV_INFO << "OBS MAC_BC0 node=" << getFullPath() << " t=" << simTime() << endl;
        handleSlotTimeout();
    }
    
    updateDisplayString();
}

void IEEE1901Mac::finish()
{
    EV_DEBUG << "IEEE1901Mac::finish()" << endl;
    EV_INFO << "IEEE1901Mac::finish() called at t=" << simTime() << " module=" << getFullPath() << endl;
    
    recordScalar("frames sent", numFramesSent);
    recordScalar("frames received", numFramesReceived);
    recordScalar("frames dropped", numFramesDropped);
    recordScalar("collisions", numCollisions);
    recordScalar("backoffs", numBackoffs);
    recordScalar("tx attempts", numTxAttempts);
    recordScalar("bpc_increments", numBpcIncrements);
    recordScalar("dc_zero_busy_increments", numDcZeroBusyIncrements);
    recordScalar("tx_fail_collisions", numTxFailCollisions);
    
    EV_INFO << "IEEE1901Mac statistics:" << endl;
    EV_INFO << "  Frames sent: " << numFramesSent << endl;
    EV_INFO << "  Frames received: " << numFramesReceived << endl;
    EV_INFO << "  Frames dropped: " << numFramesDropped << endl;
    EV_INFO << "  Collisions: " << numCollisions << endl;
    EV_INFO << "  Backoffs: " << numBackoffs << endl;
    
    // Calculate derived statistics
    if (numFramesSent + numFramesDropped > 0) {
        double successRate = (double)numFramesSent / (numFramesSent + numFramesDropped) * 100.0;
        recordScalar("success rate (%)", successRate);
        EV_INFO << "  Success rate: " << successRate << "%" << endl;
    }
    
    if (numBackoffs > 0) {
        double avgBackoffs = (double)numBackoffs / numFramesSent;
        recordScalar("average backoffs per frame", avgBackoffs);
        EV_INFO << "  Average backoffs per frame: " << avgBackoffs << endl;
    }
    // Safety cleanup: dispose any pending frames that won't be transmitted after simulation ends
    EV_INFO << "  Cleanup pending: txQueue size=" << txQueue.size() << ", has currentFrame=" << (currentFrame!=nullptr) << endl;
    if (currentFrame) {
        delete currentFrame;
        currentFrame = nullptr;
    }
    while (!txQueue.empty()) { delete txQueue.front(); txQueue.pop_front(); }
}

// Helper method implementations (minimum viable path)
void IEEE1901Mac::handleUpperLayerFrame(cMessage *msg)
{
    EV_DEBUG << "IEEE1901Mac::handleUpperLayerFrame()" << endl;
    PLCFrame *frame = dynamic_cast<PLCFrame *>(msg);
    if (!frame) {
        // Wrap unknown upper message into a PLCFrame for MAC processing
        EV_WARN << "Wrapping non-PLCFrame (" << msg->getClassName() << ") into PLCFrame" << endl;
        auto *f = new PLCFrame("PLC");
        f->setFrameType(2);
        f->setPriority(0);
        f->setSrcAddr(0);
        f->setDestAddr(0);
        f->setPayloadLength(300);
        f->setAckRequired(false);
        delete msg;
        frame = f;
    }

    // If busy, enqueue instead of dropping (observer-level buffering, no 사양 변경)
    bool macBusy = isTransmitting || currentFrame != nullptr || inPriorityResolution || backoffTimer->isScheduled();
    if (macBusy) {
        EV_INFO << "MAC busy; enqueue upper frame name=" << frame->getName() << " prio=CA" << frame->getPriority() << endl;
        txQueue.push_back(frame);
        return;
    }

    // Begin transmission flow: PRS → Backoff → TX
    startTransmission(frame);
}

void IEEE1901Mac::handleLowerLayerFrame(cMessage *msg)
{
    EV_DEBUG << "IEEE1901Mac::handleLowerLayerFrame()" << endl;
    if (!dynamic_cast<PLCFrame *>(msg)) {
        handlePhyControl(msg);
        return;
    }
    PLCFrame *frame = static_cast<PLCFrame *>(msg);
    EV_INFO << "Delivering frame up to upper layer: mod=" << getFullPath()
            << " name=" << frame->getName()
            << " src=" << frame->getSrcAddr() << " dest=" << frame->getDestAddr()
            << " prio=CA" << frame->getPriority() << endl;
    sendUp(frame);
}

void IEEE1901Mac::handleSelfMessage(cMessage *msg)
{
    EV_DEBUG << "IEEE1901Mac::handleSelfMessage() - " << msg->getName() << endl;
    
    if (msg == backoffTimer) {
        handleBackoffTimer();
    }
    else if (msg == txTimer) {
        handleTransmissionTimer();
    }
    else if (msg == sifsTimer) {
        handleSifsTimer();
    }
    else if (msg == difsTimer) {
        handleDifsTimer();
    }
    else if (msg == prs0Timer) {
        handlePrs0Timer();
    }
    else if (msg == prs1Timer) {
        handlePrs1Timer();
    }
    else {
        EV_ERROR << "Unknown self message: " << msg->getName() << endl;
    }
}

void IEEE1901Mac::handleBackoffTimer()
{
    EV_DEBUG << "IEEE1901Mac::handleBackoffTimer() - checking channel state" << endl;
    bool busy = !isChannelIdle();
    processSlotResult(busy);
}

void IEEE1901Mac::handleTransmissionTimer()
{
    EV_DEBUG << "IEEE1901Mac::handleTransmissionTimer() - TX complete" << endl;
    isTransmitting = false;
    numFramesSent++;
    emit(framesSentSignal, numFramesSent);
    // Ensure no dangling pointer remains after TX completes
    if (currentFrame) {
        delete currentFrame;
        currentFrame = nullptr;
    }
    // TX succeeded, dispose retry backup if any
    if (retryBackup) { delete retryBackup; retryBackup = nullptr; }
    // Schedule interframe spaces according to model (use SIFS/DIFS params as RIFS/CIFS)
    if (!sifsTimer->isScheduled()) {
        scheduleAt(simTime() + sifsTime, sifsTimer);
        EV_DEBUG << "Scheduled SIFS (RIFS) timer at t=" << (simTime() + sifsTime) << endl;
    }
    updateDisplayString();

    // If queued frames exist, start next immediately after CIFS elapses
}

void IEEE1901Mac::handleSifsTimer()
{
    EV_DEBUG << "IEEE1901Mac::handleSifsTimer() - stub implementation" << endl;
    // After SIFS(RIFS), schedule DIFS(CIFS)
    if (!difsTimer->isScheduled()) {
        scheduleAt(simTime() + difsTime, difsTimer);
        EV_DEBUG << "Scheduled DIFS (CIFS) timer at t=" << (simTime() + difsTime) << endl;
    }
}

void IEEE1901Mac::handleDifsTimer()
{
    EV_DEBUG << "IEEE1901Mac::handleDifsTimer() - stub implementation" << endl;
    // CIFS elapsed; medium returns to contention. Dequeue next pending frame if any.
    if (!txQueue.empty()) {
        PLCFrame *next = txQueue.front();
        txQueue.pop_front();
        EV_INFO << "Dequeue next upper frame name=" << next->getName() << " prio=CA" << next->getPriority() << endl;
        startTransmission(next);
    }
}

void IEEE1901Mac::setChannelBusy(bool busy)
{
    EV_DEBUG << "IEEE1901Mac::setChannelBusy(" << busy << ")" << endl;

    if (busy) {
        if (!channelBusy) {
            s_activeTxCount = std::max(0, s_activeTxCount + 1);
        }
        channelBusy = true;
    } else {
        if (channelBusy) {
            s_activeTxCount = std::max(0, s_activeTxCount - 1);
        }
        channelBusy = false;
    }
    s_channelBusy = (s_activeTxCount > 0);
    EV_ALWAYS << "[PHY_BUSY] node=" << getFullPath() << " busy=" << s_channelBusy << " refcnt=" << s_activeTxCount << " t=" << simTime() << endl;
    updateDisplayString();
}

bool IEEE1901Mac::isChannelIdle()
{
    bool idle = !s_channelBusy;
    EV_DEBUG << "IEEE1901Mac::isChannelIdle() - returning " << idle << endl;
    return idle;
}

void IEEE1901Mac::updateDisplayString()
{
    if (hasGUI()) {
        char buf[80];
        sprintf(buf, "S:%ld R:%ld D:%ld %s", 
                numFramesSent, numFramesReceived, numFramesDropped,
                isTransmitting ? "TX" : (isReceiving ? "RX" : (channelBusy ? "BUSY" : "IDLE")));
        getDisplayString().setTagArg("t", 0, buf);
    }
}

void IEEE1901Mac::recordStatistics()
{
    EV_DEBUG << "IEEE1901Mac::recordStatistics() - stub implementation" << endl;
    // TODO: Implement additional statistics recording
}

// HomePlug 1.0 Backoff Algorithm Implementation
void IEEE1901Mac::initializeBackoffCounters(int priority, int bpc)
{
    EV_DEBUG << "IEEE1901Mac::initializeBackoffCounters() - priority: CA" << priority << ", BPC: " << bpc << endl;
    
    int dc, cw;
    getDeferralCounterAndCW(priority, bpc, dc, cw);
    
    deferralCounter = dc;
    currentCW = cw;
    
    // Generate random backoff counter BC from [0, CW-1]
    backoffCounterBC = intuniform(0, currentCW - 1);
    
    EV_INFO << "Initialized HomePlug 1.0 backoff counters:" << endl;
    EV_INFO << "  Priority: CA" << priority << endl;
    EV_INFO << "  BPC: " << bpc << endl;
    EV_INFO << "  DC: " << deferralCounter << endl;
    EV_INFO << "  CW: " << currentCW << endl;
    EV_INFO << "  BC: " << backoffCounterBC << " (random from [0," << (currentCW-1) << "])" << endl;
}

void IEEE1901Mac::getDeferralCounterAndCW(int priority, int bpc, int &dc, int &cw)
{
    EV_DEBUG << "IEEE1901Mac::getDeferralCounterAndCW() - priority: CA" << priority << ", BPC: " << bpc << endl;
    
    // Implementation based on Table I from Jung et al. (2005)
    // "Performance evaluation of IEEE 802.11 DCF and IEEE 802.11e EDCA in non-saturated conditions"
    
    if (priority >= 2) { // CA3/CA2 - Higher priorities
        switch (bpc) {
            case 0:
                dc = 0;
                cw = 7;
                break;
            case 1:
                dc = 1;
                cw = 15;
                break;
            case 2:
                dc = 3;
                cw = 15;
                break;
            default: // BPC > 2
                dc = 15;
                cw = 31;
                break;
        }
        EV_INFO << "CA3/CA2 parameters - BPC=" << bpc << " → DC=" << dc << ", CW=" << cw << endl;
    } else { // CA1/CA0 - Lower priorities
        switch (bpc) {
            case 0:
                dc = 0;
                cw = 7;
                break;
            case 1:
                dc = 1;
                cw = 15;
                break;
            case 2:
                dc = 3;
                cw = 31;
                break;
            default: // BPC > 2
                dc = 15;
                cw = 63;
                break;
        }
        EV_INFO << "CA1/CA0 parameters - BPC=" << bpc << " → DC=" << dc << ", CW=" << cw << endl;
    }
}

void IEEE1901Mac::handleSlotTimeout()
{
    EV_DEBUG << "IEEE1901Mac::handleSlotTimeout() - BC reached 0, starting transmission" << endl;
    
    if (!currentFrame) {
        EV_ERROR << "No frame to transmit when BC reached 0" << endl;
        return;
    }
    
    EV_INFO << "HomePlug 1.0 backoff completed - transmitting frame" << endl;
    EV_INFO << "  Final BPC: " << backoffProcedureCounter << endl;
    EV_INFO << "  Final DC: " << deferralCounter << endl;
    EV_INFO << "  Final BC: " << backoffCounterBC << endl;
    
    // Start actual transmission
    isTransmitting = true;
    numTxAttempts++;
    
    // Calculate transmission duration
    simtime_t txDuration = (20 + currentFrame->getPayloadLength()) * 8.0 / bitrate;  // 20 bytes header + payload
    scheduleAt(simTime() + txDuration, txTimer);
    
    EV_INFO << "Frame transmission started - duration: " << txDuration << " s" << endl;
    
    // Send frame to lower layer (PHY)
    cGate *g = hasGate("lowerLayerOut") ? gate("lowerLayerOut") : nullptr;
    if (g && g->isConnected()) {
        // Prepare retry backup in case PHY reports TX fail (collision)
        if (retryBackup) { delete retryBackup; retryBackup = nullptr; }
        retryBackup = currentFrame->dup();
        sendDown(currentFrame);
        // ownership transferred to lower layer; clear pointer immediately
        currentFrame = nullptr;
    } else {
        EV_WARN << "lowerLayerOut not connected, dropping frame" << endl;
        delete currentFrame;
        currentFrame = nullptr;
        cancelEvent(txTimer);
        isTransmitting = false;
        updateDisplayString();
        return;
    }
    
    // Reset backoff procedure for successful transmission attempt
    resetBackoffProcedure();
    
    updateDisplayString();
}

void IEEE1901Mac::resetBackoffProcedure()
{
    EV_DEBUG << "IEEE1901Mac::resetBackoffProcedure()" << endl;
    
    EV_INFO << "Resetting backoff procedure after successful transmission" << endl;
    EV_INFO << "  Previous BPC: " << backoffProcedureCounter << endl;
    
    // Reset BPC for next frame transmission
    backoffProcedureCounter = 0;
    deferralCounter = 0;
    backoffCounterBC = 0;
    
    EV_INFO << "  Reset BPC: " << backoffProcedureCounter << endl;
}

// Priority Resolution Signal (PRS) Implementation based on HomePlug 1.0
void IEEE1901Mac::startPriorityResolution(int framePriority)
{
    EV_DEBUG << "IEEE1901Mac::startPriorityResolution() - priority: " << framePriority << endl;

    if (inPriorityResolution) {
        EV_WARN << "Already in priority resolution phase" << endl;
        return;
    }

    inPriorityResolution = true;
    currentFramePriority = framePriority;
    prs0SignalDetected = false;
    prs1SignalDetected = false;
    wonPriorityResolution = false;
    sentPrs0 = false;
    sentPrs1 = false;
    otherPrs0Present = false;
    otherPrs1Present = false;

    EV_INFO << "Starting Priority Resolution for CA" << framePriority << " frame" << endl;

    bool intent0 = shouldSendPrs0Signal(framePriority);
    bool intent1 = shouldSendPrs1Signal(framePriority);
    EV_INFO << "OBS PRS_INTENT node=" << getFullPath()
            << " ca=" << framePriority
            << " send0=" << (intent0 ? 1 : 0)
            << " send1=" << (intent1 ? 1 : 0)
            << " t=" << simTime() << endl;

    activePrsGeneration = IEEE1901GlobalScheduler::getInstance().requestPrsWindow(this, framePriority, prs0Duration, prs1Duration);
    if (intent0)
        notePrsTransmission(0);
    if (intent1)
        notePrsTransmission(1);

    updateDisplayString();
}

bool IEEE1901Mac::shouldSendPrs0Signal(int priority)
{
    // HomePlug 1.0 Priority Resolution Rules for PRS0:
    // CA0: send no signal in PRS0/PRS1
    // CA1: send signal only in PRS1
    // CA2: send signal only in PRS0  <-- This is the only one that sends in PRS0
    // CA3: send signal in both PRS0 and PRS1
    
    bool shouldSend = (priority == 2) || (priority == 3);
    EV_DEBUG << "shouldSendPrs0Signal(CA" << priority << ") = " << shouldSend << endl;
    return shouldSend;
}

bool IEEE1901Mac::shouldSendPrs1Signal(int priority)
{
    // HomePlug 1.0 Priority Resolution Rules for PRS1:
    // CA0: send no signal in PRS0/PRS1
    // CA1: send signal only in PRS1  <-- This one sends in PRS1
    // CA2: send signal only in PRS0
    // CA3: send signal in both PRS0 and PRS1  <-- This one also sends in PRS1
    
    bool shouldSend = (priority == 1) || (priority == 3);
    EV_DEBUG << "shouldSendPrs1Signal(CA" << priority << ") = " << shouldSend << endl;
    return shouldSend;
}

void IEEE1901Mac::sendPrsSignal(int prsSlot)
{
    EV_INFO << "PRS signal sent in slot " << prsSlot << endl;
    int senderId = getMacNumericId();
    s_prsBus.registerTransmission(prsSlot, senderId);
    EV_DEBUG << "[PRS] Broadcast from MAC " << senderId << " slot " << prsSlot << endl;
    for (auto *mac : s_macInstances) {
        if (!mac)
            continue;
        EV_DEBUG << "[PRS] deliver to MAC " << mac->getMacNumericId() << " slot " << prsSlot << endl;
        mac->receivePrsSignal(prsSlot, senderId);
    }
    EV_ALWAYS << "[PRS_SIGNAL] node=" << getFullPath()
              << " slot=PRS" << prsSlot
              << " gen=" << s_prsBus.generation[prsSlot]
              << " t=" << simTime() << endl;
}

void IEEE1901Mac::detectPrsSignal(int prsSlot)
{
    EV_INFO << "PRS signal detected in slot " << prsSlot << endl;
    int macId = getMacNumericId();
    prsWindowResult[prsSlot].detected = true;
    prsWindowResult[prsSlot].otherPresent = s_prsBus.hasOther(prsSlot, macId);
    if (prsSlot == 0)
        prs0SignalDetected = true;
    else
        prs1SignalDetected = true;
}

void IEEE1901Mac::receivePrsSignal(int prsSlot, int senderId)
{
    int macId = getMacNumericId();
    EV_DEBUG << "[PRS] MAC " << macId << " received signal from " << senderId << " slot " << prsSlot << endl;
    if (senderId == macId) {
        // already marked during sendPrsSignal
    }
    else {
        detectPrsSignal(prsSlot);
    }
}

bool IEEE1901Mac::evaluatePriorityResolution(int framePriority)
{
    EV_DEBUG << "IEEE1901Mac::evaluatePriorityResolution() - framePriority: " << framePriority << endl;

    const auto &globalScheduler = IEEE1901GlobalScheduler::getInstance();
    auto prs0 = globalScheduler.queryPrsResult(this, 0);
    auto prs1 = globalScheduler.queryPrsResult(this, 1);
    bool prs0Detected = prs0.detected;
    bool prs1Detected = prs1.detected;
    bool prs0Other = prs0.otherPresent;
    bool prs1Other = prs1.otherPresent;
    bool selfSentPrs0 = sentPrs0;
    bool selfSentPrs1 = sentPrs1;

    EV_ALWAYS << "Priority Resolution evaluation - "
            << "PRS0 detected: " << prs0Detected
            << ", other present: " << prs0Other
            << ", self sent: " << selfSentPrs0 << endl;
    EV_ALWAYS << "Priority Resolution evaluation - "
            << "PRS1 detected: " << prs1Detected
            << ", other present: " << prs1Other
            << ", self sent: " << selfSentPrs1 << endl;
    bool higherPriorityDetected = false;
    bool samePriorityConflict = false;
    switch (framePriority) {
        case 3:
            samePriorityConflict = prs0Other || prs1Other;
            break;
        case 2:
            higherPriorityDetected = (prs0Detected && !selfSentPrs0) || prs1Detected;
            samePriorityConflict = prs1Other;
            break;
        case 1:
            higherPriorityDetected = prs0Detected || (prs1Detected && !selfSentPrs1 && prs1Other);
            samePriorityConflict = prs1Other;
            break;
        case 0:
        default:
            higherPriorityDetected = prs0Detected || prs1Detected;
            samePriorityConflict = prs0Other || prs1Other;
            break;
    }

    EV_ALWAYS << "Priority Resolution evaluation outcome: "
            << " higherPriorityDetected=" << higherPriorityDetected
            << " samePriorityConflict=" << samePriorityConflict << endl;

    if (higherPriorityDetected)
        return false;
    if (samePriorityConflict)
        return shouldDeferPriority(framePriority);
    return true;
}

void IEEE1901Mac::onPrsPhaseStart(int slot, int generation, omnetpp::simtime_t start, omnetpp::simtime_t end)
{
    if (!inPriorityResolution || generation != activePrsGeneration)
        return;
    if (slot == 0) {
        EV_INFO << "PRS0 phase start node=" << getFullPath() << " [" << start << "," << end << ")" << endl;
    }
    else if (slot == 1) {
        EV_INFO << "PRS1 phase start node=" << getFullPath() << " [" << start << "," << end << ")" << endl;
    }
}

void IEEE1901Mac::onPrsPhaseEnd(int slot, int generation)
{
    if (!inPriorityResolution || generation != activePrsGeneration)
        return;

    auto result = concludePrsWindow(slot);
    if (slot == 0) {
        prs0SignalDetected = result.detected;
        otherPrs0Present = result.otherPresent;
        EV_INFO << "PRS0 phase completed node=" << getFullPath()
                << " detected=" << prs0SignalDetected
                << " otherPresent=" << otherPrs0Present << endl;
    }
    else if (slot == 1) {
        prs1SignalDetected = result.detected;
        otherPrs1Present = result.otherPresent;
        EV_INFO << "PRS1 phase completed node=" << getFullPath()
                << " detected=" << prs1SignalDetected
                << " otherPresent=" << otherPrs1Present << endl;
        wonPriorityResolution = evaluatePriorityResolution(currentFramePriority);
        inPriorityResolution = false;
        EV_INFO << "Priority Resolution completed - result: " << (wonPriorityResolution ? "WON" : "LOST") << endl;
        if (wonPriorityResolution)
            handlePriorityWin();
        else
            handlePriorityLoss();
        updateDisplayString();
    }
}

} // namespace inet

//
// Copyright (C) 2025 INET Framework
//
// SPDX-License-Identifier: LGPL-3.0-or-later
//

#include "inet/linklayer/plc/IEEE1901Mac.h"
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
    cancelAndDelete(sifsTimer);
    cancelAndDelete(difsTimer);
    cancelAndDelete(prs0Timer);
    cancelAndDelete(prs1Timer);
    
    delete currentFrame;
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
        
        // Initialize statistics
        numFramesSent = 0;
        numFramesReceived = 0;
        numFramesDropped = 0;
        numCollisions = 0;
        numBackoffs = 0;
        numTxAttempts = 0;
        
        // Create timer messages
        backoffTimer = new cMessage("backoffTimer");
        txTimer = new cMessage("txTimer");
        sifsTimer = new cMessage("sifsTimer");
        difsTimer = new cMessage("difsTimer");
        prs0Timer = new cMessage("prs0Timer");
        prs1Timer = new cMessage("prs1Timer");
        
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
        EV_DEBUG << "Handling self message: " << msg->getName() << endl;
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

void IEEE1901Mac::scheduleBackoff()
{
    EV_DEBUG << "IEEE1901Mac::scheduleBackoff() - HomePlug 1.0 algorithm" << endl;
    
    if (backoffTimer->isScheduled()) {
        EV_DEBUG << "Backoff timer already scheduled" << endl;
        return;
    }
    
    // Initialize backoff counters according to HomePlug 1.0
    initializeBackoffCounters(currentFramePriority, backoffProcedureCounter);
    
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
        initializeBackoffCounters(currentFramePriority, backoffProcedureCounter);
        
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
    }
    
    updateDisplayString();
}

void IEEE1901Mac::updateCountersOnIdleSlot()
{
    EV_DEBUG << "IEEE1901Mac::updateCountersOnIdleSlot() - HomePlug 1.0 algorithm" << endl;
    
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
        handleSlotTimeout();
    }
    
    updateDisplayString();
}

void IEEE1901Mac::finish()
{
    EV_DEBUG << "IEEE1901Mac::finish()" << endl;
    
    recordScalar("frames sent", numFramesSent);
    recordScalar("frames received", numFramesReceived);
    recordScalar("frames dropped", numFramesDropped);
    recordScalar("collisions", numCollisions);
    recordScalar("backoffs", numBackoffs);
    recordScalar("tx attempts", numTxAttempts);
    
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

    // Drop newly arrived frame if we are already busy with another frame
    bool macBusy = isTransmitting || currentFrame != nullptr || inPriorityResolution || backoffTimer->isScheduled();
    if (macBusy) {
        EV_WARN << "MAC busy (isTransmitting=" << isTransmitting
                << ", hasCurrentFrame=" << (currentFrame != nullptr)
                << ", inPRS=" << inPriorityResolution
                << ", backoffScheduled=" << backoffTimer->isScheduled()
                << ") - dropping incoming frame" << endl;
        delete frame;
        numFramesDropped++;
        emit(framesDroppedSignal, numFramesDropped);
        return;
    }

    // Begin transmission flow: PRS → Backoff → TX
    startTransmission(frame);
}

void IEEE1901Mac::handleLowerLayerFrame(cMessage *msg)
{
    EV_DEBUG << "IEEE1901Mac::handleLowerLayerFrame()" << endl;
    PLCFrame *frame = dynamic_cast<PLCFrame *>(msg);
    if (!frame) {
        EV_WARN << "Dropping non-PLCFrame from lower layer: " << msg->getClassName() << endl;
        delete msg;
        return;
    }
    // Forward up to upper layers
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
    
    // Check channel state and update counters accordingly
    if (isChannelIdle()) {
        updateCountersOnIdleSlot();
    } else {
        updateCountersOnBusySlot();
    }
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
    // Schedule interframe spaces according to model (use SIFS/DIFS params as RIFS/CIFS)
    if (!sifsTimer->isScheduled()) {
        scheduleAt(simTime() + sifsTime, sifsTimer);
        EV_DEBUG << "Scheduled SIFS (RIFS) timer at t=" << (simTime() + sifsTime) << endl;
    }
    updateDisplayString();
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
    // CIFS elapsed; medium returns to contention. Next PRS0 will be triggered by new upper frames.
}

void IEEE1901Mac::setChannelBusy(bool busy)
{
    EV_DEBUG << "IEEE1901Mac::setChannelBusy(" << busy << ")" << endl;
    
    if (channelBusy != busy) {
        channelBusy = busy;
        EV_INFO << "Channel state changed to: " << (busy ? "BUSY" : "IDLE") << endl;
        
        if (busy) {
            updateCountersOnBusySlot();
        } else {
            updateCountersOnIdleSlot();
        }
        
        updateDisplayString();
    }
}

bool IEEE1901Mac::isChannelIdle()
{
    EV_DEBUG << "IEEE1901Mac::isChannelIdle() - returning " << (!channelBusy) << endl;
    return !channelBusy;
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
    
    // Initialize PRS state
    inPriorityResolution = true;
    prs0SignalDetected = false;
    prs1SignalDetected = false;
    wonPriorityResolution = false;
    
    EV_INFO << "Starting Priority Resolution for CA" << framePriority << " frame" << endl;
    EV_INFO << "PRS0 duration: " << prs0Duration << " s" << endl;
    EV_INFO << "PRS1 duration: " << prs1Duration << " s" << endl;
    
    // Start PRS0 phase
    if (shouldSendPrs0Signal(framePriority)) {
        EV_INFO << "Sending signal in PRS0 slot (CA" << framePriority << ")" << endl;
        sendPrsSignal(0);
    } else {
        EV_INFO << "Not sending signal in PRS0 slot (CA" << framePriority << ")" << endl;
    }
    
    // Schedule PRS0 completion
    scheduleAt(simTime() + prs0Duration, prs0Timer);
    
    updateDisplayString();
}

void IEEE1901Mac::handlePrs0Timer()
{
    EV_DEBUG << "IEEE1901Mac::handlePrs0Timer() - PRS0 phase completed" << endl;
    
    if (!inPriorityResolution) {
        EV_ERROR << "PRS0 timer fired but not in priority resolution" << endl;
        return;
    }
    
    // Detect if any signals were present in PRS0
    detectPrsSignal(0);
    
    EV_INFO << "PRS0 phase completed - signal detected: " << (prs0SignalDetected ? "YES" : "NO") << endl;
    
    // Start PRS1 phase
    if (shouldSendPrs1Signal(currentFramePriority)) {
        EV_INFO << "Sending signal in PRS1 slot (CA" << currentFramePriority << ")" << endl;
        sendPrsSignal(1);
    } else {
        EV_INFO << "Not sending signal in PRS1 slot (CA" << currentFramePriority << ")" << endl;
    }
    
    // Schedule PRS1 completion
    scheduleAt(simTime() + prs1Duration, prs1Timer);
}

void IEEE1901Mac::handlePrs1Timer()
{
    EV_DEBUG << "IEEE1901Mac::handlePrs1Timer() - PRS1 phase completed" << endl;
    
    if (!inPriorityResolution) {
        EV_ERROR << "PRS1 timer fired but not in priority resolution" << endl;
        return;
    }
    
    // Detect if any signals were present in PRS1
    detectPrsSignal(1);
    
    EV_INFO << "PRS1 phase completed - signal detected: " << (prs1SignalDetected ? "YES" : "NO") << endl;
    
    // Evaluate priority resolution result
    wonPriorityResolution = evaluatePriorityResolution(currentFramePriority);
    inPriorityResolution = false;
    
    EV_INFO << "Priority Resolution completed - result: " << (wonPriorityResolution ? "WON" : "LOST") << endl;
    
    if (wonPriorityResolution) {
        EV_INFO << "Won priority resolution, proceeding to HomePlug 1.0 backoff stage" << endl;
        scheduleBackoff();
    } else {
        EV_INFO << "Lost priority resolution, deferring transmission" << endl;
        // Reset state and wait for next opportunity (ensure ownership cleanup)
        if (currentFrame) {
            delete currentFrame;
            currentFrame = nullptr;
            numFramesDropped++;
            emit(framesDroppedSignal, numFramesDropped);
        }
        // cancel any scheduled timers related to this attempt
        if (backoffTimer->isScheduled()) cancelEvent(backoffTimer);
        if (txTimer->isScheduled()) cancelEvent(txTimer);
        resetBackoffProcedure();
    }
    
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
    EV_DEBUG << "IEEE1901Mac::sendPrsSignal() - slot: PRS" << prsSlot << endl;
    
    // TODO: Send actual PRS signal to physical layer
    // For simulation purposes, we assume the signal is sent successfully
    
    EV_INFO << "Transmitting PRS signal in slot PRS" << prsSlot << endl;
    EV_INFO << "  Frame priority: CA" << currentFramePriority << endl;
    EV_INFO << "  Signal duration: " << (prsSlot == 0 ? prs0Duration : prs1Duration) << " s" << endl;
}

void IEEE1901Mac::detectPrsSignal(int prsSlot)
{
    EV_DEBUG << "IEEE1901Mac::detectPrsSignal() - slot: PRS" << prsSlot << endl;
    
    // TODO: Implement actual signal detection from physical layer
    // For simulation purposes, we simulate signal detection based on network conditions
    
    // Simple simulation: assume signal detected if other nodes are competing
    // In real implementation, this would come from PHY layer carrier sensing
    bool signalDetected = uniform(0, 1) < 0.3; // 30% chance of detecting competing signal
    
    if (prsSlot == 0) {
        prs0SignalDetected = signalDetected;
        EV_INFO << "PRS0 signal detection: " << (signalDetected ? "DETECTED" : "NOT DETECTED") << endl;
    } else {
        prs1SignalDetected = signalDetected;
        EV_INFO << "PRS1 signal detection: " << (signalDetected ? "DETECTED" : "NOT DETECTED") << endl;
    }
}

bool IEEE1901Mac::evaluatePriorityResolution(int framePriority)
{
    EV_DEBUG << "IEEE1901Mac::evaluatePriorityResolution() - priority: CA" << framePriority << endl;
    
    // HomePlug 1.0 Priority Resolution Evaluation:
    // A station wins if no higher priority signals are detected
    
    bool higherPriorityDetected = false;
    
    switch (framePriority) {
        case 0: // CA0 - lowest priority
            // CA0 loses if any signal is detected (CA1, CA2, or CA3 present)
            higherPriorityDetected = prs0SignalDetected || prs1SignalDetected;
            EV_INFO << "CA0 evaluation: higher priority detected = " << higherPriorityDetected << endl;
            break;
            
        case 1: // CA1
            // CA1 loses if CA2 (PRS0 only) or CA3 (both PRS0 and PRS1) is present
            // CA2 presence: PRS0 signal but no PRS1 signal
            // CA3 presence: both PRS0 and PRS1 signals
            higherPriorityDetected = prs0SignalDetected;
            EV_INFO << "CA1 evaluation: CA2/CA3 detected = " << higherPriorityDetected << endl;
            break;
            
        case 2: // CA2
            // CA2 loses if CA3 (both PRS0 and PRS1) is present
            higherPriorityDetected = prs0SignalDetected && prs1SignalDetected;
            EV_INFO << "CA2 evaluation: CA3 detected = " << higherPriorityDetected << endl;
            break;
            
        case 3: // CA3 - highest priority
            // CA3 never loses priority resolution
            higherPriorityDetected = false;
            EV_INFO << "CA3 evaluation: always wins priority resolution" << endl;
            break;
            
        default:
            EV_ERROR << "Invalid priority level: " << framePriority << endl;
            higherPriorityDetected = true; // Fail safe
            break;
    }
    
    bool won = !higherPriorityDetected;
    EV_INFO << "Priority resolution result for CA" << framePriority << ": " << (won ? "WON" : "LOST") << endl;
    
    return won;
}

} // namespace inet

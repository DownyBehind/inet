//
// Copyright (C) 2025 INET Framework
//
// SPDX-License-Identifier: LGPL-3.0-or-later
//

#ifndef __INET_IEEE1901MAC_H
#define __INET_IEEE1901MAC_H

#include "inet/common/INETDefs.h"
#include "inet/common/lifecycle/ModuleOperations.h"
#include "inet/common/lifecycle/OperationalBase.h"
#include "inet/linklayer/plc/PLCFrame_m.h"

namespace inet {

/**
 * IEEE 1901 MAC implementation for Power Line Communication.
 * 
 * This module implements the Medium Access Control (MAC) layer protocol
 * as defined in the IEEE 1901 standard for broadband over power line networks.
 * 
 * The module provides channel access coordination, frame transmission/reception,
 * and collision avoidance mechanisms suitable for PLC networks.
 */
class INET_API IEEE1901Mac : public cSimpleModule
{
  protected:
    // Module parameters
    double txPower;
    double bitrate;
    int maxRetries;
    simtime_t slotTime;
    simtime_t sifsTime;
    simtime_t difsTime;
    int cwMin;
    int cwMax;
    
    // Priority Resolution Signal parameters
    simtime_t prs0Duration;
    simtime_t prs1Duration;
    
    // State variables
    bool isTransmitting;
    bool isReceiving;
    bool channelBusy;
    int backoffCounter;
    int currentCW;
    int retryCounter;
    
    // HomePlug 1.0 Backoff Algorithm variables
    int backoffProcedureCounter;  // BPC
    int deferralCounter;          // DC
    int backoffCounterBC;         // BC (renamed to avoid confusion with existing backoffCounter)
    
    // Priority Resolution State
    bool inPriorityResolution;
    bool prs0SignalDetected;
    bool prs1SignalDetected;
    bool wonPriorityResolution;
    int currentFramePriority;
    
    // Statistics
    long numFramesSent;
    long numFramesReceived;
    long numFramesDropped;
    long numCollisions;
    long numBackoffs;
    
    // Timers
    cMessage *backoffTimer;
    cMessage *txTimer;
    cMessage *sifsTimer;
    cMessage *difsTimer;
    cMessage *prs0Timer;
    cMessage *prs1Timer;
    
    // Current frame being processed
    PLCFrame *currentFrame;
    
    // Signals for statistics
    static simsignal_t framesSentSignal;
    static simsignal_t framesReceivedSignal;
    static simsignal_t framesDroppedSignal;
    static simsignal_t collisionsSignal;
    static simsignal_t backoffsSignal;

  protected:
    // Lifecycle methods
    virtual void initialize(int stage) override;
    virtual int numInitStages() const override { return NUM_INIT_STAGES; }
    virtual void handleMessage(cMessage *msg) override;
    virtual void finish() override;
    
    // Frame transmission methods
    virtual void startTransmission(PLCFrame *frame);
    
    // Backoff and channel access methods
    virtual void scheduleBackoff();
    virtual void updateCountersOnBusySlot();
    virtual void updateCountersOnIdleSlot();
    
    // HomePlug 1.0 Backoff Algorithm methods
    virtual void initializeBackoffCounters(int priority, int bpc);
    virtual void getDeferralCounterAndCW(int priority, int bpc, int &dc, int &cw);
    virtual void handleSlotTimeout();
    virtual void resetBackoffProcedure();
    
    // Priority Resolution methods
    virtual void startPriorityResolution(int framePriority);
    virtual void handlePrs0Timer();
    virtual void handlePrs1Timer();
    virtual bool shouldSendPrs0Signal(int priority);
    virtual bool shouldSendPrs1Signal(int priority);
    virtual void sendPrsSignal(int prsSlot);
    virtual void detectPrsSignal(int prsSlot);
    virtual bool evaluatePriorityResolution(int framePriority);
    
    // Helper methods
    virtual void handleUpperLayerFrame(cMessage *msg);
    virtual void handleLowerLayerFrame(cMessage *msg);
    virtual void handleSelfMessage(cMessage *msg);
    virtual void handleBackoffTimer();
    virtual void handleTransmissionTimer();
    virtual void handleSifsTimer();
    virtual void handleDifsTimer();
    
    // Channel state management
    virtual void setChannelBusy(bool busy);
    virtual bool isChannelIdle();
    
    // Statistics and logging
    virtual void updateDisplayString();
    virtual void recordStatistics();

  public:
    IEEE1901Mac() {}
    virtual ~IEEE1901Mac();
};

} // namespace inet

#endif

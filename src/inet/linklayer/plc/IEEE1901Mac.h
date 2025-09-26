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
#include "inet/linklayer/base/MacProtocolBase.h"
#include "inet/linklayer/plc/PLCFrame_m.h"
#include "inet/linklayer/plc/IEEE1901PrsTypes.h"
#include <deque>
#include <set>
#include <vector>
#include "inet/linklayer/plc/IEEE1901Scheduler.h"

namespace inet {

// IEEE 1901 PLC specific constants
static const int MTU = 1518; // Maximum Transmission Unit for PLC frames

/**
 * IEEE 1901 MAC implementation for Power Line Communication.
 * 
 * This module implements the Medium Access Control (MAC) layer protocol
 * as defined in the IEEE 1901 standard for broadband over power line networks.
 * 
 * The module provides channel access coordination, frame transmission/reception,
 * and collision avoidance mechanisms suitable for PLC networks.
 */
class INET_API IEEE1901Mac : public MacProtocolBase
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
    bool sentPrs0;
    bool sentPrs1;
    bool otherPrs0Present;
    bool otherPrs1Present;
    cMessage *prs0Timer = nullptr;
    cMessage *prs1Timer = nullptr;
    
    // Statistics
    long numFramesSent;
    long numFramesReceived;
    long numFramesDropped;
    long numTxAttempts;
    long numCollisions;
    long numBackoffs;
    // Additional observability counters (no behavior change)
    long numBpcIncrements;          // counts all places where BPC++ occurs
    long numDcZeroBusyIncrements;   // subset: BPC++ due to DC==0 & busy
    long numTxFailCollisions;       // future use: explicit collision model
    
    // Timers
    cMessage *backoffTimer;
    cMessage *txTimer;
    // Reuse single-shot IFG timers to avoid rapid alloc/free and ownership confusion
    cMessage *sifsTimer;
    cMessage *difsTimer;
 
    // Current frame being processed
    PLCFrame *currentFrame;
    // Backup for MAC-layer retransmission in case of PHY TX failure
    PLCFrame *retryBackup = nullptr;
    bool currentTxCollided = false;
    // Upper layer transmit queue (buffers frames when MAC is busy)
    std::deque<PLCFrame*> txQueue;
    // Cached gate pointer for lower layer out
    cGate *lowerOutGate = nullptr;
    
    // Signals for statistics
    static simsignal_t framesSentSignal;
    // Shared PRS bus state across all MAC instances (single-threaded sim)
    // Shared channel busy state
    static bool s_channelBusy;
    static omnetpp::simtime_t s_busyUntil;
    static int s_activeTxCount;
    static std::vector<IEEE1901Mac*> s_macInstances;

    // Test-only: force BC=0 once to provoke collisions in a controlled test
    bool testForceBc0Once = false;

    friend class IEEE1901GlobalScheduler;

    PrsWindowResult prsWindowResult[2];
    bool pendingPrsSend[2] = {false, false};
    int activePrsGeneration = -1;

    cMessage *retryTimer = nullptr;

    // Modular helper routines
    int getMacNumericId() const;
    void beginPrsWindow(int prsSlot);
    void notePrsTransmission(int prsSlot);
    PrsWindowResult concludePrsWindow(int prsSlot);
    bool shouldDeferPriority(int framePriority) const;
    void handlePriorityLoss();
    void handlePriorityWin();
    void scheduleRetry(simtime_t delay);
    void onRetryTimer();
    void processSlotResult(bool busy);
    void handlePhyControl(cMessage *msg);
    void processPhyBusyNotification();
    void processPhyIdleNotification();
    void handleTxFailure();
    void receivePrsSignal(int prsSlot, int senderId);

    // Helpers for control messages from PHY
    void onPhyBusy();
    void onPhyIdle();
    void onTxFail();
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
  public:
    virtual void onPrsPhaseStart(int slot, int generation, omnetpp::simtime_t start, omnetpp::simtime_t end);
    virtual void onPrsPhaseEnd(int slot, int generation);
    
    // Helper methods
    virtual void handleUpperLayerFrame(cMessage *msg);
    virtual void handleLowerLayerFrame(cMessage *msg);
    virtual void handleSelfMessage(cMessage *msg) override;
    virtual void handleBackoffTimer();
    virtual void handleTransmissionTimer();
    virtual void handleSifsTimer();
    virtual void handleDifsTimer();
    
    // Network interface configuration
    virtual void configureNetworkInterface() override;
    
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

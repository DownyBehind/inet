//
// Copyright (C) 2025 INET Framework
//
// SPDX-License-Identifier: LGPL-3.0-or-later
//

#ifndef __INET_IEEE1901PHY_H
#define __INET_IEEE1901PHY_H

#include "inet/common/INETDefs.h"
#include "inet/common/lifecycle/ModuleOperations.h"
#include "inet/common/lifecycle/OperationalBase.h"
#include "inet/linklayer/plc/PLCFrame_m.h"

namespace inet {

/**
 * IEEE 1901 Physical Layer (PHY) implementation for Power Line Communication.
 * 
 * This module implements the physical layer protocol as specified in the IEEE 1901
 * standard for broadband over power line networks. It provides modulation/demodulation,
 * channel coding, and bit error rate simulation suitable for PLC environments.
 * 
 * The module supports:
 *  - Configurable data rates (2-14 Mbps)
 *  - Bit Error Rate (BER) calculation based on SNR
 *  - Packet error simulation based on frame length and BER
 *  - Signal-to-Noise Ratio modeling for PLC channels
 *  - Frame forwarding with error simulation
 */
class INET_API IEEE1901Phy : public cSimpleModule
{
  protected:
    // Module parameters
    double dataRate;              // Physical layer data rate (2-14 Mbps)
    double transmitPower;         // Transmission power in dBm
    double noisePower;            // Noise power in dBm
    double channelAttenuation;    // Channel attenuation factor
    bool enableBER;               // Enable/disable BER simulation
    double baseSNR;               // Base SNR for BER calculation
    
    // BER calculation parameters
    double berAlpha;              // BER exponential mapping parameter (alpha)
    double berBeta;               // BER exponential mapping parameter (beta)
    double maxBER;                // Maximum BER value (saturation)
    double minSNR;                // Minimum SNR for BER calculation
    double maxSNR;                // Maximum SNR for BER calculation
    
    // State variables
    bool isTransmitting;
    bool isReceiving;
    simtime_t currentTxDuration;
    
    // Statistics
    long numFramesSent;
    long numFramesReceived;
    long numFramesDroppedBER;
    long numBitsTransmitted;
    long numBitErrors;
    double totalSNR;
    long snrSamples;
    
    // Signals for statistics
    static simsignal_t framesSentSignal;
    static simsignal_t framesReceivedSignal;
    static simsignal_t framesDroppedBERSignal;
    static simsignal_t berSignal;
    static simsignal_t snrSignal;
    static simsignal_t packetErrorRateSignal;

  protected:
    // Lifecycle methods
    virtual void initialize(int stage) override;
    virtual int numInitStages() const override { return NUM_INIT_STAGES; }
    virtual void handleMessage(cMessage *msg) override;
    virtual void finish() override;
    
    // PHY layer methods
    virtual double getBitErrorRate(double snr);
    virtual double calculateSNR();
    virtual double calculatePacketErrorRate(int payloadLengthBytes, double ber);
    virtual bool shouldDropFrame(int payloadLengthBytes, double ber);
    
    // Frame processing methods
    virtual void handleFrameFromMac(PLCFrame *frame);
    virtual void handleFrameFromChannel(PLCFrame *frame);
    virtual void sendFrameToMac(PLCFrame *frame);
    virtual void sendFrameToChannel(PLCFrame *frame);
    
    // Transmission timing
    virtual simtime_t calculateTransmissionDuration(PLCFrame *frame);
    virtual void scheduleTransmissionEnd(simtime_t duration);
    
    // Helper methods
    virtual void updateDisplayString();
    virtual void recordStatistics();
    virtual double dbmToLinear(double dbm);
    virtual double linearToDbm(double linear);

  public:
    IEEE1901Phy() {}
    virtual ~IEEE1901Phy();
    
    // Public interface for other modules
    virtual double getCurrentDataRate() const { return dataRate; }
    virtual double getCurrentSNR() const;
    virtual double getCurrentBER() const;
};

} // namespace inet

#endif

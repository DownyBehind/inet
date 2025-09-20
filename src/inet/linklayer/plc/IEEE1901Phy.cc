//
// Copyright (C) 2025 INET Framework
//
// SPDX-License-Identifier: LGPL-3.0-or-later
//

#include "inet/linklayer/plc/IEEE1901Phy.h"
#include "inet/common/ModuleAccess.h"
#include "inet/common/ProtocolTag_m.h"
#include "inet/common/lifecycle/ModuleOperations.h"
#include "inet/common/InitStages.h"
#include <math.h>

namespace inet {
static const char *INJECT_DC_MSG = "__inject_dc_request";

Define_Module(IEEE1901Phy);

// Signal definitions
simsignal_t IEEE1901Phy::framesSentSignal = cComponent::registerSignal("framesSent");
simsignal_t IEEE1901Phy::framesReceivedSignal = cComponent::registerSignal("framesReceived");
simsignal_t IEEE1901Phy::framesDroppedBERSignal = cComponent::registerSignal("framesDroppedBER");
simsignal_t IEEE1901Phy::berSignal = cComponent::registerSignal("ber");
simsignal_t IEEE1901Phy::snrSignal = cComponent::registerSignal("snr");
simsignal_t IEEE1901Phy::packetErrorRateSignal = cComponent::registerSignal("packetErrorRate");

IEEE1901Phy::~IEEE1901Phy()
{
    EV_DEBUG << "IEEE1901Phy destructor called" << endl;
    // Cancel and delete reusable timers
    if (txEndMsg) {
        if (txEndMsg->isScheduled()) cancelEvent(txEndMsg);
        delete txEndMsg; txEndMsg = nullptr;
    }
    if (deliverToMacMsg) {
        if (deliverToMacMsg->isScheduled()) cancelEvent(deliverToMacMsg);
        delete deliverToMacMsg; deliverToMacMsg = nullptr;
    }
    // Dispose any undelivered frames
    while (!rxDeliverQueue.empty()) { delete rxDeliverQueue.front(); rxDeliverQueue.pop_front(); }
    rxDeliverTimes.clear();
    // Dispose any pending TX frames
    while (!txQueue.empty()) { delete txQueue.front(); txQueue.pop_front(); }
}

void IEEE1901Phy::initialize(int stage)
{
    EV_DEBUG << "IEEE1901Phy::initialize() stage " << stage << endl;
    
    if (stage == INITSTAGE_LOCAL) {
        // Read parameters from NED file
        dataRate = par("dataRate").doubleValue();
        transmitPower = par("transmitPower").doubleValue();
        noisePower = par("noisePower").doubleValue();
        channelAttenuation = par("channelAttenuation").doubleValue();
        enableBER = par("enableBER").boolValue();
        baseSNR = par("baseSNR").doubleValue();
        
        // BER calculation parameters
        berAlpha = par("berAlpha").doubleValue();
        berBeta = par("berBeta").doubleValue();
        maxBER = par("maxBER").doubleValue();
        minSNR = par("minSNR").doubleValue();
        maxSNR = par("maxSNR").doubleValue();
        
        EV_INFO << "IEEE1901Phy initialized with parameters:" << endl;
        EV_INFO << "  dataRate: " << dataRate/1e6 << " Mbps" << endl;
        EV_INFO << "  transmitPower: " << transmitPower << " dBm" << endl;
        EV_INFO << "  noisePower: " << noisePower << " dBm" << endl;
        EV_INFO << "  channelAttenuation: " << channelAttenuation << " dB" << endl;
        EV_INFO << "  enableBER: " << (enableBER ? "true" : "false") << endl;
        EV_INFO << "  baseSNR: " << baseSNR << " dB" << endl;
        EV_INFO << "  berAlpha: " << berAlpha << endl;
        EV_INFO << "  berBeta: " << berBeta << endl;
        EV_INFO << "  maxBER: " << maxBER << endl;
        
        // Initialize state variables
        isTransmitting = false;
        isReceiving = false;
        currentTxDuration = 0;
        
        // Initialize statistics
        numFramesSent = 0;
        numFramesReceived = 0;
        numFramesDroppedBER = 0;
        numBitsTransmitted = 0;
        numBitErrors = 0;
        totalSNR = 0.0;
        snrSamples = 0;
        
        EV_DEBUG << "IEEE1901Phy initialization stage " << stage << " completed" << endl;
        // Test-only: schedule DC_REQUEST injection if enabled on this PHY (e.g., EVSE side)
        if (hasPar("testInjectDcAtEvse") && (bool)par("testInjectDcAtEvse")) {
            cMessage *m = new cMessage(INJECT_DC_MSG);
            scheduleAt(simTime() + 0.2001, m);
        }
        // Create reusable delivery self-message
        if (!deliverToMacMsg)
            deliverToMacMsg = new cMessage("deliverToMac");
    }
    else if (stage == INITSTAGE_PHYSICAL_LAYER) {
        EV_DEBUG << "IEEE1901Phy physical layer initialization" << endl;
        
        // Register signals for statistics collection
        WATCH(numFramesSent);
        WATCH(numFramesReceived);
        WATCH(numFramesDroppedBER);
        WATCH(numBitsTransmitted);
        WATCH(numBitErrors);
        WATCH(isTransmitting);
        WATCH(isReceiving);
        WATCH(dataRate);
        
        updateDisplayString();
    }
}

void IEEE1901Phy::handleMessage(cMessage *msg)
{
    EV_DEBUG << "IEEE1901Phy::handleMessage() - received message: " << msg->getName() << endl;
    if (msg->isSelfMessage() && msg == txEndMsg) {
        isTransmitting = false;
        if (!txQueue.empty()) {
            PLCFrame *next = txQueue.front();
            txQueue.pop_front();
            EV_INFO << "PHY TX end; dequeue next (q=" << txQueue.size() << ")" << endl;
            handleFrameFromMac(next);
        }
        // do not delete reusable self-message
        return;
    }
    if (msg->isSelfMessage() && msg == deliverToMacMsg) {
        // Pop and deliver all due frames in order
        while (!rxDeliverQueue.empty() && !rxDeliverTimes.empty() && rxDeliverTimes.front() <= simTime()) {
            PLCFrame *f = rxDeliverQueue.front();
            rxDeliverQueue.pop_front();
            rxDeliverTimes.pop_front();
            send(static_cast<cMessage*>(f), "upperLayerOut");
        }
        // Reschedule if more pending
        if (!rxDeliverTimes.empty()) {
            scheduleAt(rxDeliverTimes.front(), deliverToMacMsg);
        }
        return;
    }
    if (msg->isSelfMessage() && !strcmp(msg->getName(), INJECT_DC_MSG)) {
        EV_WARN << "[TEST] Injecting synthetic DC_REQUEST at PHY " << getFullPath() << " t=" << simTime() << endl;
        PLCFrame *f = new PLCFrame("DC_REQUEST");
        f->setFrameType(2);
        f->setPriority(3);
        f->setSrcAddr(0);
        f->setDestAddr(1);
        f->setPayloadLength(300);
        f->setByteLength(300);
        f->setAckRequired(false);
        sendFrameToMac(f);
        // Single-shot injection; delete timer message (no runtime param mutation)
        if (msg->getOwner() != this) take(msg);
        delete msg;
        return;
    }
    
    PLCFrame *frame = dynamic_cast<PLCFrame *>(msg);
    if (!frame) {
        EV_ERROR << "Received non-PLCFrame message: " << msg->getClassName() << endl;
        delete msg;
        return;
    }
    
    if (msg->getArrivalGate()->isName("upperLayerIn")) {
        EV_DEBUG << "Handling frame from MAC layer" << endl;
        handleFrameFromMac(frame);
    }
    else if (msg->getArrivalGate()->isName("lowerLayerIn")) {
        EV_DEBUG << "Handling frame from channel" << endl;
        handleFrameFromChannel(frame);
    }
    else {
        EV_ERROR << "Unknown arrival gate: " << msg->getArrivalGate()->getName() << endl;
        delete msg;
    }
}

double IEEE1901Phy::getBitErrorRate(double snr)
{
    EV_DEBUG << "IEEE1901Phy::getBitErrorRate() - SNR: " << snr << " dB" << endl;
    
    if (!enableBER) {
        EV_DEBUG << "BER simulation disabled, returning 0" << endl;
        return 0.0;
    }
    
    // Clamp SNR to valid range
    if (snr < minSNR) snr = minSNR;
    if (snr > maxSNR) snr = maxSNR;
    
    // Approximate exponential mapping for PLC BER
    // BER = alpha * exp(-beta * SNR)
    // This provides a realistic BER curve for power line communication
    double ber = berAlpha * exp(-berBeta * snr);
    
    // Apply maximum BER limit
    if (ber > maxBER) {
        ber = maxBER;
    }
    
    EV_DEBUG << "Calculated BER: " << ber << " for SNR: " << snr << " dB" << endl;
    
    return ber;
}

double IEEE1901Phy::calculateSNR()
{
    EV_DEBUG << "IEEE1901Phy::calculateSNR()" << endl;
    
    // Calculate SNR considering transmission power, channel attenuation, and noise
    // SNR = P_tx - attenuation - P_noise
    double snr = transmitPower - channelAttenuation - noisePower;
    
    // Add some random variation to simulate channel conditions
    double variation = normal(0, 2.0); // 2 dB standard deviation
    snr += variation;
    
    EV_DEBUG << "Calculated SNR: " << snr << " dB" << endl;
    EV_DEBUG << "  Tx Power: " << transmitPower << " dBm" << endl;
    EV_DEBUG << "  Attenuation: " << channelAttenuation << " dB" << endl;
    EV_DEBUG << "  Noise Power: " << noisePower << " dBm" << endl;
    EV_DEBUG << "  Random Variation: " << variation << " dB" << endl;
    
    return snr;
}

double IEEE1901Phy::calculatePacketErrorRate(int payloadLengthBytes, double ber)
{
    EV_DEBUG << "IEEE1901Phy::calculatePacketErrorRate() - payload: " << payloadLengthBytes << " bytes, BER: " << ber << endl;
    
    if (ber <= 0.0) {
        return 0.0;
    }
    
    // Calculate packet error rate: PER = 1 - (1 - BER)^(L*8)
    // where L is payload length in bytes
    int numBits = payloadLengthBytes * 8;
    double per = 1.0 - pow(1.0 - ber, numBits);
    
    EV_DEBUG << "Calculated PER: " << per << " for " << numBits << " bits" << endl;
    
    return per;
}

bool IEEE1901Phy::shouldDropFrame(int payloadLengthBytes, double ber)
{
    EV_DEBUG << "IEEE1901Phy::shouldDropFrame() - payload: " << payloadLengthBytes << " bytes, BER: " << ber << endl;
    
    if (!enableBER || ber <= 0.0) {
        EV_DEBUG << "No frame drop - BER disabled or zero BER" << endl;
        return false;
    }
    
    double per = calculatePacketErrorRate(payloadLengthBytes, ber);
    double randomValue = uniform(0, 1);
    bool drop = (randomValue < per);
    
    EV_INFO << "Frame drop decision:" << endl;
    EV_INFO << "  Payload length: " << payloadLengthBytes << " bytes" << endl;
    EV_INFO << "  BER: " << ber << endl;
    EV_INFO << "  PER: " << per << endl;
    EV_INFO << "  Random value: " << randomValue << endl;
    EV_INFO << "  Drop frame: " << (drop ? "YES" : "NO") << endl;
    
    return drop;
}

void IEEE1901Phy::handleFrameFromMac(PLCFrame *frame)
{
    EV_DEBUG << "IEEE1901Phy::handleFrameFromMac() - frame type: " << frame->getFrameType() << endl;
    
    if (isTransmitting) {
        // Enqueue instead of dropping if capacity allows
        if ((int)txQueue.size() < txQueueCapacity) {
            EV_INFO << "PHY busy; enqueue frame from MAC (q=" << txQueue.size()+1 << "/" << txQueueCapacity << ")" << endl;
            txQueue.push_back(frame);
        } else {
            EV_WARN << "PHY busy; TX queue full -> dropping frame from MAC" << endl;
            delete frame;
        }
        return;
    }
    
    EV_INFO << "Processing frame from MAC layer:" << endl;
    EV_INFO << "  Frame Type: " << frame->getFrameType() << endl;
    EV_INFO << "  Priority: " << frame->getPriority() << endl;
    EV_INFO << "  Payload Length: " << frame->getPayloadLength() << " bytes" << endl;
    
    // Calculate transmission duration
    simtime_t txDuration = calculateTransmissionDuration(frame);
    
    // Set transmission state
    isTransmitting = true;
    currentTxDuration = txDuration;
    
    // Update statistics
    numFramesSent++;
    numBitsTransmitted += frame->getPayloadLength() * 8;
    emit(framesSentSignal, numFramesSent);
    
    EV_INFO << "Starting transmission - duration: " << txDuration << " s" << endl;
    
    // Send frame to channel (no errors on transmission side)
    sendFrameToChannel(frame);
    
    // Schedule transmission end using reusable self-message
    scheduleTransmissionEnd(txDuration);
    
    updateDisplayString();
}

void IEEE1901Phy::handleFrameFromChannel(PLCFrame *frame)
{
    EV_DEBUG << "IEEE1901Phy::handleFrameFromChannel() - frame type: " << frame->getFrameType() << endl;
    
    EV_INFO << "Processing frame from channel:" << endl;
    EV_INFO << "  Frame Type: " << frame->getFrameType() << endl;
    EV_INFO << "  Priority: " << frame->getPriority() << endl;
    EV_INFO << "  Payload Length: " << frame->getPayloadLength() << " bytes" << endl;
    
    // Calculate current SNR and BER
    double snr = calculateSNR();
    double ber = getBitErrorRate(snr);
    
    // Update statistics
    totalSNR += snr;
    snrSamples++;
    emit(snrSignal, snr);
    emit(berSignal, ber);
    
    // Cache payload length and PER for later statistics emission
    const int payloadLen = frame->getPayloadLength();
    double per = calculatePacketErrorRate(payloadLen, ber);

    // Determine if frame should be dropped due to bit errors
    bool dropFrame = shouldDropFrame(payloadLen, ber);
    
    if (dropFrame) {
        EV_INFO << "Frame dropped due to bit errors (BER-based simulation)" << endl;
        numFramesDroppedBER++;
        emit(framesDroppedBERSignal, numFramesDroppedBER);
        
        // Calculate estimated bit errors for statistics
        int estimatedBitErrors = (int)(payloadLen * 8 * ber);
        numBitErrors += estimatedBitErrors;
        
        delete frame;
    } else {
    EV_INFO << "Frame received successfully at " << getFullPath() << ", forwarding to MAC: name="
            << frame->getName() << " src=" << frame->getSrcAddr() << " dest=" << frame->getDestAddr()
            << " prio=CA" << frame->getPriority() << endl;
        numFramesReceived++;
        emit(framesReceivedSignal, numFramesReceived);
        
        // Forward frame to MAC layer
        sendFrameToMac(frame);
    }
    
    // Emit packet error rate (computed before potential deletion/ownership transfer)
    emit(packetErrorRateSignal, per);
    
    updateDisplayString();
}

void IEEE1901Phy::sendFrameToMac(PLCFrame *frame)
{
    EV_DEBUG << "IEEE1901Phy::sendFrameToMac()" << endl;
    
    // Add some reception delay to simulate processing time
    simtime_t receptionDelay = (20 + frame->getPayloadLength()) * 8.0 / dataRate;  // 20 bytes header + payload
    
    EV_INFO << "Forwarding frame to MAC layer - reception delay: " << receptionDelay << " s" << endl;
    // Mailbox delivery: keep ownership until scheduled delivery to avoid cross-module UAF
    rxDeliverQueue.push_back(frame);
    rxDeliverTimes.push_back(simTime() + receptionDelay);
    if (!deliverToMacMsg->isScheduled())
        scheduleAt(rxDeliverTimes.front(), deliverToMacMsg);
}

void IEEE1901Phy::sendFrameToChannel(PLCFrame *frame)
{
    EV_DEBUG << "IEEE1901Phy::sendFrameToChannel()" << endl;
    
    EV_INFO << "Sending frame to channel from " << getFullPath() << ": name="
            << frame->getName() << " src=" << frame->getSrcAddr() << " dest=" << frame->getDestAddr()
            << " prio=CA" << frame->getPriority() << " rate=" << dataRate/1e6 << "Mbps" << endl;
    
    send(static_cast<cMessage*>(frame), "lowerLayerOut");
}

simtime_t IEEE1901Phy::calculateTransmissionDuration(PLCFrame *frame)
{
    EV_DEBUG << "IEEE1901Phy::calculateTransmissionDuration()" << endl;
    
    // Calculate transmission time based on frame size and data rate
    // Duration = (frame_size_bits) / (data_rate_bps)
    int frameSizeBits = (20 + frame->getPayloadLength()) * 8;  // 20 bytes header + payload, convert to bits
    simtime_t duration = (double)frameSizeBits / dataRate;
    
    EV_DEBUG << "Transmission duration calculation:" << endl;
    EV_DEBUG << "  Frame size: " << frameSizeBits << " bits" << endl;
    EV_DEBUG << "  Data rate: " << dataRate/1e6 << " Mbps" << endl;
    EV_DEBUG << "  Duration: " << duration << " s" << endl;
    
    return duration;
}

void IEEE1901Phy::scheduleTransmissionEnd(simtime_t duration)
{
    EV_DEBUG << "IEEE1901Phy::scheduleTransmissionEnd() - duration: " << duration << " s" << endl;
    
    // Schedule transmission completion
    if (!txEndMsg)
        txEndMsg = new cMessage("transmissionEnd");
    if (txEndMsg->isScheduled())
        cancelEvent(txEndMsg);
    if (txEndMsg->getOwner() != this) take(txEndMsg);
    scheduleAt(simTime() + duration, txEndMsg);
}

void IEEE1901Phy::finish()
{
    EV_DEBUG << "IEEE1901Phy::finish()" << endl;
    EV_INFO << "IEEE1901Phy::finish() called at t=" << simTime() << " module=" << getFullPath() << endl;
    
    recordScalar("frames sent", numFramesSent);
    recordScalar("frames received", numFramesReceived);
    recordScalar("frames dropped (BER)", numFramesDroppedBER);
    recordScalar("bits transmitted", numBitsTransmitted);
    recordScalar("bit errors", numBitErrors);
    
    EV_INFO << "IEEE1901Phy statistics:" << endl;
    EV_INFO << "  Frames sent: " << numFramesSent << endl;
    EV_INFO << "  Frames received: " << numFramesReceived << endl;
    EV_INFO << "  Frames dropped (BER): " << numFramesDroppedBER << endl;
    EV_INFO << "  Bits transmitted: " << numBitsTransmitted << endl;
    EV_INFO << "  Bit errors: " << numBitErrors << endl;
    
    // Calculate derived statistics
    // Report RX-side success ratio based on channel reception outcomes
    if (numFramesReceived + numFramesDroppedBER > 0) {
        double rxSuccessRate = (double)numFramesReceived / (numFramesReceived + numFramesDroppedBER) * 100.0;
        recordScalar("rx success rate (%)", rxSuccessRate);
        EV_INFO << "  RX success rate: " << rxSuccessRate << "%" << endl;
    }
    
    if (numBitsTransmitted > 0) {
        double actualBER = (double)numBitErrors / numBitsTransmitted;
        recordScalar("actual BER", actualBER);
        EV_INFO << "  Actual BER: " << actualBER << endl;
    }
    
    if (snrSamples > 0) {
        double avgSNR = totalSNR / snrSamples;
        recordScalar("average SNR (dB)", avgSNR);
        EV_INFO << "  Average SNR: " << avgSNR << " dB" << endl;
    }
    // Cleanup reusable self-message safely
    if (txEndMsg) {
        if (txEndMsg->isScheduled()) cancelEvent(txEndMsg);
        delete txEndMsg; txEndMsg = nullptr;
    }
}

// Helper method implementations
void IEEE1901Phy::updateDisplayString()
{
    if (hasGUI()) {
        char buf[100];
        sprintf(buf, "Rate: %.1fM\nSent: %ld\nRcvd: %ld\nDropped: %ld\n%s", 
                dataRate/1e6, numFramesSent, numFramesReceived, numFramesDroppedBER,
                isTransmitting ? "TX" : (isReceiving ? "RX" : "IDLE"));
        getDisplayString().setTagArg("t", 0, buf);
    }
}

void IEEE1901Phy::recordStatistics()
{
    EV_DEBUG << "IEEE1901Phy::recordStatistics()" << endl;
    // Additional statistics recording if needed
}

double IEEE1901Phy::dbmToLinear(double dbm)
{
    return pow(10.0, dbm / 10.0);
}

double IEEE1901Phy::linearToDbm(double linear)
{
    return 10.0 * log10(linear);
}

// Public interface methods
double IEEE1901Phy::getCurrentSNR() const
{
    return baseSNR; // Simplified for external access
}

double IEEE1901Phy::getCurrentBER() const
{
    if (!enableBER) return 0.0;
    return const_cast<IEEE1901Phy*>(this)->getBitErrorRate(getCurrentSNR());
}

} // namespace inet

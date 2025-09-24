//
// Copyright (C) 2025 INET Framework
//
// SPDX-License-Identifier: LGPL-3.0-or-later
//

package inet.linklayer.plc;

//
// IEEE 1901 Physical Layer (PHY) module for Power Line Communication.
//
// This module implements the physical layer protocol as specified in the IEEE 1901
// standard for broadband over power line networks. It provides signal transmission,
// bit error rate simulation, and channel modeling suitable for PLC environments.
//
// Key Features:
//  - Configurable data rates from 2 to 14 Mbps
//  - Bit Error Rate (BER) simulation based on Signal-to-Noise Ratio (SNR)
//  - Packet error simulation using exponential BER mapping
//  - Channel attenuation and noise modeling
//  - Frame forwarding with error-based dropping
//  - Comprehensive statistics collection
//
// BER Calculation:
//  The module uses an approximate exponential mapping to calculate BER from SNR:
//  BER = alpha * exp(-beta * SNR)
//
// Packet Error Rate:
//  Frame dropping probability is calculated as:
//  PER = 1 - (1 - BER)^(payloadLength * 8)
//
// @see ~PLCFrame, ~IEEE1901Mac
//
simple IEEE1901Phy
{
    parameters:
        @display("i=block/layer");
        @class(IEEE1901Phy);
        @signal[framesSent](type=long);
        @signal[framesReceived](type=long);
        @signal[framesDroppedBER](type=long);
        @signal[ber](type=double);
        @signal[snr](type=double);
        @signal[packetErrorRate](type=double);
        @statistic[framesSent](title="frames sent"; source=framesSent; record=count,last; interpolationmode=none);
        @statistic[framesReceived](title="frames received"; source=framesReceived; record=count,last; interpolationmode=none);
        @statistic[framesDroppedBER](title="frames dropped (BER)"; source=framesDroppedBER; record=count,last; interpolationmode=none);
        @statistic[ber](title="bit error rate"; source=ber; record=stats,histogram,vector; interpolationmode=none);
        @statistic[snr](title="signal to noise ratio"; source=snr; record=stats,histogram,vector; interpolationmode=none);
        @statistic[packetErrorRate](title="packet error rate"; source=packetErrorRate; record=stats,histogram,vector; interpolationmode=none);

        // Physical layer parameters
        double dataRate @unit(bps) = default(10Mbps);        // Physical layer data rate (2-14 Mbps)
        double transmitPower @unit(dBm) = default(10dBm);    // Transmission power
        double noisePower @unit(dBm) = default(-90dBm);      // Noise power level
        double channelAttenuation @unit(dB) = default(20dB); // Channel attenuation
        
        // Signal quality parameters
        double baseSNR @unit(dB) = default(25dB);            // Base signal-to-noise ratio
        bool enableBER = default(true);                      // Enable bit error rate simulation
        bool enableCollisionModel = default(false);          // Optional explicit same-slot collision model (default off)
        
        // BER calculation parameters (exponential mapping)
        double berAlpha = default(0.5);                      // BER exponential coefficient (alpha)
        double berBeta = default(0.4);                       // BER exponential decay rate (beta)
        double maxBER = default(0.1);                        // Maximum bit error rate (10%)
        double minSNR @unit(dB) = default(0dB);              // Minimum SNR for BER calculation
        double maxSNR @unit(dB) = default(50dB);             // Maximum SNR for BER calculation
        
        // Channel modeling parameters
        double channelVariation @unit(dB) = default(2dB);    // Random channel variation (std dev)
        double fadingDepth @unit(dB) = default(10dB);        // Maximum fading depth
        double fadingDuration @unit(s) = default(1ms);       // Fading coherence time
        
        // Frame processing parameters
        double processingDelay @unit(s) = default(1us);      // PHY processing delay
        double propagationDelay @unit(s) = default(0.1us);   // Signal propagation delay
        
        // Power line specific parameters
        double lineImpedance @unit(ohm) = default(50ohm);    // Characteristic line impedance
        double frequency @unit(Hz) = default(30MHz);         // Center frequency for PLC
        double bandwidth @unit(Hz) = default(28MHz);         // Available bandwidth
        
        // Error correction capabilities
        bool enableFEC = default(true);                      // Forward Error Correction
        double fecCodingRate = default(0.5);                 // FEC coding rate (0.5 = rate 1/2)
        double fecCodingGain @unit(dB) = default(6dB);       // FEC coding gain
        
        // Adaptive parameters
        bool adaptiveRate = default(false);                  // Enable adaptive data rate
        double rateAdaptationThreshold @unit(dB) = default(20dB); // SNR threshold for rate adaptation
        double minDataRate @unit(bps) = default(2Mbps);      // Minimum data rate
        double maxDataRate @unit(bps) = default(14Mbps);     // Maximum data rate
        
        // Test-only diagnostics (default disabled)
        bool testInjectDcAtEvse = default(false);
        int testInjectRepeat = default(0);
        
    gates:
        input upperLayerIn @labels(PLCFrame);     // From MAC layer
        output upperLayerOut @labels(PLCFrame);   // To MAC layer
        input lowerLayerIn @labels(PLCFrame);     // From channel
        output lowerLayerOut @labels(PLCFrame);   // To channel
}

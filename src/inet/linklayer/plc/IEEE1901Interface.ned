//
// Copyright (C) 2025 INET Framework
//
// SPDX-License-Identifier: LGPL-3.0-or-later
//

package inet.linklayer.plc;

import inet.networklayer.common.NetworkInterface;

//
// IEEE 1901 Power Line Communication Interface
//
// This compound module provides a complete IEEE 1901 PLC network interface
// by combining the IEEE1901Mac and IEEE1901Phy modules. It can be used as a
// drop-in replacement for EthernetInterface in INET simulations.
//
// The interface implements the complete IEEE 1901 protocol stack:
//  - Medium Access Control (MAC) with priority resolution and HomePlug 1.0 backoff
//  - Physical Layer (PHY) with configurable data rates and BER simulation
//  - Power Line Communication channel modeling
//
// Key Features:
//  - HomePlug 1.0 compatible priority resolution (CA0-CA3)
//  - Configurable backoff procedure with BPC limits
//  - IEEE 1901 compliant data rates (2-14 Mbps)
//  - Realistic BER simulation based on SNR
//  - Complete PLC protocol stack in a single interface
//
// Usage:
//  This module can replace EthernetInterface in existing INET network
//  configurations to simulate power line communication networks.
//
// @see ~IEEE1901Mac, ~IEEE1901Phy, ~PLCFrame
//
module IEEE1901Interface extends NetworkInterface
{
    parameters:
        @display("i=block/ifcard;bgb=400,300");
        
        // Interface identification
        string interfaceTableModule = default(absPath(".interfaceTable"));
        *.interfaceTableModule = default(absPath(this.interfaceTableModule));
        string address @mutable = default("auto");
        double bitrate @unit(bps) = default(10Mbps);
        
        // Exposed PHY parameters
        double dataRate @unit(bps) = default(10Mbps);        // Physical layer data rate (2-14 Mbps)
        double snr @unit(dB) = default(25dB);                // Signal-to-noise ratio for BER calculation
        
        // Exposed MAC parameters
        bool usePriorityResolution = default(true);          // Enable HomePlug 1.0 priority resolution
        int maxBPC = default(7);                             // Maximum Backoff Procedure Counter
        
        // Additional interface parameters
        int mtu = default(1500);                             // Maximum Transmission Unit
        bool promiscuous = default(false);                   // Promiscuous mode
        string multicastGroups = default("");               // Multicast groups to join
        
        // Power Line Communication specific parameters
        double transmitPower @unit(dBm) = default(10dBm);    // Transmission power
        double noisePower @unit(dBm) = default(-90dBm);      // Noise power level
        double channelAttenuation @unit(dB) = default(20dB); // Channel attenuation
        
        // Priority Resolution parameters
        double prs0Duration @unit(s) = default(35.84us);     // PRS0 slot duration
        double prs1Duration @unit(s) = default(35.84us);     // PRS1 slot duration
        
        // Backoff algorithm parameters
        double slotTime @unit(s) = default(9us);             // Backoff slot duration
        double sifsTime @unit(s) = default(10us);            // Short Inter-Frame Space
        double difsTime @unit(s) = default(34us);            // DCF Inter-Frame Space
        int cwMin = default(16);                             // Minimum contention window
        int cwMax = default(1024);                           // Maximum contention window
        
        // BER simulation parameters
        bool enableBER = default(true);                      // Enable bit error rate simulation
        double berAlpha = default(0.5);                      // BER exponential coefficient
        double berBeta = default(0.4);                       // BER exponential decay rate
        double maxBER = default(0.1);                        // Maximum bit error rate
        
        // Network parameters
        int nodeAddress = default(0);                        // Node address in PLC network
        int networkId = default(1);                          // Network identifier
        int defaultPriority = default(0);                    // Default frame priority (CA0-CA3)
        
        // Performance tuning parameters
        int maxQueueSize = default(100);                     // MAC queue size
        int maxRetries = default(7);                         // Maximum retransmission attempts
        bool useAcknowledgments = default(true);             // Enable ACK mechanism
        double ackTimeout @unit(s) = default(100us);         // ACK timeout
        
    gates:
        // Accept PLCFrame messages from/to upper layer (SlacApp uses PLCFrame)
        input upperLayerIn @labels(PLCFrame);
        output upperLayerOut @labels(PLCFrame);
        inout plc @labels(PLCFrame);                         // Wired PLC medium connection
        
    submodules:
        mac: IEEE1901Mac {
            parameters:
                @display("p=200,100");
                
                // Pass through exposed MAC parameters
                maxRetries = parent.maxRetries;
                slotTime = parent.slotTime;
                sifsTime = parent.sifsTime;
                difsTime = parent.difsTime;
                cwMin = parent.cwMin;
                cwMax = parent.cwMax;
                
                // Priority Resolution parameters
                prs0Duration = parent.prs0Duration;
                prs1Duration = parent.prs1Duration;
                
                // Network parameters
                nodeAddress = parent.nodeAddress;
                networkId = parent.networkId;
                defaultPriority = parent.defaultPriority;
                
                // Buffer and performance parameters
                maxQueueSize = parent.maxQueueSize;
                useAcknowledgments = parent.useAcknowledgments;
                ackTimeout = parent.ackTimeout;
        }
        
        phy: IEEE1901Phy {
            parameters:
                @display("p=200,200");
                
                // Pass through exposed PHY parameters
                dataRate = parent.dataRate;
                transmitPower = parent.transmitPower;
                noisePower = parent.noisePower;
                channelAttenuation = parent.channelAttenuation;
                baseSNR = parent.snr;
                
                // BER simulation parameters
                enableBER = parent.enableBER;
                berAlpha = parent.berAlpha;
                berBeta = parent.berBeta;
                maxBER = parent.maxBER;
        }
        
    connections:
        // Connect upper layer to MAC
        upperLayerIn --> mac.upperLayerIn;
        mac.upperLayerOut --> upperLayerOut;
        
        // Connect MAC to PHY
        mac.lowerLayerOut --> phy.upperLayerIn;
        phy.upperLayerOut --> mac.lowerLayerIn;

        phy.lowerLayerOut --> plc$o;
        plc$i --> phy.lowerLayerIn;
}

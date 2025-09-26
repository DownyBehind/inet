//
// Copyright (C) 2025 INET Framework
//
// SPDX-License-Identifier: LGPL-3.0-or-later
//

package inet.linklayer.plc;

import inet.linklayer.base.MacProtocolBase;

//
// IEEE 1901 Medium Access Control (MAC) module for Power Line Communication.
//
// This module implements the MAC layer protocol as specified in the IEEE 1901
// standard for broadband over power line networks. It provides medium access
// control, collision avoidance, and frame transmission coordination suitable
// for PLC network environments.
//
// The module supports:
//  - CSMA/CA (Carrier Sense Multiple Access with Collision Avoidance)
//  - Priority-based channel access (CA0-CA3)
//  - Backoff algorithms for collision resolution
//  - Frame acknowledgment mechanisms
//  - Power line specific timing parameters
//
// Frame types supported:
//  - Data frames for user traffic
//  - Beacon frames for network coordination
//  - Control frames for network management
//
// @see ~PLCFrame, ~PLCBeaconFrame, ~PLCDataFrame, ~PLCControlFrame
//
simple IEEE1901Mac extends MacProtocolBase
{
    parameters:
        @display("i=block/layer");
        @class(IEEE1901Mac);
        @signal[framesSent](type=long);
        @signal[framesReceived](type=long);
        @signal[framesDropped](type=long);
        @signal[collisions](type=long);
        @signal[backoffs](type=long);
        @statistic[framesSent](title="frames sent"; source=framesSent; record=count,last; interpolationmode=none);
        @statistic[framesReceived](title="frames received"; source=framesReceived; record=count,last; interpolationmode=none);
        @statistic[framesDropped](title="frames dropped"; source=framesDropped; record=count,last; interpolationmode=none);
        @statistic[collisions](title="collisions"; source=collisions; record=count,last; interpolationmode=none);
        @statistic[backoffs](title="backoffs"; source=backoffs; record=count,last; interpolationmode=none);

        // Physical layer parameters
        double txPower @unit(dBm) = default(20dBm);          // Transmission power
        double bitrate @unit(bps) = default(200Mbps);        // Physical layer bit rate
        
        // MAC protocol parameters
        int maxRetries = default(7);                         // Maximum number of retransmission attempts
        double slotTime @unit(s) = default(9us);             // Duration of a backoff slot
        double sifsTime @unit(s) = default(10us);            // Short Inter-Frame Space
        double difsTime @unit(s) = default(34us);            // DCF Inter-Frame Space (SIFS + 2*SlotTime)
        
        // Priority Resolution Signal parameters (HomePlug 1.0)
        double prs0Duration @unit(s) = default(35.84us);     // Duration of PRS0 slot
        double prs1Duration @unit(s) = default(35.84us);     // Duration of PRS1 slot
        
        // Contention window parameters
        int cwMin = default(16);                             // Minimum contention window size
        int cwMax = default(1024);                           // Maximum contention window size
        
        // Priority parameters (IEEE 1901 Channel Access priorities)
        int defaultPriority = default(0);                   // Default priority level (CA0=0, CA1=1, CA2=2, CA3=3)
        
        // Buffer parameters
        int maxQueueSize = default(100);                     // Maximum number of frames in transmission queue
        
        // Acknowledgment parameters
        bool useAcknowledgments = default(true);             // Enable/disable acknowledgment mechanism
        double ackTimeout @unit(s) = default(100us);         // Timeout for waiting for acknowledgments
        
        // Network parameters
        int nodeAddress = default(0);                        // Node address in PLC network
        int networkId = default(1);                          // Network identifier
        
        // Beacon parameters (for coordinated networks)
        bool isCoordinator = default(false);                 // Whether this node is a network coordinator
        double beaconInterval @unit(s) = default(100ms);     // Beacon transmission interval
        
        // Energy efficiency parameters
        bool powerSaving = default(false);                   // Enable power saving mode
        double sleepDuration @unit(s) = default(10ms);       // Duration of sleep periods in power saving mode
        
        // Channel sensing parameters
        double channelSenseThreshold @unit(dBm) = default(-80dBm);  // Threshold for channel busy detection
        double carrierSenseTime @unit(s) = default(5us);     // Duration of carrier sensing
        
        // Frame timing parameters
        double frameDetectionTime @unit(s) = default(2us);   // Time to detect frame start
        double framePreambleTime @unit(s) = default(8us);    // Preamble duration
        
        // Test-only parameters (disabled by default)
        bool testForceBc0Once = default(false);              // If true, force BC=0 once to provoke a collision in tests
        
    // inherit gates from MacProtocolBase (upper/lower in/out)
}

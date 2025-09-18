//
// Copyright (C) 2025 INET Framework
//
// SPDX-License-Identifier: LGPL-3.0-or-later
//


namespace inet;

//
// This class represents a PLC (Power Line Communication) MAC frame 
// as defined in IEEE 1901 standard for broadband over power line networks.
//
// Frame format follows IEEE 1901 MAC frame structure with the following fields:
// - Frame Type: Indicates the type of frame (DATA, BEACON, CONTROL)
// - Priority: Channel Access priority level (CA0-CA3)
// - Source/Destination Addresses: Node identifiers in the PLC network
// - Payload Length: Length of the data payload
// - ACK Required: Flag indicating if acknowledgment is required
// - Transmission timing information for physical layer coordination
//
class PLCFrame extends cPacket
{
    // Frame Type field - indicates the type of PLC frame
    int frameType;       // 0=DATA, 1=BEACON, 2=CONTROL
    
    // Priority field - Channel Access priority level
    int priority;        // CA0=0, CA1=1, CA2=2, CA3=3
    
    // Address fields for source and destination identification
    int srcAddr;         // Source node address
    int destAddr;        // Destination node address  
    
    // Payload information
    int payloadLength;   // Length of data payload in bytes
    
    // Acknowledgment control
    bool ackRequired;    // Flag indicating if ACK is required
    
    // Transmission timing information for PHY layer coordination
    double txStartTime;  // Transmission start time
    double txEndTime;    // Transmission end time
}

//
// PLC Beacon frame - specialized frame type for network synchronization
// and management in IEEE 1901 networks
//
class PLCBeaconFrame extends PLCFrame
{
    frameType = 1;       // Always BEACON type
    priority = 3;        // Highest priority (CA3)
    ackRequired = false; // Beacons typically don't require ACK
    
    // Beacon-specific fields
    int beaconPeriod;    // Beacon transmission period in microseconds
    int networkId;       // Network identifier
    bool coordinated;    // Indicates if this is a coordinated beacon
}

//
// PLC Data frame - used for actual data transmission in IEEE 1901 networks
//
class PLCDataFrame extends PLCFrame
{
    frameType = 0;       // Always DATA type
    
    // Data-specific fields
    int sequenceNumber;  // Sequence number for ordering and duplicate detection
    bool fragmented;     // Indicates if frame is part of a fragmented transmission
    int fragmentIndex;   // Index of this fragment (if fragmented)
    int totalFragments;  // Total number of fragments (if fragmented)
}

//
// PLC Control frame - used for network control and management
//
class PLCControlFrame extends PLCFrame
{
    frameType = 2;       // Always CONTROL type
    priority = 3;        // High priority for control traffic
    
    // Control-specific fields
    int controlType;     // Type of control message
    int requestId;       // Request identifier for request/response matching
}

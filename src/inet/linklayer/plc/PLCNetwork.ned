//
// Copyright (C) 2025 INET Framework
//
// SPDX-License-Identifier: LGPL-3.0-or-later
//

package inet.linklayer.plc;

import inet.applications.generic.IpvxTrafGen;
import inet.common.scenario.ScenarioManager;
import inet.networklayer.configurator.ipv4.Ipv4NetworkConfigurator;
import inet.physicallayer.wired.common.WireJunction;
import inet.node.base.StandardHostBase;

//
// PLC Node with IEEE 1901 interface
//
module PLCNode extends StandardHostBase
{
    parameters:
        @display("i=device/pc");
        hasUdp = true;
        hasIpv4 = true;
        numLoInterfaces = 1;
        
        // Traffic generation parameters
        double trafficRate @unit(bps) = default(1Mbps);
        int frameSize @unit(B) = default(1500B);
        
    gates:
        inout plcg @labels(PLCFrame);
        
    submodules:
        // IEEE 1901 PLC interface
        plcInterface: IEEE1901Interface {
            @display("p=250,400");
        }
        
        // Traffic generator
        trafficGen: IpvxTrafGen {
            @display("p=150,300");
            parameters:
                packetLength = parent.frameSize;
                sendInterval = exponential(parent.frameSize*8/parent.trafficRate);
                startTime = uniform(0s, 1s);
                stopTime = 10s;
                destAddresses = "node1 node2 node3 node4"; // Will be configured in ini
                protocol = 17; // UDP
        }
        
    connections:
        // Connect traffic generator to network layer
        trafficGen.ipOut --> networkLayer.transportIn++;
        trafficGen.ipIn <-- networkLayer.transportOut++;
        
        // Connect PLC interface to the gate
        plcInterface.upperLayerOut --> networkLayer.ifIn++;
        plcInterface.upperLayerIn <-- networkLayer.ifOut++;
        plcInterface.radioIn <-- plcg;
}

//
// PLC Network with 5 IEEE 1901 nodes connected via shared power line medium
//
// This network demonstrates IEEE 1901 Power Line Communication with:
//  - 5 nodes using IEEE1901Interface
//  - Shared medium simulation using WireJunction
//  - Constant traffic generation for performance evaluation
//  - Priority resolution and backoff algorithm testing
//
// The network topology represents a typical PLC deployment where multiple
// devices share the same power line infrastructure and compete for medium access
// using the IEEE 1901 MAC protocol with HomePlug 1.0 priority resolution.
//
network PLCNetwork
{
    parameters:
        @display("bgb=800,600;bgg=100,1,grey95");
        
        // Network-wide parameters
        int numNodes = default(5);                           // Number of PLC nodes
        
    submodules:
        // Network configuration
        configurator: Ipv4NetworkConfigurator {
            @display("p=100,50");
        }
        
        // Scenario manager for advanced configuration
        scenarioManager: ScenarioManager {
            @display("p=200,50");
        }
        
        // PLC nodes with IEEE 1901 interfaces
        node[numNodes]: PLCNode {
            @display("p=150+150*cos(2*3.14159*index/numNodes),300+150*sin(2*3.14159*index/numNodes)");
        }
        
        // Shared PLC medium (power line)
        plcChannel: WireJunction {
            @display("p=400,300;i=misc/cloud_l");
        }
        
    connections:
        // Connect all nodes to the shared PLC channel
        for i=0..numNodes-1 {
            node[i].plcg <--> PLCMedium <--> plcChannel.port++;
        }
}

//
// Power Line Communication Medium Channel
//
// Models the shared power line medium with appropriate characteristics:
//  - Propagation delay for electrical signals
//  - Bit rate representing the physical medium capacity
//  - Packet error rate and collision detection
//
channel PLCMedium extends ned.DatarateChannel
{
    parameters:
        datarate = default(100Mbps);    // Physical medium capacity
        delay = default(0.1us);         // Electrical signal propagation delay
        per = default(0);               // Packet error rate (handled by PHY layer)
        ber = default(0);               // Bit error rate (handled by PHY layer)
        
        @display("ls=black,3");
        @statistic[channelUtilization](title="channel utilization"; record=last);
        @statistic[throughput](title="throughput"; record=last);
}

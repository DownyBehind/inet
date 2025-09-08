# IEEE 1901 Power Line Communication Example

This example demonstrates IEEE 1901 Power Line Communication (PLC) simulation using the INET Framework.

## Network Topology

The simulation consists of:

- 5 PLC nodes with IEEE1901Interface
- Shared power line medium modeled using WireJunction
- Constant traffic generation (1500-byte frames at 1 Mbps per node)
- Different priority levels for testing priority resolution

## Configurations

### PLCBasic

Basic configuration with the requested parameters:

- SNR: 20 dB
- Maximum BPC: 4
- Initial CW: 7
- Priority resolution: enabled
- Simulation time: 10 seconds

### PLCHighLoad

Increased traffic rate (2 Mbps per node) for stress testing.

### PLCVaryingSNR

Different SNR conditions for each node to test performance under varying channel conditions.

### PLCNoPriorityResolution

Comparison configuration with priority resolution disabled.

### PLCParameterSweep

Parameter sweep for BPC and CW values.

### PLCAnalysis

Comprehensive analysis with detailed statistics collection.

### PLCDebug

Debug configuration with verbose logging.

## Key Metrics Logged

### Throughput per Node

- Frame transmission/reception counts
- Successful delivery rates
- Application-level throughput

### Average Delay

- End-to-end packet delay
- MAC layer queueing delay
- Transmission completion time

### Collision Count

- MAC layer collision detection
- Priority resolution conflicts
- Backoff procedure triggers

## Running the Simulation

```bash
# Basic configuration
opp_run -m -u Cmdenv -c PLCBasic -n .:../../../src --image-path=../../../images omnetpp.ini

# High load scenario
opp_run -m -u Cmdenv -c PLCHighLoad -n .:../../../src --image-path=../../../images omnetpp.ini

# Analysis with parameter sweep
opp_run -m -u Cmdenv -c PLCParameterSweep -n .:../../../src --image-path=../../../images omnetpp.ini
```

## Expected Results

The simulation will demonstrate:

1. **Priority Resolution**: Nodes with higher priority (CA3) should achieve better performance
2. **Backoff Algorithm**: Proper collision avoidance using HomePlug 1.0 backoff
3. **Channel Access**: Fair medium access with appropriate prioritization
4. **Performance Metrics**: Realistic throughput and delay characteristics for PLC networks

## Files

- `omnetpp.ini`: Configuration file with all scenarios
- `PLCNetwork.ned`: Network topology definition (in src/inet/linklayer/plc/)
- `IEEE1901Interface.ned`: PLC interface compound module
- `IEEE1901Mac.cc/.h/.ned`: MAC layer implementation
- `IEEE1901Phy.cc/.h/.ned`: PHY layer implementation
- `PLCFrame.msg`: Frame format definitions

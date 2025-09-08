# IEEE 1901 Power Line Communication (PLC) Implementation

This directory contains a complete implementation of the IEEE 1901 standard-based Power Line Communication (PLC) protocol for the INET Framework.

## Implementation Overview

The IEEE 1901 PLC implementation provides a complete MAC/PHY stack with HomePlug 1.0 compatible priority resolution mechanism and backoff algorithms.

### Key Features

- **IEEE 1901 Standard Compliance**: Implementation of international standard for power line communication
- **HomePlug 1.0 Compatibility**: Interoperability with existing HomePlug equipment
- **Priority-based Medium Access**: Support for CA0-CA3 4-level priority system
- **Realistic PHY Simulation**: SNR-based BER calculation and packet error simulation
- **INET Framework Integration**: Drop-in replacement for EthernetInterface

## Installation and Setup

### Prerequisites

- OMNeT++ 6.0 or later
- INET Framework 4.4 or later
- C++17 compatible compiler

### Building the Module

1. **Add to INET Source Tree**:

   ```bash
   # Navigate to INET source directory
   cd inet/src/inet/linklayer/
   # The plc/ directory should be present with all source files
   ```

2. **Update INET Build Configuration**:
   Add the following to `inet/src/Makefile` or use the INET build system:

   ```makefile
   # Add PLC sources to compilation
   SOURCES += linklayer/plc/IEEE1901Mac.cc \
              linklayer/plc/IEEE1901Phy.cc
   ```

3. **Generate Message Files**:

   ```bash
   # Generate C++ headers from .msg files
   cd inet/src/inet/linklayer/plc/
   opp_msgc PLCFrame.msg
   ```

4. **Rebuild INET**:
   ```bash
   cd inet/
   make clean
   make MODE=release
   ```

### Including in OMNeT++ Projects

1. **Project Configuration**:

   ```ini
   # In .oppbuildspec or IDE project settings
   [General]
   include-path = ../../inet/src
   library-path = ../../inet/src
   libraries = INET
   ```

2. **NED Path Setup**:

   ```ini
   # In omnetpp.ini
   [General]
   ned-path = .;../../inet/src
   image-path = ../../inet/images
   ```

3. **Required Imports**:
   ```ned
   // In your .ned files
   import inet.linklayer.plc.IEEE1901Interface;
   import inet.linklayer.plc.PLCNetwork;
   ```

## File Structure

```
src/inet/linklayer/plc/
├── PLCFrame.msg              # PLC frame definitions
├── PLCFrame_m.h              # Generated from .msg file
├── PLCFrame_m.cc             # Generated from .msg file
├── IEEE1901Mac.h             # MAC layer header
├── IEEE1901Mac.cc            # MAC layer implementation
├── IEEE1901Mac.ned           # MAC layer NED module
├── IEEE1901Phy.h             # PHY layer header
├── IEEE1901Phy.cc            # PHY layer implementation
├── IEEE1901Phy.ned           # PHY layer NED module
├── IEEE1901Interface.ned     # Compound interface module
├── PLCNetwork.ned            # Example network topology
└── README.md                 # This documentation
```

## Module Description

### 1. PLCFrame (PLCFrame.msg)

Defines IEEE 1901 frame formats.

#### Basic Frame Structure

```cpp
class PLCFrame extends FieldsChunk {
    int frameType;       // 0=DATA, 1=BEACON, 2=CONTROL
    int priority;        // CA0=0, CA1=1, CA2=2, CA3=3
    int srcAddr;         // Source node address
    int destAddr;        // Destination node address
    int payloadLength;   // Payload length in bytes
    bool ackRequired;    // ACK required flag
    double txStartTime;  // Transmission start time
    double txEndTime;    // Transmission end time
}
```

#### Specialized Frame Types

- **PLCBeaconFrame**: Beacon frames for network synchronization
- **PLCDataFrame**: Data transmission frames with fragmentation support
- **PLCControlFrame**: Network control and management frames

### 2. IEEE1901Mac (MAC Layer)

Implements core IEEE 1901 MAC functionality.

#### Key Features

- **Priority Resolution Signals (PRS)**: HomePlug 1.0 standard CA0-CA3 priority mechanism
- **Backoff Algorithm**: BPC/DC/BC counter-based collision avoidance
- **Frame Transmission Control**: ACK-based reliable transmission
- **Statistics Collection**: Throughput, delay, collision metrics

#### Priority Resolution Mechanism

```
CA0 (Lowest):  No signal in PRS0/PRS1
CA1:          Signal only in PRS1
CA2:          Signal only in PRS0
CA3 (Highest): Signal in both PRS0/PRS1
```

#### Backoff Algorithm (Based on Jung et al. 2005)

```
CA3/CA2 (High Priority):
  BPC=0 → DC=0, CW=7
  BPC=1 → DC=1, CW=15
  BPC=2 → DC=3, CW=15
  BPC>2 → DC=15, CW=31

CA1/CA0 (Low Priority):
  BPC=0 → DC=0, CW=7
  BPC=1 → DC=1, CW=15
  BPC=2 → DC=3, CW=31
  BPC>2 → DC=15, CW=63
```

### 3. IEEE1901Phy (PHY Layer)

Handles physical layer signal processing and error simulation.

#### Key Features

- **Data Rates**: Configurable 2-14 Mbps
- **BER Calculation**: SNR-based exponential mapping `BER = α × exp(-β × SNR)`
- **Packet Error Simulation**: `PER = 1 - (1 - BER)^(length×8)`
- **Channel Modeling**: Power line characteristics (attenuation, noise)

#### BER Calculation Formula

```cpp
double getBitErrorRate(double snr) {
    double ber = berAlpha * exp(-berBeta * snr);
    return min(ber, maxBER);
}
```

### 4. IEEE1901Interface (Compound Module)

Provides complete PLC interface integrating MAC and PHY.

#### Features

- **Drop-in Replacement**: Can replace EthernetInterface in existing simulations
- **Parameter Exposure**: Key configuration values controllable from parent modules
- **INET Compliance**: Follows standard INET interface contracts

## Usage Instructions

### 1. Basic Usage

Replace Ethernet interface with PLC interface in networks:

```ned
// Traditional Ethernet approach
node: StandardHost {
    *.numEthInterfaces = 1;
}

// Using PLC interface
node: StandardHost {
    *.numEthInterfaces = 0;
    submodules:
        plcInterface: IEEE1901Interface;
    gates:
        plcg;
    connections:
        plcInterface.radioIn <-- plcg;
}
```

### 2. Parameter Configuration

Configure key parameters in omnetpp.ini:

```ini
# PHY Layer Settings
**.plcInterface.dataRate = 10Mbps        # Data transmission rate
**.plcInterface.snr = 25dB               # Signal-to-noise ratio
**.plcInterface.enableBER = true         # Enable BER simulation

# MAC Layer Settings
**.plcInterface.usePriorityResolution = true  # Enable priority resolution
**.plcInterface.maxBPC = 7                    # Maximum backoff procedure counter
**.plcInterface.defaultPriority = 0           # Default priority (CA0-CA3)

# Backoff Timing
**.plcInterface.slotTime = 9us               # Slot time
**.plcInterface.prs0Duration = 35.84us       # PRS0 duration
**.plcInterface.prs1Duration = 35.84us       # PRS1 duration
```

### 3. Network Configuration

Example with 5 nodes connected to shared power line:

```ned
network PLCNetwork {
    submodules:
        node[5]: PLCNode;
        plcChannel: WireJunction;
    connections:
        for i=0..4 {
            node[i].plcg <--> PLCMedium <--> plcChannel.port++;
        }
}
```

### 4. Traffic Generation

Generate constant traffic for performance measurement:

```ini
# 1500-byte frames at 1 Mbps
**.node[*].frameSize = 1500B
**.node[*].trafficRate = 1Mbps

# Assign different priorities
**.node[0].plcInterface.defaultPriority = 3  # CA3 (Highest)
**.node[1].plcInterface.defaultPriority = 2  # CA2
**.node[2].plcInterface.defaultPriority = 1  # CA1
**.node[3].plcInterface.defaultPriority = 0  # CA0 (Lowest)
```

## Essential Configuration Files

### Minimal omnetpp.ini

```ini
[General]
network = inet.linklayer.plc.PLCNetwork
ned-path = .;../../../src
image-path = ../../../images
sim-time-limit = 10s

[Config Basic]
# Essential PLC parameters
**.plcInterface.dataRate = 10Mbps
**.plcInterface.snr = 20dB
**.plcInterface.usePriorityResolution = true
**.plcInterface.maxBPC = 4

# Enable statistics
**.statistic-recording = true
```

### Required NED Imports

```ned
import inet.linklayer.plc.IEEE1901Interface;
import inet.linklayer.plc.PLCNetwork;
import inet.node.base.StandardHostBase;
import inet.physicallayer.wired.common.WireJunction;
```

## Performance Metrics

### Collected Statistics

- **Throughput**: Per-node frame transmission/reception counts
- **Delay**: End-to-end packet delay
- **Collision Count**: MAC layer collisions and backoff occurrences
- **BER/PER**: Bit error rate and packet error rate
- **SNR**: Signal-to-noise ratio variations

### Statistics Activation

```ini
# Enable key statistics collection
**.framesSent.statistic-recording = true
**.framesReceived.statistic-recording = true
**.collisions.statistic-recording = true
**.endToEndDelay.statistic-recording = true
**.ber.statistic-recording = true
**.snr.statistic-recording = true
```

## Running Examples

Execute example network simulations:

```bash
# Basic configuration
cd examples/plc
opp_run -m -u Cmdenv -c PLCBasic -n .:../../../src omnetpp.ini

# High load testing
opp_run -m -u Cmdenv -c PLCHighLoad -n .:../../../src omnetpp.ini

# Parameter sweep execution
opp_run -m -u Cmdenv -c PLCParameterSweep -n .:../../../src omnetpp.ini
```

## Troubleshooting

### Common Issues

1. **Message File Not Found**:

   ```bash
   # Regenerate message files
   cd src/inet/linklayer/plc/
   opp_msgc PLCFrame.msg
   ```

2. **Module Not Found**:

   ```ini
   # Check NED path in omnetpp.ini
   ned-path = .;../../../src
   ```

3. **Compilation Errors**:

   ```bash
   # Clean and rebuild
   make clean
   make MODE=release -j4
   ```

4. **Statistics Not Recorded**:
   ```ini
   # Enable statistics explicitly
   **.statistic-recording = true
   **.vector-recording = true
   ```

## Extension and Development

### Adding New Priority Classes

1. Define new priority values in `PLCFrame.msg`
2. Modify priority resolution logic in `IEEE1901Mac.cc`
3. Update backoff parameter tables

### Adding New PHY Models

1. Modify `getBitErrorRate()` function in `IEEE1901Phy.cc`
2. Add channel modeling parameters
3. Implement new BER curves

### Debugging and Analysis

```ini
# Enable detailed logging
**.cmdenv-log-level = debug
**.plcInterface.**.cmdenv-log-level = trace

# Enable vector recording
**.vector-recording = true
```

## References

- IEEE Std 1901-2010: IEEE Standard for Broadband over Power Line Networks
- Jung et al. (2005): "Performance evaluation of IEEE 802.11 DCF and IEEE 802.11e EDCA in non-saturated conditions"
- HomePlug 1.0 Specification
- INET Framework Documentation

---

# IEEE 1901 전력선 통신(PLC) 구현

본 디렉토리는 INET Framework를 위한 IEEE 1901 표준 기반 전력선 통신(Power Line Communication, PLC) 프로토콜의 완전한 구현을 포함합니다.

## 구현 개요

IEEE 1901 PLC 구현은 HomePlug 1.0 호환 우선순위 해결 메커니즘과 백오프 알고리즘을 포함한 완전한 MAC/PHY 스택을 제공합니다.

### 주요 특징

- **IEEE 1901 표준 준수**: 전력선 통신을 위한 국제 표준 구현
- **HomePlug 1.0 호환성**: 기존 HomePlug 장비와의 상호 운용성
- **우선순위 기반 매체 접근**: CA0-CA3 4단계 우선순위 지원
- **현실적인 PHY 시뮬레이션**: SNR 기반 BER 계산 및 패킷 오류 시뮬레이션
- **INET Framework 통합**: 기존 EthernetInterface의 대체 모듈로 사용 가능

## 설치 및 설정

### 필수 요구사항

- OMNeT++ 6.0 이상
- INET Framework 4.4 이상
- C++17 호환 컴파일러

### 모듈 빌드

1. **INET 소스 트리에 추가**:

   ```bash
   # INET 소스 디렉토리로 이동
   cd inet/src/inet/linklayer/
   # plc/ 디렉토리와 모든 소스 파일이 있어야 함
   ```

2. **INET 빌드 설정 업데이트**:
   `inet/src/Makefile`에 다음을 추가하거나 INET 빌드 시스템 사용:

   ```makefile
   # PLC 소스를 컴파일에 추가
   SOURCES += linklayer/plc/IEEE1901Mac.cc \
              linklayer/plc/IEEE1901Phy.cc
   ```

3. **메시지 파일 생성**:

   ```bash
   # .msg 파일에서 C++ 헤더 생성
   cd inet/src/inet/linklayer/plc/
   opp_msgc PLCFrame.msg
   ```

4. **INET 재빌드**:
   ```bash
   cd inet/
   make clean
   make MODE=release
   ```

### OMNeT++ 프로젝트에 포함

1. **프로젝트 설정**:

   ```ini
   # .oppbuildspec 또는 IDE 프로젝트 설정에서
   [General]
   include-path = ../../inet/src
   library-path = ../../inet/src
   libraries = INET
   ```

2. **NED 경로 설정**:

   ```ini
   # omnetpp.ini에서
   [General]
   ned-path = .;../../inet/src
   image-path = ../../inet/images
   ```

3. **필수 Import**:
   ```ned
   // .ned 파일에서
   import inet.linklayer.plc.IEEE1901Interface;
   import inet.linklayer.plc.PLCNetwork;
   ```

## 파일 구조

```
src/inet/linklayer/plc/
├── PLCFrame.msg              # PLC 프레임 정의
├── PLCFrame_m.h              # .msg 파일에서 생성
├── PLCFrame_m.cc             # .msg 파일에서 생성
├── IEEE1901Mac.h             # MAC 계층 헤더
├── IEEE1901Mac.cc            # MAC 계층 구현
├── IEEE1901Mac.ned           # MAC 계층 NED 모듈
├── IEEE1901Phy.h             # PHY 계층 헤더
├── IEEE1901Phy.cc            # PHY 계층 구현
├── IEEE1901Phy.ned           # PHY 계층 NED 모듈
├── IEEE1901Interface.ned     # 복합 인터페이스 모듈
├── PLCNetwork.ned            # 예제 네트워크 토폴로지
└── README.md                 # 본 문서
```

## 모듈 설명

### 1. PLCFrame (PLCFrame.msg)

IEEE 1901 프레임 형식을 정의합니다.

#### 기본 프레임 구조

```cpp
class PLCFrame extends FieldsChunk {
    int frameType;       // 0=DATA, 1=BEACON, 2=CONTROL
    int priority;        // CA0=0, CA1=1, CA2=2, CA3=3
    int srcAddr;         // 송신 노드 주소
    int destAddr;        // 수신 노드 주소
    int payloadLength;   // 페이로드 길이
    bool ackRequired;    // ACK 필요 여부
    double txStartTime;  // 전송 시작 시간
    double txEndTime;    // 전송 종료 시간
}
```

#### 특화된 프레임 타입

- **PLCBeaconFrame**: 네트워크 동기화용 비콘 프레임
- **PLCDataFrame**: 데이터 전송용 프레임 (분할 전송 지원)
- **PLCControlFrame**: 네트워크 제어용 프레임

### 2. IEEE1901Mac (MAC 계층)

IEEE 1901 MAC 계층의 핵심 기능을 구현합니다.

#### 주요 기능

- **우선순위 해결 신호 (PRS)**: HomePlug 1.0 표준의 CA0-CA3 우선순위 메커니즘
- **백오프 알고리즘**: BPC/DC/BC 카운터 기반 충돌 회피
- **프레임 전송 제어**: ACK 기반 신뢰성 있는 전송
- **통계 수집**: 처리량, 지연시간, 충돌 횟수 등

#### 우선순위 해결 메커니즘

```
CA0 (최저): PRS0/PRS1에서 신호 전송 안함
CA1       : PRS1에서만 신호 전송
CA2       : PRS0에서만 신호 전송
CA3 (최고): PRS0/PRS1 모두에서 신호 전송
```

#### 백오프 알고리즘 (Jung et al. 2005 기반)

```
CA3/CA2 (고우선순위):
  BPC=0 → DC=0, CW=7
  BPC=1 → DC=1, CW=15
  BPC=2 → DC=3, CW=15
  BPC>2 → DC=15, CW=31

CA1/CA0 (저우선순위):
  BPC=0 → DC=0, CW=7
  BPC=1 → DC=1, CW=15
  BPC=2 → DC=3, CW=31
  BPC>2 → DC=15, CW=63
```

### 3. IEEE1901Phy (PHY 계층)

물리 계층 신호 처리 및 오류 시뮬레이션을 담당합니다.

#### 주요 기능

- **데이터 전송률**: 2-14 Mbps 설정 가능
- **BER 계산**: SNR 기반 지수적 매핑 `BER = α × exp(-β × SNR)`
- **패킷 오류 시뮬레이션**: `PER = 1 - (1 - BER)^(길이×8)`
- **채널 모델링**: 전력선 특성 반영 (감쇠, 잡음 등)

#### BER 계산 공식

```cpp
double getBitErrorRate(double snr) {
    double ber = berAlpha * exp(-berBeta * snr);
    return min(ber, maxBER);
}
```

### 4. IEEE1901Interface (복합 모듈)

MAC과 PHY를 통합한 완전한 PLC 인터페이스를 제공합니다.

#### 특징

- **드롭인 대체**: 기존 EthernetInterface 대신 사용 가능
- **매개변수 노출**: 주요 설정값을 상위 모듈에서 제어 가능
- **INET 호환성**: 표준 INET 인터페이스 규약 준수

## 사용 방법

### 1. 기본 사용법

네트워크에서 Ethernet 인터페이스 대신 PLC 인터페이스 사용:

```ned
// 기존 Ethernet 방식
node: StandardHost {
    *.numEthInterfaces = 1;
}

// PLC 방식으로 변경
node: StandardHost {
    *.numEthInterfaces = 0;
    submodules:
        plcInterface: IEEE1901Interface;
    gates:
        plcg;
    connections:
        plcInterface.radioIn <-- plcg;
}
```

### 2. 매개변수 설정

주요 매개변수를 omnetpp.ini에서 설정:

```ini
# PHY 계층 설정
**.plcInterface.dataRate = 10Mbps        # 데이터 전송률
**.plcInterface.snr = 25dB               # 신호 대 잡음비
**.plcInterface.enableBER = true         # BER 시뮬레이션 활성화

# MAC 계층 설정
**.plcInterface.usePriorityResolution = true  # 우선순위 해결 활성화
**.plcInterface.maxBPC = 7                    # 최대 백오프 절차 카운터
**.plcInterface.defaultPriority = 0           # 기본 우선순위 (CA0-CA3)

# 백오프 타이밍
**.plcInterface.slotTime = 9us               # 슬롯 시간
**.plcInterface.prs0Duration = 35.84us       # PRS0 지속시간
**.plcInterface.prs1Duration = 35.84us       # PRS1 지속시간
```

### 3. 네트워크 구성

5개 노드가 공유 전력선에 연결된 예제:

```ned
network PLCNetwork {
    submodules:
        node[5]: PLCNode;
        plcChannel: WireJunction;
    connections:
        for i=0..4 {
            node[i].plcg <--> PLCMedium <--> plcChannel.port++;
        }
}
```

### 4. 트래픽 생성

일정한 트래픽을 생성하여 성능 측정:

```ini
# 1500바이트 프레임을 1Mbps로 전송
**.node[*].frameSize = 1500B
**.node[*].trafficRate = 1Mbps

# 서로 다른 우선순위 할당
**.node[0].plcInterface.defaultPriority = 3  # CA3 (최고)
**.node[1].plcInterface.defaultPriority = 2  # CA2
**.node[2].plcInterface.defaultPriority = 1  # CA1
**.node[3].plcInterface.defaultPriority = 0  # CA0 (최저)
```

## 필수 설정 파일

### 최소 omnetpp.ini

```ini
[General]
network = inet.linklayer.plc.PLCNetwork
ned-path = .;../../../src
image-path = ../../../images
sim-time-limit = 10s

[Config Basic]
# 필수 PLC 매개변수
**.plcInterface.dataRate = 10Mbps
**.plcInterface.snr = 20dB
**.plcInterface.usePriorityResolution = true
**.plcInterface.maxBPC = 4

# 통계 활성화
**.statistic-recording = true
```

### 필수 NED Import

```ned
import inet.linklayer.plc.IEEE1901Interface;
import inet.linklayer.plc.PLCNetwork;
import inet.node.base.StandardHostBase;
import inet.physicallayer.wired.common.WireJunction;
```

## 성능 메트릭

### 수집되는 통계

- **처리량**: 노드별 전송/수신 프레임 수
- **지연시간**: 종단간 패킷 지연
- **충돌 횟수**: MAC 계층 충돌 및 백오프 발생 횟수
- **BER/PER**: 비트 오류율 및 패킷 오류율
- **SNR**: 신호 대 잡음비 변화

### 통계 활성화

```ini
# 주요 통계 수집 활성화
**.framesSent.statistic-recording = true
**.framesReceived.statistic-recording = true
**.collisions.statistic-recording = true
**.endToEndDelay.statistic-recording = true
**.ber.statistic-recording = true
**.snr.statistic-recording = true
```

## 예제 실행

예제 네트워크 시뮬레이션 실행:

```bash
# 기본 설정으로 실행
cd examples/plc
opp_run -m -u Cmdenv -c PLCBasic -n .:../../../src omnetpp.ini

# 고부하 상황 테스트
opp_run -m -u Cmdenv -c PLCHighLoad -n .:../../../src omnetpp.ini

# 매개변수 스위프 실행
opp_run -m -u Cmdenv -c PLCParameterSweep -n .:../../../src omnetpp.ini
```

## 문제 해결

### 일반적인 문제들

1. **메시지 파일을 찾을 수 없음**:

   ```bash
   # 메시지 파일 재생성
   cd src/inet/linklayer/plc/
   opp_msgc PLCFrame.msg
   ```

2. **모듈을 찾을 수 없음**:

   ```ini
   # omnetpp.ini에서 NED 경로 확인
   ned-path = .;../../../src
   ```

3. **컴파일 오류**:

   ```bash
   # 클린 후 재빌드
   make clean
   make MODE=release -j4
   ```

4. **통계가 기록되지 않음**:
   ```ini
   # 통계를 명시적으로 활성화
   **.statistic-recording = true
   **.vector-recording = true
   ```

## 확장 및 개발

### 새로운 우선순위 클래스 추가

1. `PLCFrame.msg`에서 새로운 우선순위 값 정의
2. `IEEE1901Mac.cc`의 우선순위 해결 로직 수정
3. 백오프 매개변수 테이블 업데이트

### 새로운 PHY 모델 추가

1. `IEEE1901Phy.cc`의 `getBitErrorRate()` 함수 수정
2. 채널 모델링 매개변수 추가
3. 새로운 BER 곡선 구현

### 디버깅 및 분석

```ini
# 상세 로깅 활성화
**.cmdenv-log-level = debug
**.plcInterface.**.cmdenv-log-level = trace

# 벡터 기록 활성화
**.vector-recording = true
```

## 참고 문헌

- IEEE Std 1901-2010: IEEE Standard for Broadband over Power Line Networks
- Jung et al. (2005): "Performance evaluation of IEEE 802.11 DCF and IEEE 802.11e EDCA in non-saturated conditions"
- HomePlug 1.0 Specification
- INET Framework Documentation

## 라이선스

이 구현은 INET Framework의 LGPL-3.0-or-later 라이선스를 따릅니다.

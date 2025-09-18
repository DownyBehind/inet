cplusplus {{
}}

namespace inet;

class HpgpDcBase extends cMessage
{
    int priority;           // CA0..3
    int jobId;
    int seq;
    int payloadLength;      // bytes
    int srcAddr;
    int destAddr;
}

class DcRequest extends HpgpDcBase { }
class DcResponse extends HpgpDcBase { }

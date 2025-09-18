cplusplus {{
}}

namespace inet;

class HpgpSlacBase extends cMessage
{
    int priority;           // CA0..3
    int seq;
    int payloadLength;      // bytes
}

class SLACParmReq extends HpgpSlacBase { }
class SLACParmCnf extends HpgpSlacBase { }
class StartAtten extends HpgpSlacBase { int index; }
class MnbcSound extends HpgpSlacBase { int index; }
class AttenCharRsp extends HpgpSlacBase { }
class AttenCharInd extends HpgpSlacBase { }
class SlacMatchReq extends HpgpSlacBase { }
class SlacMatchCnf extends HpgpSlacBase { }

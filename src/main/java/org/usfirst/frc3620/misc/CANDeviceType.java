package org.usfirst.frc3620.misc;

public enum CANDeviceType {
    /* CAN bus ID for a devType|mfg|api|dev.
     * total of 32 bits: 8 bit devType, 8 bit mfg, 10 bit API, 6 bit device id.
     */

    /*
     * PDPs used to be 0x08041400.
     * 2019.02.09: PDPs respond to APIs 0x50 0x51 0x52 0x59 0x5d
     */
    CTRE_PDP(0x08041400, 4),

    REV_PDH(0x08051800, 4),

    /* we always used 0x09041400 for PCMs */
    CTRE_PCM(0x09041400, 4),

    REV_PH(0x09051800, 4),

    /*
     * SRX used to be 0x02041400.

    As of 2019.02.08: (SRX @ devid 1)
     7 0x007 = 020401C1
    81 0x051 = 02041441 ** using this?
    82 0x052 = 02041481
    83 0x053 = 020414C1
    87 0x057 = 020415C1
    91 0x05B = 020416C1
    92 0x05C = 02041701
    93 0x05D = 02041741
    94 0x05E = 02041781

    2020.01.20 Device id is 0x0204 (https://github.com/CrossTheRoadElec/Phoenix-api/blob/master/src/main/java/com/ctre/phoenix/motorcontrol/can/TalonSRX.java)

    Talon FX and SRX are the same.
    */
    TALON_PHOENIX5(0x02041441, 64),
    TALON_PHOENIX6(0x02044855, 64),

    /*
    SPX used to be 0x01041400.

    As of 2019.02.08:  (SPX @ devid 2)
     7 0x007 = 010401C2
    81 0x051 = 01041442 ** using this
    83 0x053 = 010414C2
    91 0x05B = 010416C2
    92 0x05C = 01041702
    93 0x05D = 01041742
    94 0x05E = 01041782

    2020.01.20 Device id is 0x0104 (https://github.com/CrossTheRoadElec/Phoenix-api/blob/master/src/main/java/com/ctre/phoenix/motorcontrol/can/VictorSPX.java)
    */
    VICTOR_SPX(0x01041442, 64),

    // per REV (x02051800)
    SPARK_MAX(0x02051800, 64);

    int msgId, maxDevices;

    CANDeviceType(int _msgId, int _maxDevices) {
        msgId = _msgId;
        maxDevices = _maxDevices;
    }

    public int getMsgId() {
        return msgId;
    }

    public int getMaxDevices() {
        return maxDevices;
    }
}
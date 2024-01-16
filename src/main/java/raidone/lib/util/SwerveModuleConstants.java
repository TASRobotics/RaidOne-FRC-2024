package raidone.lib.util;

import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveModuleConstants {
    public final String MODULE_NUMBER;
    public final int THROTTLE_ID;
    public final int ROTOR_ID;
    public final int CAN_CODER_ID;
    public final Rotation2d ANGLE_OFFSET;

    /**
     * 
     * @param throttleID
     * @param rotorID
     * @param canCoderID
     * @param angleOffset
     */
    public SwerveModuleConstants(String moduleNum, int throttleID, int rotorID, int canCoderID, Rotation2d angleOffset) {
        this.MODULE_NUMBER = moduleNum;
        this.THROTTLE_ID = throttleID;
        this.ROTOR_ID = rotorID;
        this.CAN_CODER_ID = canCoderID;
        this.ANGLE_OFFSET = angleOffset;
    }
}

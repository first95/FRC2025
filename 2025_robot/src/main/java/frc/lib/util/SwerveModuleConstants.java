package frc.lib.util;

public class SwerveModuleConstants {
    public final int moduleNumber, driveMotorID, angleMotorID;
    public final double angleOffset, xPos, yPos;

    public SwerveModuleConstants(int moduleNumber, int driveMotorID, int angleMotorID, double angleOffset, double xPos, double yPos) {
        this.moduleNumber = moduleNumber;
        this.driveMotorID = driveMotorID;
        this.angleMotorID = angleMotorID;
        this.angleOffset = angleOffset;
        this.xPos = xPos;
        this.yPos = yPos;
    }
}

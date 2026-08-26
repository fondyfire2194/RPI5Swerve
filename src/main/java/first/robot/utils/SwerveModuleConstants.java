package first.robot.utils;

import org.wpilib.math.geometry.Rotation2d;

public class SwerveModuleConstants {
  public final int driveCanBusNum;
  public final int angleCanBusNum;
  public final int moduleNumber;
  public final int driveMotorID;
  public final int angleMotorID;
  public final int canCoderID;
  public final Rotation2d angleOffset;
  public final boolean driveReversed;

  /**
   * Swerve Module Constants to be used when creating swerve modules.
   * 
   * @param driveCanBusNum
   * @param angleCanBusNum
   * @param modulenumber
   * @param driveMotorID
   * @param angleMotorID
   * @param canCoderID
   * @param angleOffset
   * @param driveReversed
   */
  public SwerveModuleConstants(
      int driveCanBusNum, int angleCanBusNum, int moduleNumber,
      int driveMotorID, int angleMotorID, int cancoderID, Rotation2d angleOffset, boolean driveReversed) {
    this.driveCanBusNum = driveCanBusNum;
    this.angleCanBusNum = angleCanBusNum;
    this.moduleNumber = moduleNumber;
    this.driveMotorID = driveMotorID;
    this.angleMotorID = angleMotorID;
    this.canCoderID = cancoderID;
    this.angleOffset = angleOffset;
    this.driveReversed = driveReversed;
  }
}
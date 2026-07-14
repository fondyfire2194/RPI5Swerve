
package first.robot.utils;

import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Meters;

import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.SwerveDriveKinematics;
import org.wpilib.math.util.Units;
import org.wpilib.units.measure.Distance;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

/**
 * Field setup important dimensions
 * 
 * *******************WIDTH******************************
 * 
 * Width 26ft 11-1/4 inches - half width 13ft 5-5/8 inches = 161-5/8 inches
 * 
 * Speaker center is offset 57 inches = 4ft 9 inches from center line
 * 
 * Speaker center from source side is 18ft 2-5/8 inches or 218-5/8 inches
 * 
 * Speaker center from amp side is 8ft 8-5/8 inches or 104-5/8 inches
 * 
 * Math check 18ft 2-5/8 + 8ft 8-5/8 = 26ft 11-1/4 = field width
 * 
 * 
 * *******************LENGTH******************************
 * 
 * Length 54ft 3-1/4 inches - center line is 27ft 1-5/8 inches = 325-5/8 inches
 * 
 * Start line is 6ft 4-1/8 inches from speaker face = 76-1/8 inches
 * 
 * 
 * *******************NOTES******************************
 * https://www.firstinspires.org/robotics/frc/playing-field
 * 
 * The 3 start line notes are centered on speaker and 4ft 9 inches = 57 inches
 * either side of that 109.5" from speaker face
 * 
 * Field center line notes are 27ft 1-5/8 inches = 325-5/8 inches from speaker
 * face
 * 
 * One is on the field center line, others are 5ft 6 inches = 66 inches apart
 * from there
 *
 * Outside notes are 2ft 5-5/8 inches = 29-5/8 inches from field edge
 * 
 * 
 * 
 * Math check 4 x 5ft 6 = 22 ft + 2 x 2ft 5-5/8 = 4ft 11-1/4 = 26ft 11-1/4 =
 * field width
 * 
 * Stage layout
 * 
 * Stage center pillar plate begins 120.5" from speaker face and is on field
 * widthcenter line
 * Stage right and laft pillar plates field edge centers are 61-3/8 inches
 * either
 * side of field width center line and are 231.2 inches from speaker face.
 * 
 * 
 * 
 * 
 * 
 * 
 */

public final class Constants {

        public static final class CANIDConstants {
                // can ids 4 through 15 are used for swerve modules see SwerveConstants
                public static final int pdpID = 1;

        }

        public static final class SwerveConstants {

                public static final double stickDeadband = 0.05;

                public static final boolean invertGyro = true; // Always ensure Gyro is CCW+ CW-

                public static final Distance robotWidthWithBumpers = Meters
                                .of(Meters.convertFrom(36, Inches));
                public static final Distance robotLengthWithBumpers = Meters
                                .of(Meters.convertFrom(32, Inches));

                /* Drivetrain Constants */
                public static final Distance trackWidth = Meters.of(Meters.convertFrom(19.5, Inches));// 22.125
                public static final Distance wheelBase = Meters.of(Meters.convertFrom(17.625, Inches));// 27.25
                public static final Distance wheelDiameter = Meters.of(Meters.convertFrom(3.9, Inches));// 3.86
                public static final Distance wheelCircumference = Meters
                                .of(wheelDiameter.magnitude() * Math.PI);

                public static final double openLoopRamp = 0.25;
                public static final double closedLoopRamp = 0.0;

                public static double mk4iL1DriveGearRatio = 1 / ((14.0 / 50.0) * (25.0 / 19.0) * (15.0 / 45.0));// 8.14.122807

                public static double mk4iL2DriveGearRatio = 1 / ((14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0));// 6.75

                public static double mk4iL1TurnGearRatio = 1 / ((14.0 / 50.0) * (10.0 / 60.0));// 21.43 1/.046667

                public static double mk4iL2TurnGearRatio = 1 / ((14.0 / 50.0) * (10.0 / 60.0));// 21.43 1/.046667

                public static double driveGearRatio = mk4iL2DriveGearRatio;

                public static double angleGearRatio = mk4iL2TurnGearRatio;

                public static final Translation2d flModuleOffset = new Translation2d(wheelBase.magnitude() / 2.0,
                                trackWidth.magnitude() / 2.0);
                public static final Translation2d frModuleOffset = new Translation2d(wheelBase.magnitude() / 2.0,
                                -trackWidth.magnitude() / 2.0);
                public static final Translation2d blModuleOffset = new Translation2d(-wheelBase.magnitude() / 2.0,
                                trackWidth.magnitude() / 2.0);
                public static final Translation2d brModuleOffset = new Translation2d(-wheelBase.magnitude() / 2.0,
                                -trackWidth.magnitude() / 2.0);

                public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
                                flModuleOffset, frModuleOffset, blModuleOffset, brModuleOffset);

                /* Swerve Voltage Compensation */
                public static final double voltageComp = 12.0;

                /* Swerve Current Limiting */
                public static final int angleContinuousCurrentLimit = 30;
                public static final int driveContinuousCurrentLimit = 40; // 60

                /* Swerve Profiling Values */
                public static final double kmaxTheoreticalSpeed = 4.6; // 3.7;// mps *1.2 L2
                public static final double kmaxSpeed = 4.0; // meters per second *1.2 L2 3.9
                public static final double maxAngularVelocity = 1.0 * Math.PI;

                public static final double maxTranslationalSpeed = Units.feetToMeters(11.5);

                /* Angle Motor PID Values */
                public static final double[] angleKP = { .1, .1, .1, .1 };
                public static final double[] angleKI = { 0.0, 0.0, 0.0, 0.0 };
                public static final double[] angleKD = { 0, 0, 0, 0 };
                public static final double[] angleKFF = { 0, 0, 0, 0 };

                /* Drive Motor PID Values */
                public static final double driveKP = 0.1;
                public static final double driveKI = 0.0;
                public static final double driveKD = 0.0;
                public static final double driveKFF = .95 / kmaxTheoreticalSpeed;

                /*
                 * Drive Motor Feedforward and PP Values
                 * FIELD CARPET
                 */
                public static final double[] driveKS = { 0.2, 0.2, 0.2, 0.2 };
                public static final double[] driveKV = { 2.5, 2.5, 2.5, 2.5 };
                // 2.56/2.38,2.8.2.2,2.1 recorded speeds
                public static final double[] driveKA = { 0.59, 0.59, 0.59, 0.59 };
                public static final double[] driveKP1 = { 0.5, 0.5, 0.5, 0.5 };

                /*
                 * Drive Motor Feedforward and PP Values
                 * SCHOOL LIBRARY CARPET
                 * 
                 * public static final double driveKS = 0.60;//
                 * public static final double driveKV = 2.70;//
                 * public static final double driveKA = 0.59;//
                 * public static final double driveKP1 = 0.01;//
                 * 
                 * public static PIDConstants PPTransConstants = new PIDConstants(.5, 0, 0); //
                 * 2.0 Translation constants 3
                 * public static PIDConstants PPRotConstants = new PIDConstants(.5, 0, 0); //
                 * 2.0 Translation constants 3
                 * ^/
                 * // team 5907 driveKs = 0.22542;driveKv = 2.4829; driveKa = 0.120; driveP =
                 * // 0.08;
                 * 
                 * /* Drive Motor Conversion Factors
                 */
                public static final double driveConversionPositionFactor = (wheelDiameter.magnitude() * Math.PI)
                                / driveGearRatio;
                public static final double driveConversionVelocityFactor = driveConversionPositionFactor / 60.0;

                public static final double angleConversionFactor = (2 * Math.PI) / angleGearRatio;

                /* Neutral Modes */
                public static final IdleMode angleNeutralMode = IdleMode.kBrake;
                public static final IdleMode driveNeutralMode = IdleMode.kBrake;

                /* Motor Inverts */
                public static final boolean driveInvert = false;
                public static final boolean angleInvert = true;

                /* Angle Encoder Invert */
                public static final boolean canCoderInvert = false;

                public static String[] modNames = { "FL ", "FR ", "BL ", "BR " };

                /* Module Specific Constants */
                /* Front Left Module - Module 0 */
                public static final class Mod0 {

                        public static final int driveMotorID = 13;
                        public static final int angleMotorID = 14;
                        public static final int cancoderID = 15;

                        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0);// 253
                        public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID,
                                        angleMotorID,
                                        cancoderID, angleOffset, false);
                }

                /* Front Right Module - Module 1 */
                public static final class Mod1 {
                        public static final int driveMotorID = 10;
                        public static final int angleMotorID = 11;
                        public static final int cancoderID = 12;

                        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0);// 108
                        public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID,
                                        angleMotorID,
                                        cancoderID, angleOffset, true);
                }

                /* Back Left Module - Module 2 */
                public static final class Mod2 {

                        public static final int driveMotorID = 7;
                        public static final int angleMotorID = 8;
                        public static final int cancoderID = 9;
                        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0);// 207
                        public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID,
                                        angleMotorID,
                                        cancoderID, angleOffset, false);
                }

                /* Back Right Module - Module 3 */
                public static final class Mod3 {
                        public static final int driveMotorID = 4;
                        public static final int angleMotorID = 5;
                        public static final int cancoderID = 6;

                        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0);// 239
                        public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID,
                                        angleMotorID,
                                        cancoderID, angleOffset, true);
                }

                public static double alignKp = .02;
                public static double alighKd = 0;

                public static double maxTranslationalAcceleration;

                public static double turnToAngleMaxVelocity;

                public static double debounceTime;

                public static double alignNoteKp = .02;

                public static double alignNoteKd = 0;

                public static double odometryUpdateFrequency = 100;

                public static double notePickupSpeed = 0.75;

                public static double wheelRadius = Units.inchesToMeters(4) / 2;

                public static double minLobDistance = Units.feetToMeters(33);// 9 meters

                public static double maxLobDistance = Units.feetToMeters(43);// 13

                public static double rangeLobDistance = maxLobDistance - minLobDistance;

                public static double maxMovingShotDistance = Units.feetToMeters(30);

        }

        public static final class GlobalConstants {
                public static final int ROBOT_LOOP_HZ = 50;
                /** Robot loop period */
                public static final double ROBOT_LOOP_PERIOD = 1.0 / ROBOT_LOOP_HZ;
        }

}
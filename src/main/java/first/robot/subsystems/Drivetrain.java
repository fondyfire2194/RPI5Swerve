// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot.subsystems;

import org.wpilib.command3.Mechanism;
import org.wpilib.hardware.hal.CANBusMap;
import org.wpilib.math.estimator.SwerveDrivePoseEstimator;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.kinematics.SwerveDriveKinematics;
import org.wpilib.math.kinematics.SwerveDriveOdometry;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.kinematics.SwerveModuleVelocity;
import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.math.util.Units;
import org.wpilib.networktables.NetworkTable;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.networktables.StructArrayPublisher;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.Pigeon2;

import first.robot.utils.Constants.SwerveConstants;
import first.robot.utils.SD;

/** Represents a swerve drive style drivetrain. */
public class Drivetrain extends Mechanism {
        public static final double kMaxVelocity = 3.0; // 3 meters per second
        public static final double kMaxAngularVelocity = Math.PI; // 1/2 rotation per second

        private static final double xOffset = .381;
        private static final double yOffset = .381;

        private final Translation2d frontLeftLocation = new Translation2d(xOffset, yOffset);
        private final Translation2d frontRightLocation = new Translation2d(xOffset, -yOffset);
        private final Translation2d backLeftLocation = new Translation2d(-xOffset, yOffset);
        private final Translation2d backRightLocation = new Translation2d(-xOffset, -yOffset);

        public final SwerveModule frontLeft = new SwerveModule( SwerveConstants.Mod0.constants);
        public final SwerveModule frontRight = new SwerveModule( SwerveConstants.Mod1.constants);
        public final SwerveModule backLeft = new SwerveModule( SwerveConstants.Mod2.constants);
        public final SwerveModule backRight = new SwerveModule( SwerveConstants.Mod3.constants);

         private final Pigeon2 imu = new Pigeon2(50, CANBus.systemcore(CANBusMap.CAN_S1));

       // private final Pigeon2 imu = new Pigeon2(50, new CANBus("CV1"));

        private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
                        frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);

        private final SwerveDriveOdometry odometry;

        private double lastRot;

        private final NetworkTableInstance inst = NetworkTableInstance.getDefault();

        private final NetworkTable driveStateTable = inst.getTable("DriveState");
        /** The current module states */
        public SwerveModuleVelocity[] ModuleVelocities = new SwerveModuleVelocity[4];
        /** The current module states */

        /** The target module states */
        public SwerveModuleVelocity[] ModuleTargets = new SwerveModuleVelocity[4];
        /** The current module positions */
        public SwerveModulePosition[] ModulePositions = new SwerveModulePosition[4];

        private final StructArrayPublisher<SwerveModuleVelocity> driveModuleVelocities = driveStateTable
                        .getStructArrayTopic("ModuleStates", SwerveModuleVelocity.struct).publish();
        private final StructArrayPublisher<SwerveModuleVelocity> driveModuleTargets = driveStateTable
                        .getStructArrayTopic("ModuleTargets", SwerveModuleVelocity.struct).publish();
        private final StructArrayPublisher<SwerveModulePosition> driveModulePositions = driveStateTable
                        .getStructArrayTopic("ModulePositions", SwerveModulePosition.struct).publish();

        /*
         * Here we use SwerveDrivePoseEstimator so that we can fuse odometry readings.
         * The numbers used
         * below are robot specific, and should be tuned.
         */
        private final SwerveDrivePoseEstimator poseEstimator;

        public Drivetrain() {

                poseEstimator = new SwerveDrivePoseEstimator(
                                kinematics,
                                imu.getRotation2d(),
                                new SwerveModulePosition[] {
                                                frontLeft.getModulePosition(),
                                                frontRight.getModulePosition(),
                                                backLeft.getModulePosition(),
                                                backRight.getModulePosition()
                                },
                                Pose2d.kZero,
                                VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
                                VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));

                odometry = new SwerveDriveOdometry(
                                kinematics,
                                imu.getRotation2d(),
                                new SwerveModulePosition[] {
                                                frontLeft.getModulePosition(),
                                                frontRight.getModulePosition(),
                                                backLeft.getModulePosition(),
                                                backRight.getModulePosition()
                                });

                frontLeft.setCancoderStartOffset();
                frontRight.setCancoderStartOffset();
                backLeft.setCancoderStartOffset();
                backRight.setCancoderStartOffset();

        }

        /**
         * Method to drive the robot using joystick info.
         *
         * @param xVelocity     Velocity of the robot in the x direction (forward).
         * @param yVelocity     Velocity of the robot in the y direction (sideways).
         * @param rot           Angular rate of the robot.
         * @param fieldRelative Whether the x and y velocities are relative to the
         *                      field.
         * @param isOpenLoop    drive is open loop speed
         * @param period        update loop time
         */
        public void drive(
                        double xVelocity, double yVelocity, double rot, boolean fieldRelative, boolean isOpenLoop,
                        double period) {

                var chassisVelocities = new ChassisVelocities(xVelocity, yVelocity, rot);
                if (fieldRelative) {
                        chassisVelocities = chassisVelocities.toRobotRelative(imu.getRotation2d());
                }
                chassisVelocities = chassisVelocities.discretize(period);

                var velocities = SwerveDriveKinematics.desaturateWheelVelocities(
                                kinematics.toWheelVelocities(chassisVelocities), kMaxVelocity);

                frontLeft.setDesiredVelocity(velocities[0], isOpenLoop);
                frontRight.setDesiredVelocity(velocities[1], isOpenLoop);
                backLeft.setDesiredVelocity(velocities[2], isOpenLoop);
                backRight.setDesiredVelocity(velocities[3], isOpenLoop);

        }

        /** Updates the field relative position of the robot. */
        /** Updates the field relative position of the robot. */
        public void updateOdometry() {
                poseEstimator.update(
                                imu.getRotation2d(),
                                new SwerveModulePosition[] {
                                                frontLeft.getModulePosition(),
                                                frontRight.getModulePosition(),
                                                backLeft.getModulePosition(),
                                                backRight.getModulePosition()
                                });

                // Also apply vision measurements. We use 0.3 seconds in the past as an example
                // -- on
                // a real robot, this must be calculated based either on latency or timestamps.
                // poseEstimator.addVisionMeasurement(
                // ExampleGlobalMeasurementSensor.getEstimatedGlobalPose(poseEstimator.getEstimatedPosition()),
                // Timer.getTimestamp() - 0.3);
                // }

        }

        public void periodic() {

                ModuleVelocities[0] = frontLeft.getVelocity();
                ModuleVelocities[1] = frontRight.getVelocity();
                ModuleVelocities[2] = backLeft.getVelocity();
                ModuleVelocities[3] = backRight.getVelocity();

                driveModuleVelocities.set(ModuleVelocities);
                driveModuleTargets.set(ModuleTargets);
                driveModulePositions.set(ModulePositions);

                SD.sd2("PigeonYaw", imu.getYaw().getValueAsDouble());
                SD.sd2("PigeonPitch", imu.getPitch().getValueAsDouble());
                SD.sd2("PigeonRoll", imu.getRoll().getValueAsDouble());

        }
}

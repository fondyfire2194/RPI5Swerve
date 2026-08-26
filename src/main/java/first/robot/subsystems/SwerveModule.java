// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot.subsystems;

import org.wpilib.command3.Mechanism;
import org.wpilib.math.controller.SimpleMotorFeedforward;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.kinematics.SwerveModuleVelocity;
import org.wpilib.math.util.MathUtil;
import org.wpilib.math.util.Units;
import org.wpilib.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import first.robot.utils.Constants;
import first.robot.utils.Constants.SwerveConstants;
import first.robot.utils.SD;
import first.robot.utils.SwerveModuleConstants;


public class SwerveModule extends Mechanism {

        private final SparkMax driveMotor;
        private final SparkMax angleMotor;

        private final RelativeEncoder driveEncoder;
        final RelativeEncoder angleEncoder;

        private final SparkClosedLoopController driveController;

        private final SparkClosedLoopController anglePIDController;

        public final CANcoder angleCancoder;

        private double cancoderStartOffset;

        private final int moduleNumber;

        private final int driveCanBusNum;

        private final int angleCanBusNum;
        
        private final int angleMotorID;
        private final int driveMotorID;

        private final int canCoderID;

        private Rotation2d lastAngle = new Rotation2d();

        private SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(1, 3);

        private boolean driveReversed;

        public static final SparkMaxConfig driveConfig = new SparkMaxConfig();
        public static final SparkMaxConfig angleConfig = new SparkMaxConfig();
        // Set the distance per pulse for the drive encoder. We can simply use the
        // distance traveled for one rotation of the wheel divided by the encoder
        // resolution.

        public SwerveModule(SwerveModuleConstants moduleConstants) {
                driveCanBusNum = moduleConstants.driveCanBusNum;                
                angleCanBusNum = moduleConstants.angleCanBusNum;
                moduleNumber = moduleConstants.moduleNumber;
                driveReversed = moduleConstants.driveReversed;
                angleMotorID = moduleConstants.angleMotorID;
                driveMotorID = moduleConstants.driveMotorID;
                canCoderID = moduleConstants.canCoderID;

                /* Angle Motor Config */
                angleMotor = new SparkMax(angleCanBusNum, angleMotorID,
                                MotorType.kBrushless);
                angleEncoder = angleMotor.getEncoder();

                anglePIDController = angleMotor.getClosedLoopController();

                angleMotor.configure(
                                angleConfig,
                                ResetMode.kResetSafeParameters,
                                PersistMode.kPersistParameters);

                angleCancoder = new CANcoder(canCoderID, new CANBus("CV1"));
                /* Configure CANcoder */
                var toApply = new CANcoderConfiguration();

                /*
                 * User can change the configs if they want, or leave it empty for
                 * factory-default
                 */
                angleCancoder.getConfigurator().apply(toApply);

                /* Speed up signals to an appropriate rate */
                // BaseStatusSignal.setUpdateFrequencyForAll(100, angleCancoder.getPosition(),
                // angleCancoder.getVelocity());

                /* Drive Motor Config */
                driveMotor = new SparkMax(driveCanBusNum, driveMotorID,
                                MotorType.kBrushless);

                driveEncoder = driveMotor.getEncoder();
                driveController = driveMotor.getClosedLoopController();

                {
                        // // Configure basic settings of the intake motor
                        driveConfig
                                        .inverted(driveReversed)
                                        .idleMode(IdleMode.kCoast)
                                        .openLoopRampRate(0.5)
                                        .closedLoopRampRate(.25)
                                        .smartCurrentLimit(50);
                        driveConfig.encoder
                                        .positionConversionFactor(SwerveConstants.driveConversionPositionFactor)
                                        .velocityConversionFactor(SwerveConstants.driveConversionVelocityFactor);

                        driveConfig.closedLoop
                                        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                                        // // Set PID values for speed control. We don't need to pass a closed loop
                                        // // slot, as it will default to slot 0.
                                        .p(0.00005)
                                        .i(0)
                                        .d(0)
                                        .outputRange(-1, 1)
                                        .p(0.072, ClosedLoopSlot.kSlot1)
                                        .i(0, ClosedLoopSlot.kSlot1)
                                        .d(0, ClosedLoopSlot.kSlot1)
                                        // .positionWrappingInputRange(-Math.PI, Math.PI)
                                        .outputRange(-.5, .5, ClosedLoopSlot.kSlot1).feedForward
                                        // // kV is now in Volts, so we multiply by the nominal voltage (12V)
                                        .kV(12.0 / 5767, ClosedLoopSlot.kSlot0);

                }

                {
                        // // Configure basic settings of the intake motor
                        angleConfig
                                        .inverted(false)
                                        .idleMode(IdleMode.kCoast)
                                        .openLoopRampRate(0.5)
                                        .closedLoopRampRate(.25)
                                        .smartCurrentLimit(50);
                        angleConfig.encoder
                                        .positionConversionFactor(SwerveConstants.angleConversionFactor)
                                        .velocityConversionFactor(SwerveConstants.angleConversionFactor / 60);

                        angleConfig.closedLoop
                                        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                                        // // Set PID values for speed control. We don't need to pass a closed loop
                                        // // slot, as it will default to slot 0.
                                        .p(0.00005)
                                        .i(0)
                                        .d(0)
                                        .outputRange(-1, 1)
                                        .p(.15, ClosedLoopSlot.kSlot1)
                                        .i(0, ClosedLoopSlot.kSlot1)
                                        .d(0, ClosedLoopSlot.kSlot1)
                                        .positionWrappingInputRange(-Math.PI, Math.PI)
                                        .positionWrappingEnabled(true)
                                        .outputRange(-.25, .25, ClosedLoopSlot.kSlot1)

                                                        .feedForward
                                        // // kV is now in Volts, so we multiply by the nominal voltage (12V)
                                        .kV(12.0 / 5767, ClosedLoopSlot.kSlot0);

                        angleEncoder.setPosition(0);

                }

                driveFeedforward = new SimpleMotorFeedforward(
                                SwerveConstants.driveKS[moduleNumber],
                                SwerveConstants.driveKV[moduleNumber],
                                SwerveConstants.driveKA[moduleNumber]);

                driveMotor.configure(
                                driveConfig,
                                ResetMode.kResetSafeParameters,
                                PersistMode.kPersistParameters);

                angleMotor.configure(
                                angleConfig,
                                ResetMode.kResetSafeParameters,
                                PersistMode.kPersistParameters);

                angleEncoder.setPosition(0);

        }

        public void setCancoderStartOffset() {
                cancoderStartOffset = angleCancoder.getPosition().getValueAsDouble();
                angleEncoder.setPosition(cancoderStartOffset * Math.PI / 2.);
        }

        /**
         * Returns the current position of the module.
         *
         * @return The current position of the module.
         */
        public SwerveModulePosition getModulePosition() {
                return new SwerveModulePosition(
                                driveEncoder.getPosition().get(), getAngle());
        }

        /**
         * Returns the current velocity of the module.
         *
         * @return The current velocity of the module.
         */
        public SwerveModuleVelocity getVelocity() {
                return new SwerveModuleVelocity(
                                driveEncoder.getVelocity().get(),
                                getAngle());
        }

        /**
         * Sets the desired velocity for the module.
         *
         * @param desiredVelocity Desired velocity and angle.
         * @param isOpenLoop      Drive open loop speed
         */
        public void setDesiredVelocity(SwerveModuleVelocity desiredVelocity, boolean isOpenloop) {

                // var encoderRotation = new Rotation2d(angleEncoder.getPosition().get());
                var encoderRotation = new Rotation2d(
                                MathUtil.inputModulus(angleEncoder.getPosition().get(), -Math.PI, Math.PI));

                // Optimize the desired velocity to avoid spinning further than 90 degrees, then
                // scale velocity
                // by cosine of angle error. This scales down movement perpendicular to the
                // desired direction of
                // travel that can occur when modules change directions. This results in
                // smoother driving.
                SwerveModuleVelocity optvelocity = desiredVelocity.optimize(encoderRotation)
                                .cosineScale(encoderRotation);

                SD.sd2("OptVel", optvelocity.velocity);
                SD.sd2("OptAngleTgt", optvelocity.angle.getDegrees());
                SmartDashboard.putBoolean("Openloop", isOpenloop);
                // Calculate the drive output from the drive PID controller and feedforward.

                if (isOpenloop)
                        driveMotor.setThrottle(optvelocity.velocity / SwerveConstants.kmaxTheoreticalSpeed);

                else {

                        double driveff = driveFeedforward.calculate(optvelocity.velocity);

                        driveController.setSetpoint(optvelocity.velocity, ControlType.kVelocity,
                                        ClosedLoopSlot.kSlot0, driveff, ArbFFUnits.kVoltage);
                }

                double angleTarget = Math.abs(desiredVelocity.velocity) <= (Constants.SwerveConstants.kmaxSpeed * 0.001)
                                ? lastAngle.getRadians()
                                : desiredVelocity.angle.getRadians();

                lastAngle = Rotation2d.fromRadians(angleTarget);

                anglePIDController.setSetpoint(angleTarget, ControlType.kPosition, ClosedLoopSlot.kSlot1);

        }

        private Rotation2d getAngle() {
                return new Rotation2d(MathUtil.inputModulus(angleEncoder.getPosition().get(),
                                -Math.PI, Math.PI));
        }

        public void moduleTelemtry() {

                String modulePrefix = SwerveConstants.modNames[moduleNumber];

                SD.sd2(modulePrefix + " Drive Position Inches",
                                Units.metersToInches(driveEncoder.getPosition().get()));
                SD.sd2(modulePrefix + " TgtDrvVel",
                                Units.metersToInches(getVelocity().velocity));
                SD.sd2(modulePrefix + " ActDrvVel",
                                Units.metersToInches(driveEncoder.getVelocity().get()));
                SD.sd2(modulePrefix + " Drive Throttle",
                                driveMotor.getThrottle());
                SD.sd2(modulePrefix + "Act Angle",
                                getAngle().getDegrees());
                SD.sd2(modulePrefix + " Tgt Angle",
                                getVelocity().angle.getDegrees());

                SD.sd2(modulePrefix + " Angle Throttle",
                                angleMotor.getThrottle());

                SD.sd2(modulePrefix + " CanCoderStartOffset",
                                cancoderStartOffset);

                SD.sd3(modulePrefix + " CanCoderPosition",
                                2 * Math.PI * angleCancoder.getAbsolutePosition().getValueAsDouble());

                SD.sd3("ACF", SwerveConstants.angleConversionFactor);
        }
}

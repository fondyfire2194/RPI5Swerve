// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot;

import org.wpilib.command3.Scheduler;
import org.wpilib.command3.button.CommandGamepad;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.SwerveModuleVelocity;
import org.wpilib.math.util.Units;
import org.wpilib.opmode.PeriodicOpMode;
import org.wpilib.opmode.Utility;
import org.wpilib.smartdashboard.SmartDashboard;

@Utility(name = "Front Left Module Test", group = "Group 1")
public class FrontLeftModuleTest extends PeriodicOpMode {
  private final Robot robot;
  SwerveModuleVelocity velocity;
  public CommandGamepad controller = new CommandGamepad(0);

  /** The Robot instance is passed into the opmode via the constructor. */
  public FrontLeftModuleTest(Robot robot) {
    this.robot = robot;
  }

  @Override
  public void disabledPeriodic() {
    /* Called periodically (on every DS packet) while the robot is disabled. */
    Scheduler.getDefault().run();
    SmartDashboard.putNumber("CLX", controller.getLeftX() * Math.PI);

    velocity = new SwerveModuleVelocity(controller.getLeftY(), new Rotation2d(controller.getLeftX() * Math.PI));

    SmartDashboard.putNumber("CLXRads", controller.getLeftX() * Math.PI);

    SmartDashboard.putNumber("AngleTgt", velocity.angle.getDegrees());

    robot.modulefl.moduleTelemtry();
  }

  @Override
  public void start() {
    /* Called once when the robot is enabled. */

  }

  @Override
  public void periodic() {
    Scheduler.getDefault().run();

    // velocity = new SwerveModuleVelocity(controller.getLeftY(), new
    // Rotation2d(controller.getLeftX() * Math.PI));

    SmartDashboard.putNumber("CLX", controller.getLeftX() * Math.PI);

    SmartDashboard.putNumber("AngleTgt", velocity.angle.getDegrees());

    if (robot.driverController.northFace().getAsBoolean()) {
      velocity = new SwerveModuleVelocity(-controller.getLeftY(), new Rotation2d(Units.degreesToRadians(45)));
    }
    if (robot.driverController.southFace().getAsBoolean()) {
      velocity = new SwerveModuleVelocity(-controller.getLeftY(), new Rotation2d(Units.degreesToRadians(-45)));
    }
    if (robot.driverController.eastFace().getAsBoolean()) {
      velocity = new SwerveModuleVelocity(-controller.getLeftY(), new Rotation2d(Units.degreesToRadians(90)));
    }
    if (robot.driverController.westFace().getAsBoolean()) {
      velocity = new SwerveModuleVelocity(-controller.getLeftY(), new Rotation2d(Units.degreesToRadians(-90)));
    }
    if (robot.driverController.leftBumper().getAsBoolean()) {
      velocity = new SwerveModuleVelocity(-controller.getLeftY(), new Rotation2d(Units.degreesToRadians(170)));
    }

    if (robot.driverController.rightBumper().getAsBoolean()) {
      velocity = new SwerveModuleVelocity(-controller.getLeftY(), new Rotation2d(Units.degreesToRadians(-170)));
    }

    robot.modulefl.setDesiredVelocity(velocity, true);

    robot.modulefl.moduleTelemtry();

    /*
     * Called periodically
     * 
     * 
     * (set time interval) while the robot is enabled.
     */
  }

  @Override
  public void end() {
    /* Called when the robot is disabled (after previously being enabled). */

    robot.close();
    Scheduler.getDefault().cancelAll();
  }

  @Override
  public void close() {
    robot.close();

    Scheduler.getDefault().cancelAll();
    /*
     * Called when the opmode is de-selected / no additional methods will be called.
     */
  }
}

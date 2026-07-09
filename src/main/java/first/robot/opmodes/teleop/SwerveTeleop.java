// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot.opmodes.teleop;

import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.command3.button.CommandGamepad;
import org.wpilib.math.kinematics.SwerveModuleVelocity;
import org.wpilib.opmode.PeriodicOpMode;
import org.wpilib.opmode.Teleop;

import first.robot.Robot;
import first.robot.utils.Constants.SwerveConstants;

@Teleop(name = "Swerve Teleop", group = "Group 1")
public class SwerveTeleop extends PeriodicOpMode {
  private final Robot robot;
  SwerveModuleVelocity velocity;
  public CommandGamepad controller = new CommandGamepad(0);

  /** The Robot instance is passed into the opmode via the constructor. */
  public SwerveTeleop(Robot robot) {
    this.robot = robot;
    robot.drive.setDefaultCommand(robot.drive
        .runRepeatedly(
            () -> robot.drive.drive(
                -controller.getLeftY() * SwerveConstants.maxTranslationalSpeed,
                -controller.getLeftX() * SwerveConstants.maxTranslationalSpeed,
                controller.getRightX() * SwerveConstants.maxAngularVelocity,
                true,
                true,
                .02))
        .withPriority(Command.LOWEST_PRIORITY)
        .named("swerve Drive (Default Command)"));
  }

  @Override
  public void disabledPeriodic() {
    /* Called periodically (on every DS packet) while the robot is disabled. */
    Scheduler.getDefault().run();

    robot.drive.frontLeft.moduleTelemtry();
    robot.drive.frontRight.moduleTelemtry();
    robot.drive.backLeft.moduleTelemtry();
    robot.drive.backRight.moduleTelemtry();

  }

  @Override
  public void start() {
    /* Called once when the robot is enabled. */

  }

  @Override
  public void periodic() {
    Scheduler.getDefault().run();

    robot.drive.frontLeft.moduleTelemtry();
    robot.drive.frontRight.moduleTelemtry();
    robot.drive.backLeft.moduleTelemtry();
    robot.drive.backRight.moduleTelemtry();

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

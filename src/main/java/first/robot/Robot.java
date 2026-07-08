// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot;

import org.wpilib.command3.Command;
import org.wpilib.command3.button.CommandGamepad;
import org.wpilib.framework.OpModeRobot;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.SwerveModuleVelocity;

import first.robot.subsystems.SwerveModule;
import first.robot.utils.Constants.SwerveConstants;

/**
 * The methods in this class are called automatically as described in the
 * OpModeRobot documentation.
 * OpMode classes anywhere in the package (or sub-packages) where this class is
 * located are
 * automatically registered to display in the Driver Station. If you change the
 * name of this class
 * or the package after creating this project, you must also update the
 * Main.java file in the
 * project.
 */
public class Robot extends OpModeRobot {

  // public Drivetrain drive = new Drivetrain();

  public SwerveModule modulefl = new SwerveModule(0, SwerveConstants.Mod0.constants);
  public SwerveModule modulefr = new SwerveModule(1, SwerveConstants.Mod1.constants);
  public SwerveModule modulerl = new SwerveModule(2, SwerveConstants.Mod2.constants);
  public SwerveModule modulerr = new SwerveModule(3, SwerveConstants.Mod3.constants);

  public CommandGamepad driverController = new CommandGamepad(0);

  public Robot() {
    /**
     * This function is run when the robot is first started up and should be used
     * for any
     * initialization code.
     */

  }

  @Override
  public void publishOpModes() {
    // TODO Auto-generated method stub
    super.publishOpModes();
  }

  /** This function is called exactly once when the DS first connects. */
  @Override
  public void driverStationConnected() {
  }

  /**
   * This function is called periodically anytime when no opmode is selected,
   * including when the Driver Station is disconnected.
   */
  @Override
  public void nonePeriodic() {

  }

  public Command setVelocityCommand(SwerveModule module, double angleDegrees) {
    SwerveModuleVelocity velocity = new SwerveModuleVelocity(0, Rotation2d.fromDegrees(angleDegrees));
    return Command.noRequirements(coro -> module.setDesiredVelocity(velocity, true)).named("Set Test Angle");

  }

}
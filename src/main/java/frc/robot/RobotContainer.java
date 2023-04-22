// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem exampleSubsystem = new ExampleSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Update drive based on controller

    // Doesn't run?

    // new RunCommand(() -> {
    //   double x = MathUtil.applyDeadband(driverController.getLeftX(), 0.015);
    //   double y = MathUtil.applyDeadband(-driverController.getLeftY(), 0.015);
    //   double speed = Math.hypot(x, y)
    //       * DriveConstants.kMaxSpeedMetersPerSecond;
    //   double angleRadians = Math.atan2(y, x);
    //   double cwRotationSpeed = driverController.getRightX();
    //   driveSubsystem.updateVelocity(angleRadians, speed, cwRotationSpeed);
    //   System.out.printf("Driving towards: %.2f %.2f at speed %.2f with rot %.2f\n", x, y, speed, cwRotationSpeed);
    // }, driveSubsystem).schedule();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(exampleSubsystem);
  }
}

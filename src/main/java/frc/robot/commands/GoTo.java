// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class GoTo extends CommandBase {
  private final DriveSubsystem driveSubsystem;
  private final Pose2d targetPose;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public GoTo(DriveSubsystem driveSubsystem, Pose2d targetPose) {
    this.driveSubsystem = driveSubsystem;
    this.targetPose = targetPose; 
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d currentPose = driveSubsystem.getPose();
    Twist2d twist = currentPose.log(targetPose);

    double vxMetersPerSecond = twist.dx;
    double vyMetersPerSecond = twist.dy;
    double omegaRadiansPerSecond = twist.dtheta;

    double speed = Math.hypot(vxMetersPerSecond, vyMetersPerSecond);

    // Clamp max speed
    if (speed > DriveConstants.kMaxSpeedMetersPerSecond) {
      vxMetersPerSecond *= DriveConstants.kMaxSpeedMetersPerSecond/speed;
      vyMetersPerSecond *= DriveConstants.kMaxSpeedMetersPerSecond/speed;
    }

    double angleRadians = Math.abs(speed) > 1e-6 ? Math.atan2(vyMetersPerSecond, vxMetersPerSecond) : 0;

    driveSubsystem.updateVelocity(angleRadians, speed, -omegaRadiansPerSecond);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Pose2d currentPose = driveSubsystem.getPose();
    Transform2d diff = targetPose.minus(currentPose);
    return Math.abs(diff.getX()) < 0.1 && Math.abs(diff.getY()) < 0.1 && Math.abs(diff.getRotation().getDegrees()) < 10;
  }
}

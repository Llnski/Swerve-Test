// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.CANIds;

import static frc.robot.util.TranslationUtils.*;

public class DriveSubsystem extends SubsystemBase {
  SwerveModule backLeft, backRight, frontLeft, frontRight;

  Translation2d centerOfRotation = new Translation2d();
  Translation2d velocity = new Translation2d();
  double cwRotationSpeed = 0;

  SwerveModule[] swerveModules = new SwerveModule[4];

  public DriveSubsystem() {
    // Back left
    swerveModules[0] = new SwerveModule(
        new CANSparkMax(CANIds.kBackLeftSpeed, DriveConstants.kMotorType),
        new CANSparkMax(CANIds.kBackLeftPivot, DriveConstants.kMotorType),
        DriveConstants.backLeftPosition,
        centerOfRotation);

    // Back right
    swerveModules[1] = new SwerveModule(
        new CANSparkMax(CANIds.kBackRightSpeed, DriveConstants.kMotorType),
        new CANSparkMax(CANIds.kBackRightPivot, DriveConstants.kMotorType),
        DriveConstants.frontLeftPosition,
        centerOfRotation);

    // Front left
    swerveModules[2] = new SwerveModule(
        new CANSparkMax(CANIds.kFrontLeftSpeed, DriveConstants.kMotorType),
        new CANSparkMax(CANIds.kFrontLeftPivot, DriveConstants.kMotorType),
        DriveConstants.frontLeftPosition,
        centerOfRotation);

    // Front right
    swerveModules[3] = new SwerveModule(
        new CANSparkMax(CANIds.kFrontRightSpeed, DriveConstants.kMotorType),
        new CANSparkMax(CANIds.kFrontRightPivot, DriveConstants.kMotorType),
        DriveConstants.frontRightPosition,
        centerOfRotation);
  }

  public void updateVelocity(double angleRadians, double speed, double cwRotationSpeed) {
    speed = MathUtil.clamp(speed, 0, DriveConstants.kMaxSpeedMetersPerSecond); // Clamp speed
    this.velocity = new Translation2d(Math.cos(angleRadians), Math.sin(angleRadians)).times(speed);
    this.cwRotationSpeed = cwRotationSpeed;

    double maxModuleSpeed = Double.NEGATIVE_INFINITY;

    for (SwerveModule module : swerveModules) {
      module.updateLocalVelocity(velocity, cwRotationSpeed);
      maxModuleSpeed = Math.max(maxModuleSpeed, module.getTargetSpeed());
    }

    // If max module speed (velocity + rotation) is greater than max NEO speed,
    // then rescale each module speed accordingly
    if (maxModuleSpeed > DriveConstants.kMaxSpeedMetersPerSecond) {
      for (SwerveModule module : swerveModules) {
        module.setTargetSpeed(module.getTargetSpeed() / maxModuleSpeed * DriveConstants.kMaxSpeedMetersPerSecond);
      }
    }
  }

  public void updateCenterOfRotation(Translation2d centerOfRotation) {
    for (SwerveModule module : swerveModules) {
      module.updateCenterOfRotation(centerOfRotation);
    }
  }

  @Override
  public void periodic() {
    // SwerveModules update motors locally
  }
}

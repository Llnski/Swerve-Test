// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveModule.SwerveModuleBuilder;
import frc.robot.util.Vector2;
import frc.robot.Constants.CANIds;

import static frc.robot.util.TranslationUtils.*;

public class DriveSubsystem extends SubsystemBase {
  SwerveModule backLeft, backRight, frontLeft, frontRight;

  Vector2 centerOfRotation = new Vector2(0, 0);
  Vector2 velocity = new Vector2(0, 0);
  double cwRotationSpeed = 0;

  SwerveModule swerveModule1, swerveModule2, swerveModule3, swerveModule4;
  SwerveModule[] swerveModules = new SwerveModule[4];

  public DriveSubsystem() {
    swerveModule1 = new SwerveModuleBuilder()
      .setSteeringMotor(new CANSparkMax(CANIds.kMod1SteeringMotor, DriveConstants.kMotorType))
      .setDriveMotor(new CANSparkMax(CANIds.kMod1DriveMotor, DriveConstants.kMotorType))
      .setPosition(DriveConstants.kMod1Position)
      .setCenterOfRotation(centerOfRotation)
      .build();

    swerveModule2 = new SwerveModuleBuilder()
      .setSteeringMotor(new CANSparkMax(CANIds.kMod2SteeringMotor, DriveConstants.kMotorType))
      .setDriveMotor(new CANSparkMax(CANIds.kMod2DriveMotor, DriveConstants.kMotorType))
      .setPosition(DriveConstants.kMod2Position)
      .setCenterOfRotation(centerOfRotation)
      .build();

    swerveModule3 = new SwerveModuleBuilder()
      .setSteeringMotor(new CANSparkMax(CANIds.kMod3SteeringMotor, DriveConstants.kMotorType))
      .setDriveMotor(new CANSparkMax(CANIds.kMod3DriveMotor, DriveConstants.kMotorType))
      .setPosition(DriveConstants.kMod3Position)
      .setCenterOfRotation(centerOfRotation)
      .build();

    swerveModule4 = new SwerveModuleBuilder()
      .setSteeringMotor(new CANSparkMax(CANIds.kMod4SteeringMotor, DriveConstants.kMotorType))
      .setDriveMotor(new CANSparkMax(CANIds.kMod4DriveMotor, DriveConstants.kMotorType))
      .setPosition(DriveConstants.kMod4Position)
      .setCenterOfRotation(centerOfRotation)
      .build();

    swerveModules[0] = swerveModule1;
    swerveModules[1] = swerveModule2;
    swerveModules[2] = swerveModule3;
    swerveModules[3] = swerveModule4;
  
    for (SwerveModule module : swerveModules) {
      module.setIdleMode(IdleMode.kBrake);
    }

    System.out.println("Registering swerve modules");
    // CommandScheduler.getInstance().registerSubsystem(swerveModules);
  }

  public void updateVelocity(double angleRadians, double speed, double cwRotationSpeed) {
    double clampedSpeed = MathUtil.clamp(speed, 0, DriveConstants.kMaxSpeedMetersPerSecond); // Clamp speed
    this.velocity = new Vector2(Math.cos(angleRadians), Math.sin(angleRadians)).times(clampedSpeed);
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

  public void updateCenterOfRotation(Vector2 centerOfRotation) {
    for (SwerveModule module : swerveModules) {
      module.updateCenterOfRotation(centerOfRotation);
    }
  }

  public void setIdleMode(IdleMode idleMode) {
    for (SwerveModule module : swerveModules) {
      module.setIdleMode(idleMode);
    }
  }

  @Override
  public void periodic() {
    // SwerveModules update motors locally

    // double backLeftPosition = swerveModules[0].getPivotEncoder().getPosition();
    // double angleDegrees = Math.toDegrees(swerveModules[0].getCurrentAngleRadians());
    // System.out.println("Back left position: " + angleDegrees);
  }
}

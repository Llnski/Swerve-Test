// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveModule.SwerveModuleBuilder;
import frc.robot.util.Vector2;
import frc.robot.Constants.CANIds;

public class DriveSubsystem extends SubsystemBase {
  SwerveModule backLeft, backRight, frontLeft, frontRight;

  Vector2 centerOfRotation = new Vector2(0, 0);
  Vector2 velocity = new Vector2(0, 0);
  double cwRotationSpeed = 0;

  public SwerveModule swerveModule1, swerveModule2, swerveModule3, swerveModule4;
  SwerveModule[] swerveModules = new SwerveModule[4];

  public DriveSubsystem() {
    swerveModule1 = new SwerveModuleBuilder("1")
        .setSteeringMotorId(CANIds.kMod1SteeringMotor)
        .setDriveMotorId(CANIds.kMod1DriveMotor)
        .setCanCoderId(CANIds.kMod1CANCoder)
        .setPosition(DriveConstants.kMod1Position)
        .setCenterOfRotation(centerOfRotation)
        .setCANCoderOffset(DriveConstants.kMod1CANCoderOffset)
        .build();

    swerveModule2 = new SwerveModuleBuilder("2")
        .setSteeringMotorId(CANIds.kMod2SteeringMotor)
        .setDriveMotorId(CANIds.kMod2DriveMotor)
        .setCanCoderId(CANIds.kMod2CANCoder)
        .setPosition(DriveConstants.kMod2Position)
        .setCenterOfRotation(centerOfRotation)
        .setCANCoderOffset(DriveConstants.kMod2CANCoderOffset)
        .build();

    swerveModule3 = new SwerveModuleBuilder("3")
        .setSteeringMotorId(CANIds.kMod3SteeringMotor)
        .setDriveMotorId(CANIds.kMod3DriveMotor)
        .setCanCoderId(CANIds.kMod3CANCoder)
        .setPosition(DriveConstants.kMod3Position)
        .setCANCoderOffset(DriveConstants.kMod3CANCoderOffset)
        .setCenterOfRotation(centerOfRotation)
        .build();

    swerveModule4 = new SwerveModuleBuilder("4")
        .setSteeringMotorId(CANIds.kMod4SteeringMotor)
        .setDriveMotorId(CANIds.kMod4DriveMotor)
        .setCanCoderId(CANIds.kMod4CANCoder)
        .setPosition(DriveConstants.kMod4Position)
        .setCANCoderOffset(DriveConstants.kMod4CANCoderOffset)
        .setCenterOfRotation(centerOfRotation)
        .build();

    swerveModules[0] = swerveModule1;
    swerveModules[1] = swerveModule2;
    swerveModules[2] = swerveModule3;
    swerveModules[3] = swerveModule4;

    for (SwerveModule module : swerveModules) {
      module.setIdleMode(IdleMode.kBrake, IdleMode.kBrake);
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

  public void setIdleMode(IdleMode driveIdleMode, IdleMode steeringIdleMode) {
    for (SwerveModule module : swerveModules) {
      module.setIdleMode(driveIdleMode, steeringIdleMode);
    }
  }

  public Vector2 getPosition() {
    Vector2 total = new Vector2();
    for (SwerveModule sm : swerveModules) {
      total.plus(sm.getFieldPosition());
    }
    return total.div(4);
  }

  @Override
  public void periodic() {
    // SwerveModules update motors locally

    // double backLeftPosition = swerveModules[0].getPivotEncoder().getPosition();
    // double angleDegrees =
    // Math.toDegrees(swerveModules[0].getCurrentAngleRadians());
    // System.out.println("Back left position: " + angleDegrees);
  }
}

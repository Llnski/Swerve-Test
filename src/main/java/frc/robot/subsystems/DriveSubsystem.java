// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Arrays;
import java.util.stream.Collectors;

import com.ctre.phoenix.sensors.Pigeon2;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveModule.SwerveModuleBuilder;
import frc.robot.Constants.CANIds;

public class DriveSubsystem extends SubsystemBase {
  SwerveModule backLeft, backRight, frontLeft, frontRight;

  SwerveDriveKinematics kinematics;
  SwerveDriveOdometry odometry;
  SwerveDrivePoseEstimator poseEstimator;

  public SwerveModule swerveModule1, swerveModule2, swerveModule3, swerveModule4;
  SwerveModule[] swerveModules = new SwerveModule[4];

  Pigeon2 pigeon;

  Rotation2d getRotation() {
    return Rotation2d.fromDegrees(pigeon.getYaw());
  }

  public DriveSubsystem(Pigeon2 pigeon) {
    this.pigeon = pigeon;

    swerveModule1 = new SwerveModuleBuilder("1")
        .setSteeringMotorId(CANIds.kMod1SteeringMotor)
        .setDriveMotorId(CANIds.kMod1DriveMotor)
        .setCanCoderId(CANIds.kMod1CANCoder)
        .setPosition(DriveConstants.kMod1Position)
        .setCANCoderOffset(DriveConstants.kMod1CANCoderOffset)
        .build();

    swerveModule2 = new SwerveModuleBuilder("2")
        .setSteeringMotorId(CANIds.kMod2SteeringMotor)
        .setDriveMotorId(CANIds.kMod2DriveMotor)
        .setCanCoderId(CANIds.kMod2CANCoder)
        .setPosition(DriveConstants.kMod2Position)
        .setCANCoderOffset(DriveConstants.kMod2CANCoderOffset)
        .build();

    swerveModule3 = new SwerveModuleBuilder("3")
        .setSteeringMotorId(CANIds.kMod3SteeringMotor)
        .setDriveMotorId(CANIds.kMod3DriveMotor)
        .setCanCoderId(CANIds.kMod3CANCoder)
        .setPosition(DriveConstants.kMod3Position)
        .setCANCoderOffset(DriveConstants.kMod3CANCoderOffset)
        .build();

    swerveModule4 = new SwerveModuleBuilder("4")
        .setSteeringMotorId(CANIds.kMod4SteeringMotor)
        .setDriveMotorId(CANIds.kMod4DriveMotor)
        .setCanCoderId(CANIds.kMod4CANCoder)
        .setPosition(DriveConstants.kMod4Position)
        .setCANCoderOffset(DriveConstants.kMod4CANCoderOffset)
        .build();

    kinematics = new SwerveDriveKinematics(
      DriveConstants.kMod1Position,
      DriveConstants.kMod2Position,
      DriveConstants.kMod3Position,
      DriveConstants.kMod4Position
    );

    swerveModules[0] = swerveModule1;
    swerveModules[1] = swerveModule2;
    swerveModules[2] = swerveModule3;
    swerveModules[3] = swerveModule4;

    for (SwerveModule module : swerveModules) {
      module.setIdleMode(IdleMode.kBrake, IdleMode.kBrake);
    }

    System.out.println("Registering swerve modules");
    // CommandScheduler.getInstance().registerSubsystem(swerveModules);

    SwerveModulePosition[] modulePositions = new SwerveModulePosition[swerveModules.length];

    for (int i = 0; i < modulePositions.length; i++){
      modulePositions[i] = swerveModules[i].getPosition();
    }

    odometry = new SwerveDriveOdometry(kinematics, getRotation(), modulePositions);
  }

  public void updateSpeeds(ChassisSpeeds fieldRelativeSpeeds) {
    double yawDegrees = pigeon.getYaw();
    Rotation2d robotRotation = Rotation2d.fromDegrees(yawDegrees);
    ChassisSpeeds robotRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, robotRotation);
    SwerveModuleState[] states = kinematics.toSwerveModuleStates(robotRelativeSpeeds);
    
    for (int i = 0; i < states.length; i++) {
      SwerveModuleState state = states[i];
      SwerveModule module = swerveModules[i];
      Rotation2d moduleRotation = module.getRotation();
      SwerveModuleState optimizedState = SwerveModuleState.optimize(state, moduleRotation);
      module.updateTargetState(optimizedState);
    }
  }

  public void setIdleMode(IdleMode driveIdleMode, IdleMode steeringIdleMode) {
    for (SwerveModule module : swerveModules) {
      module.setIdleMode(driveIdleMode, steeringIdleMode);
    }
  }

  public Pose2d getPoseMeters() {
    return odometry.getPoseMeters();
  }

  @Override
  public void periodic() {
    // SwerveModules update motors locally

    SwerveModulePosition[] modulePositions = new SwerveModulePosition[swerveModules.length];

    for (int i = 0; i < modulePositions.length; i++){
      modulePositions[i] = swerveModules[i].getPosition();
    }

    odometry.update(getRotation(), modulePositions);


    // double backLeftPosition = swerveModules[0].getPivotEncoder().getPosition();
    // double angleDegrees =
    // Math.toDegrees(swerveModules[0].getCurrentAngleRadians());
    // System.out.println("Back left position: " + angleDegrees);
  }
}

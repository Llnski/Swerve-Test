// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.util.Units;
import frc.robot.util.Vector2;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class AutonConstants{
    public static final int EXTENSION_SPEED_1 = 1;
    public static final int ROTATION_SPEED_1 = 1;
    public static final int INTAKE_SPEED_1 = 1;
    public static final int EXTENSION_SPEED_2 = 1;
    public static final int ROTATION_SPEED_2 = 1;
    public static final int INTAKE_SPEED_2 = 1;
    public static final int DRIVE_ANGLE_1 = 1;
    public static final int DRIVE_SPEED_1 = 1;
    public static final int DRIVE_ROTATION_1 = 1;
  }
  public static class CANIds {
    // Module 1 (Front-right)
    public static final int kMod1CANCoder = 10;
    public static final int kMod1SteeringMotor = 11;
    public static final int kMod1DriveMotor = 12;

    // Module 2 (Front-left)
    public static final int kMod2CANCoder = 20;
    public static final int kMod2SteeringMotor = 21;
    public static final int kMod2DriveMotor = 22;

    // Module 3 (Back-left)
    public static final int kMod3CANCoder = 30;
    public static final int kMod3SteeringMotor = 31;
    public static final int kMod3DriveMotor = 32;

    // Module 4 (Back-right)
    public static final int kMod4CANCoder = 40;
    public static final int kMod4SteeringMotor = 41;
    public static final int kMod4DriveMotor = 42;
  
    public static final int kArmExtension = 60;
    public static final int kArmRotation = 61;
    public static final int kIntake = 62;
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class ArmConstants {
    public static final double kArmMinDegrees = -50;
    public static final double kArmMaxDegrees = 50;

    public static final double kArmGearRatio = 1;
    public static final double kArmRotationsToDegrees = 1;

    public static final double kArmP = 0.001;
    public static final double kArmI = 0;
    public static final double kArmD = 0;
  }

  public static class DriveConstants {
    public static final MotorType kMotorType = MotorType.kBrushless;

    public static final double kLength = Units.inchesToMeters(28);
    public static final double kWidth = Units.inchesToMeters(28);

    public static final Vector2 kMod1Position = new Vector2(kWidth / 2, kLength / 2);
    public static final Vector2 kMod2Position = new Vector2(-kWidth / 2, kLength / 2).unaryMinus();
    public static final Vector2 kMod3Position = new Vector2(-kWidth / 2, -kLength / 2);
    public static final Vector2 kMod4Position = new Vector2(kWidth / 2, -kLength / 2).unaryMinus();

    public static final double kMod1CANCoderOffset = 49.83398;
    public static final double kMod2CANCoderOffset = -134.20898;
    public static final double kMod3CANCoderOffset = -71.10352;
    public static final double kMod4CANCoderOffset = 0.87891;

    public static final double kMaxSpeedMetersPerSecond = 2.0; // TODO: Determine max possible/desired speed


    // Used to get pivot angle from NEO encoders (for now)
    public static final double kSteeringGearRatio = 1/21.665599999999998;
    public static final double kSteeringInitialAngleRadians = Math.PI / 2; // Have initial pivot angle be facing

    public static final double kDriveGearRatio = 1/6.75;
    public static final double kWheelRadiusMeters = Units.inchesToMeters(3.5);
    public static final double kWheelCircumferenceMeters = 2 * Math.PI * kWheelRadiusMeters;

    public static final double kBalanceP = 0.001;
    public static final double kBalanceI = 0;
    public static final double kBalanceD = 0;
  }
}

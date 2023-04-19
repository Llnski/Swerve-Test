// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

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
  public static class CANIds {
    public static final int kBackLeftPivot = 5;
    public static final int kBackLeftSpeed = 6;

    public static final int kBackRightPivot = 7;
    public static final int kBackRightSpeed = 8;

    public static final int kFrontLeftPivot = 9;
    public static final int kFrontLeftSpeed = 10;

    public static final int kFrontRightPivot = 11;
    public static final int kFrontRightSpeed = 12;
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class DriveConstants {
    public static final MotorType kMotorType = MotorType.kBrushless;
    public static final double kLength = Units.inchesToMeters(28);
    public static final double kWidth = Units.inchesToMeters(28);
    public static final Translation2d backLeftPosition = new Translation2d(-kWidth / 2, -kLength / 2);
    public static final Translation2d backRightPosition = new Translation2d(kWidth / 2, -kLength / 2);
    public static final Translation2d frontLeftPosition = new Translation2d(-kWidth / 2, kLength / 2);
    public static final Translation2d frontRightPosition = new Translation2d(kWidth / 2, kLength / 2);
  }
}

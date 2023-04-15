// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class DriveSubsystem extends SubsystemBase {
  private CANSparkMax motorBL;
  private CANSparkMax motorBR;
  private CANSparkMax motorFL;
  private CANSparkMax motorFR;

  public DriveSubsystem() {
      motorBL = new CANSparkMax(backLeft, MotorType.kBrushless);
      motorBR = new CANSparkMax(backRight, MotorType.kBrushless);
      motorFL = new CANSparkMax(frontLeft, MotorType.kBrushless);
      motorFR = new CANSparkMax(frontRight, MotorType.kBrushless);
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public CommandBase exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  public boolean exampleCondition() {
    return true;
  }
  double L = Constants.Length;
  double W = Constants.Width;
  public void drive (double x1, double y1, double x2) {

    double r = Math.sqrt ((L * L) + (W * W));
    y1 *= -1;

    double a = x1 - x2 * (L / r);
    double b = x1 + x2 * (L / r);
    double c = y1 - x2 * (W / r);
    double d = y1 + x2 * (W / r);

    double backRightSpeed = Math.sqrt ((a * a) + (d * d));
    double backLeftSpeed = Math.sqrt ((a * a) + (c * c));
    double frontRightSpeed = Math.sqrt ((b * b) + (d * d));
    double frontLeftSpeed = Math.sqrt ((b * b) + (c * c));

    double backRightAngle = Math.atan2 (a, d) / Math.pi;
    double backLeftAngle = Math.atan2 (a, c) / Math.pi;
    double frontRightAngle = Math.atan2 (b, d) / Math.pi;
    double frontLeftAngle = Math.atan2 (b, c) / Math.pi;
}
  @Override
  public void periodic() {
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}

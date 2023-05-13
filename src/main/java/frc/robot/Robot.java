// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenixpro.configs.CANcoderConfiguration;
import com.ctre.phoenixpro.hardware.CANcoder;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.DriveSubsystem;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private final XboxController driverController = new XboxController(
      OperatorConstants.kDriverControllerPort);

  private final DriveSubsystem driveSubsystem = new DriveSubsystem();

  CANcoder canCoder1;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    // canCoder1 = new CANcoder(30, "rio");
    // var config = new CANcoderConfiguration();
    // canCoder1.getConfigurator().apply(config);
    // canCoder1.getPosition().setUpdateFrequency(100);
    // canCoder1.getVelocity().setUpdateFrequency(100);
    driveSubsystem.setIdleMode(IdleMode.kCoast);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.

    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    driveSubsystem.setIdleMode(IdleMode.kBrake);

    driveSubsystem.swerveModule1.pivotEncoder.setPosition(0);
    driveSubsystem.swerveModule2.pivotEncoder.setPosition(0);
    driveSubsystem.swerveModule3.pivotEncoder.setPosition(0);
    driveSubsystem.swerveModule4.pivotEncoder.setPosition(0);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
      double x = MathUtil.applyDeadband(driverController.getLeftX(), 0.035);
      double y = MathUtil.applyDeadband(-driverController.getLeftY(), 0.035);
      double speed = Math.hypot(x, y)
          * DriveConstants.kMaxSpeedMetersPerSecond;
      double angleRadians = (Math.abs(x) > 0 || Math.abs(y) > 0) ? Math.atan2(y, x) : Math.PI / 2;
      double cwRotationSpeed = MathUtil.applyDeadband(driverController.getRightX(), 0.03);
      // "CW rotation" is really CCW, todo
      driveSubsystem.updateVelocity(angleRadians, speed, -cwRotationSpeed);
      System.out.printf("Driving towards: %.2f %.2f at speed %.2f with angle %.2f with rot %.2f\n", x, y, speed, Math.toDegrees(angleRadians), cwRotationSpeed);

      // double position1 = driveSubsystem.swerveModule1.canCoder.getPosition().getValue();
      // double position2 = driveSubsystem.swerveModule2.canCoder.getPosition().getValue();
      // double position3 = driveSubsystem.swerveModule3.canCoder.getPosition().getValue();
      // double position4 = driveSubsystem.swerveModule4.canCoder.getPosition().getValue();


      // System.out.printf("Pos 1: %.3f. Pos 2: %.3f. Pos 3: %.3f. Pos 4: %.3f\n", position1, position2, position3, position4);
      // var position = canCoder1.getPosition();
      // System.out.printf("Pos 1: %.5f. Latency: %.3f\n", position.getValue(), position.getTimestamp().getLatency());
  }

  @Override
  public void teleopExit() {
    driveSubsystem.setIdleMode(IdleMode.kCoast);
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();

    // driveSubsystem.swerveModule1.canCoder.setPosition(0);
    // driveSubsystem.swerveModule2.canCoder.setPosition(0);
    // driveSubsystem.swerveModule3.canCoder.setPosition(0);
    // driveSubsystem.swerveModule4.canCoder.setPosition(0);
    canCoder1.setPosition(0.4);
    canCoder1.getPosition().waitForUpdate(0.1);
    System.out.println("Zeroed all CANCoders.");
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}

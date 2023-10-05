// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.GoTo;
import frc.robot.subsystems.ArmSubsystem;
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

  private final ArmSubsystem armSubsystem = new ArmSubsystem();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    driveSubsystem.setIdleMode(IdleMode.kBrake, IdleMode.kBrake);
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
    // m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // // schedule the autonomous command (example)
    // if (m_autonomousCommand != null) {
    //   m_autonomousCommand.schedule();
    // }

    var command = new SequentialCommandGroup(
      new GoTo(driveSubsystem, new Pose2d(new Translation2d(1, 0), Rotation2d.fromRadians(0))),
      new GoTo(driveSubsystem, new Pose2d(new Translation2d(1, -1), Rotation2d.fromRadians(0))),
      new GoTo(driveSubsystem, new Pose2d(new Translation2d(0, -1), Rotation2d.fromRadians(0))),
      new GoTo(driveSubsystem, new Pose2d(new Translation2d(0, 0), Rotation2d.fromRadians(0)))
    ).andThen(() -> {
      driveSubsystem.updateVelocity(Math.PI/2, 0, 0);
    });
      
    command.schedule();
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

    driveSubsystem.setIdleMode(IdleMode.kBrake, IdleMode.kBrake);
  }

  double speed = 0;
  double speedControlKp = 0.1;

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
      double x = MathUtil.applyDeadband(driverController.getLeftX(), 0.035);
      double y = MathUtil.applyDeadband(-driverController.getLeftY(), 0.035);
      double inputSpeed = Math.hypot(x, y)
          * DriveConstants.kMaxSpeedMetersPerSecond;
      double angleRadians = (Math.abs(x) > 1e-6 || Math.abs(y) > 1e-6) ? Math.atan2(y, x) : Math.PI / 2;
      double cwRotationSpeed = -MathUtil.applyDeadband(driverController.getRightX(), 0.03);

      // TODO: Either switch to something other than kP/weighted average
      // and/or characterize in terms of convergence time (i.e. how long from speed=0.5 to speed=1)

      speed += speedControlKp * (inputSpeed - speed);

      angleRadians *= -1;
      angleRadians -= Math.PI/2;

      driveSubsystem.updateVelocity(angleRadians, speed, -cwRotationSpeed);
      // System.out.printf("Driving towards: %.2f %.2f at speed %.2f with angle %.2f with rot %.2f\n", x, y, speed, Math.toDegrees(angleRadians), cwRotationSpeed);
      Pose2d pose = driveSubsystem.getPose();
      System.out.println(pose.toString());


      double armExtensionSpeed = driverController.getLeftTriggerAxis() * 0.2;
      boolean invertArmExtension = driverController.getLeftBumper();
      if (invertArmExtension) armExtensionSpeed *= -1;

      armSubsystem.setArmExtensionSpeed(armExtensionSpeed);

      double armRotationSpeed = driverController.getRightTriggerAxis() * 0.2;
      boolean invertArmRotation = driverController.getRightBumper();
      if (invertArmRotation) armRotationSpeed *= -1;

      armSubsystem.setArmRotationSpeed(armRotationSpeed);

      // System.out.printf("Pos 1: %.3f. Pos 2: %.3f. Pos 3: %.3f. Pos 4: %.3f\n", position1, position2, position3, position4);
      // var position = canCoder1.getPosition();
      // System.out.printf("Pos 1: %.5f. Latency: %.3f\n", position.getValue(), position.getTimestamp().getLatency());
  }

  @Override
  public void teleopExit() {
    driveSubsystem.setIdleMode(IdleMode.kCoast, IdleMode.kCoast);
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();

    // canCoder1.setPosition(0.4);
    // canCoder1.getPosition().waitForUpdate(0.1);
    // System.out.println("Zeroed all CANCoders.");
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {

    // var position = canCoder1.getAbsolutePosition();
    // System.out.printf("CANCoder 30: %.2f\n", position);

    double position1 = Math.toDegrees(driveSubsystem.swerveModule1.getCurrentAngleRadians());
    double position2 = Math.toDegrees(driveSubsystem.swerveModule2.getCurrentAngleRadians());
    double position3 = Math.toDegrees(driveSubsystem.swerveModule3.getCurrentAngleRadians());
    double position4 = Math.toDegrees(driveSubsystem.swerveModule4.getCurrentAngleRadians());


    System.out.printf("Pos 1: %.5f. Pos 2: %.5f. Pos 3: %.5f. Pos 4: %.5f\n", position1, position2, position3, position4);

  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}

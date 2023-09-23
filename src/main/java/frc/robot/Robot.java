// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.Pigeon2;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.CANIds;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SquareTest;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.util.Vector2;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private Vector2 prevPos;

  private RobotContainer m_robotContainer;

  private final XboxController driverController = new XboxController(
      OperatorConstants.kDriverControllerPort);

  private final DriveSubsystem driveSubsystem = new DriveSubsystem();

  public static Pigeon2 pigeon = new Pigeon2(CANIds.kPigeon);

  static {
      pigeon.setYaw(0);
  }

  CANCoder canCoder1;

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

    driveSubsystem.setIdleMode(IdleMode.kBrake, IdleMode.kBrake);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
      double x = MathUtil.applyDeadband(driverController.getLeftX(), 0.035);
      double y = MathUtil.applyDeadband(-driverController.getLeftY(), 0.035);
      double speed = Math.hypot(x, y)
          * DriveConstants.kMaxSpeedMetersPerSecond;
      double angleRadians = (Math.abs(x) > 1e-6 || Math.abs(y) > 1e-6) ? Math.atan2(y, x) : Math.PI / 2;
      double cwRotationSpeed = -MathUtil.applyDeadband(driverController.getRightX(), 0.03);
      angleRadians *= -1;
      angleRadians -= Math.PI/2;

      driveSubsystem.updateVelocity(angleRadians, speed, -cwRotationSpeed);
      System.out.printf("Driving towards: %.2f %.2f at speed %.2f with angle %.2f with rot %.2f\n", x, y, speed, Math.toDegrees(angleRadians), cwRotationSpeed);


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
    /* 
    double position1 = Math.toDegrees(driveSubsystem.swerveModule1.getCurrentAngleRadians());
    double position2 = Math.toDegrees(driveSubsystem.swerveModule2.getCurrentAngleRadians());
    double position3 = Math.toDegrees(driveSubsystem.swerveModule3.getCurrentAngleRadians());
    double position4 = Math.toDegrees(driveSubsystem.swerveModule4.getCurrentAngleRadians());


    System.out.printf("Pos 1: %.5f. Pos 2: %.5f. Pos 3: %.5f. Pos 4: %.5f\n", position1, position2, position3, position4);
    */

    if (Constants.SquareTest.currentTest=="Square1" || Constants.SquareTest.currentTest=="Square2") {
      int currSide = 0;

      
      double x = driveSubsystem.getPosition().getX();
      double y = driveSubsystem.getPosition().getY();
      double desiredX = Constants.SquareTest.xLoc1[currSide];
      double desiredY = Constants.SquareTest.yLoc1[currSide];
      if (Constants.SquareTest.currentTest=="Square2"){
        desiredX = Constants.SquareTest.xLoc2[currSide];
        desiredY = Constants.SquareTest.yLoc2[currSide];
      }
      if (desiredX-x<=Constants.SquareTest.threshold && desiredY-y<=Constants.SquareTest.threshold) {
        int toAdd = currSide == 3 ? -3 : 1;
        currSide += toAdd;
      }
      else {
        
        double speed = 0.1 * DriveConstants.kMaxSpeedMetersPerSecond;
        double angleRadians = Math.atan2(desiredY-y,desiredX-x);
        double cwRotationSpeed = 0;
        // "CW rotation" is really CCW, todo
        driveSubsystem.updateVelocity(angleRadians, speed, -cwRotationSpeed);
        System.out.printf("Driving towards: %.2f %.2f at speed %.2f with angle %.2f with rot %.2f\n", x, y, speed, Math.toDegrees(angleRadians), cwRotationSpeed);
      }
    }
    if (Constants.SquareTest.currentTest=="Spin"){
      
      double cwRotationSpeed = 1*Constants.DriveConstants.kMaxSpeedMetersPerSecond;
      driveSubsystem.updateVelocity(0, 0, -cwRotationSpeed);
      System.out.printf("Current center: %.2f, change in position: %.2f\n",driveSubsystem.getPosition(),driveSubsystem.getPosition().minus(prevPos));
      prevPos = driveSubsystem.getPosition();
    }
    else{
      teleopExit();
    }
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}

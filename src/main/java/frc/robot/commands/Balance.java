package frc.robot.commands;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutonConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class Balance extends CommandBase {
    DriveSubsystem driveSubsystem;
    Pigeon2 pigeon;
    PIDController controller = new PIDController(AutonConstants.kBalanceP, AutonConstants.kBalanceI, AutonConstants.kBalanceD);
    double toleranceDegrees;

    public Balance(DriveSubsystem driveSubsystem, Pigeon2 pigeon, double toleranceDegrees) {
        this.pigeon = pigeon;
        this.driveSubsystem = driveSubsystem;
    }

    @Override
    public void execute() {
        double pitch = pigeon.getPitch();
        double output = this.controller.calculate(pitch, 0);
        System.out.printf("Balancing. Pitch: %.2f; output: %.2f\n", pitch, output);
        this.driveSubsystem.updateVelocity(-Math.PI/2, output, 0);
    }

    @Override
    public boolean isFinished() {
        System.out.println("Finished balancing");
        return Math.abs(this.controller.getPositionError()) < toleranceDegrees;
    }
}

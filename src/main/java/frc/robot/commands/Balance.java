package frc.robot.commands;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class Balance extends CommandBase {
    DriveSubsystem driveSubsystem;
    Pigeon2 pigeon;
    PIDController controller = new PIDController(DriveConstants.kBalanceP, DriveConstants.kBalanceI, DriveConstants.kBalanceD);
    double toleranceDegrees;

    public Balance(DriveSubsystem driveSubsystem, Pigeon2 pigeon, double toleranceDegrees) {
        this.pigeon = pigeon;
        this.driveSubsystem = driveSubsystem;

        this.controller.setSetpoint(0);
        this.controller.setTolerance(toleranceDegrees);
    }

    @Override
    public void execute() {
        double output = this.controller.calculate(pigeon.getPitch());
        this.driveSubsystem.updateVelocity(Math.PI/2, output, 0);
    }

    @Override
    public boolean isFinished() {
        return this.controller.atSetpoint();
    }
}

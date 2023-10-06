package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.CANIds;

public class ArmSubsystem extends SubsystemBase {
    private CANSparkMax armRotationMotor;
    private SparkMaxPIDController armRotationController;
    private RelativeEncoder armRotationEncoder;
    private double targetArmAngleDegrees;

    private CANSparkMax armExtensionMotor;
    private SparkMaxPIDController armExtensionController;

    private CANSparkMax intakeMotor;

    public ArmSubsystem() {
        this.armExtensionMotor = new CANSparkMax(CANIds.kArmExtension, MotorType.kBrushless);
        this.armExtensionController = this.armExtensionMotor.getPIDController();

        // this.armExtensionController.setP(ArmConstants.kArmP);
        // this.armExtensionController.setI(ArmConstants.kArmI);
        // this.armExtensionController.setD(ArmConstants.kArmD);

        this.armRotationMotor = new CANSparkMax(CANIds.kArmRotation, MotorType.kBrushless);
        this.armRotationController = this.armRotationMotor.getPIDController();

        this.intakeMotor = new CANSparkMax(CANIds.kIntake, MotorType.kBrushed);

        // this.armRotationController.setP(ArmConstants.kArmP);
        // this.armRotationController.setI(ArmConstants.kArmI);
        // this.armRotationController.setD(ArmConstants.kArmD);

        this.targetArmAngleDegrees = 0;

        this.armRotationEncoder = armRotationMotor.getEncoder();
        this.armRotationEncoder.setPosition(0);
        this.armRotationEncoder.setPositionConversionFactor(ArmConstants.kArmRotationsToDegrees);

        this.armExtensionMotor.setIdleMode(IdleMode.kBrake);
        this.armRotationMotor.setIdleMode(IdleMode.kBrake);
    }

    public void setArmExtensionSpeed(double speedPercent) {
        armExtensionMotor.set(speedPercent);
    }

    public void setArmRotationSpeed(double speedPercent) {
        armRotationMotor.set(speedPercent);
    }

    public void setIntakeSpeed(double speedPercent) {
        intakeMotor.set(speedPercent);
    }

    public void setAngle(double degrees) {
        this.targetArmAngleDegrees = degrees; 
        this.armRotationController.setReference(this.targetArmAngleDegrees, ControlType.kPosition);
    }

    public void changeAngle(double deltaDegrees) {
        this.targetArmAngleDegrees += deltaDegrees;
        this.armRotationController.setReference(this.targetArmAngleDegrees, ControlType.kPosition);
    }

    public void setIntakeVelocity(double velocity) {
        this.intakeMotor.set(velocity);
    }
}

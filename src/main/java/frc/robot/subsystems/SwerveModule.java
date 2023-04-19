package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

import static frc.robot.util.TranslationUtils.*;

public class SwerveModule extends SubsystemBase {
    private CANSparkMax speedMotor, pivotMotor;

    // Position vectors for updating speed
    private Translation2d position, corToPosition, cwPerpDirection;

    private Translation2d targetLocalVelocity;

    private PIDController pivotController = new PIDController(0.1, 0, 0, 0);
    private SparkMaxPIDController velocityController = speedMotor.getPIDController();

    private RelativeEncoder pivotEncoder = pivotMotor.getEncoder();

    private double rotationSpeed = 0;

    public SwerveModule(CANSparkMax speedMotor, CANSparkMax pivotMotor, Translation2d position,
            Translation2d centerOfRotation) {
        this.speedMotor = speedMotor;
        this.pivotMotor = pivotMotor;
        this.position = position;
        this.corToPosition = position.minus(centerOfRotation);
        this.cwPerpDirection = normalize(cwPerp(corToPosition));

        // Continuous across angles
        this.pivotController.enableContinuousInput(-180, 180);
    }

    public void updateLocalVelocity(Translation2d targetChassisVelocity, double rotationSpeed) {
        this.rotationSpeed = rotationSpeed;
        targetLocalVelocity = targetChassisVelocity.plus(cwPerpDirection.times(rotationSpeed));

        double currentAngle = getCurrentAngleRadians();
        double targetAngleRadians = Math.atan2(targetLocalVelocity.getY(), targetLocalVelocity.getX());
        if (Math.abs(currentAngle - targetAngleRadians) > Math.PI / 2) {
            targetAngleRadians = Math.atan2(targetLocalVelocity.getY(), targetLocalVelocity.getX());
        }

        this.pivotController.setSetpoint(Math.toDegrees(targetAngleRadians));
    }

    public void updateCenterOfRotation(Translation2d newCenterOfRotation) {
        corToPosition = position.minus(newCenterOfRotation);
        updateLocalVelocity(newCenterOfRotation, rotationSpeed);
    }

    private double getCurrentAngleRadians() {
        double angle = pivotEncoder.getPosition() / DriveConstants.kSteeringGearRatio * 2 * Math.PI
                + DriveConstants.kSteeringInitialAngleRadians;
        return MathUtil.angleModulus(angle);
    }

    public Translation2d getTargetLocalVelocity() {
        return targetLocalVelocity;
    }

    public void setTargetSpeed(double speed) {
        targetLocalVelocity = normalize(targetLocalVelocity).times(speed);
    }

    public double getTargetSpeed() {
        return targetLocalVelocity.getNorm();
    }

    @Override
    public void periodic() {
        double currentAngleRadians = getCurrentAngleRadians();

        // Update motor velocity based on dot product between
        // and target local velocity current heading
        double speed = Math.cos(currentAngleRadians) * targetLocalVelocity.getX()
                + Math.sin(currentAngleRadians) * targetLocalVelocity.getY();
        velocityController.setReference(speed, ControlType.kVelocity);

        // Control motor to optimal heading
        double currentAngleDegrees = Math.toDegrees(currentAngleRadians);
        double pivotOutput = pivotController.calculate(currentAngleDegrees);
        pivotMotor.set(pivotOutput);
    }

}
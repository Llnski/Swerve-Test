package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.util.Vector2;

public class SwerveModule extends SubsystemBase {
    private CANSparkMax speedMotor, pivotMotor;

    // Position vectors for updating speed
    private Vector2 position, corToPosition, cwPerpDirection;

    private Vector2 targetLocalVelocity = new Vector2(0, 0);

    private PIDController pivotController = new PIDController(0.0045, 0.0008, 0);
    private SparkMaxPIDController velocityController;

    private RelativeEncoder pivotEncoder;

    private double rotationSpeed = 0;

    private boolean shouldFlipAngle = false;

    private boolean shouldNegateSpeed = false;

    public void shouldFlipAngle(boolean flipAngle) {
        this.shouldFlipAngle = flipAngle;
    }

    public RelativeEncoder getPivotEncoder() {
        return pivotEncoder;
    }

    public void setIdleMode(IdleMode idleMode) {
        pivotMotor.setIdleMode(idleMode);
        speedMotor.setIdleMode(idleMode);
    }

    public SwerveModule(CANSparkMax speedMotor, CANSparkMax pivotMotor, Vector2 position,
            Vector2 centerOfRotation) {
        this.speedMotor = speedMotor;
        this.pivotMotor = pivotMotor;

        this.speedMotor.restoreFactoryDefaults();
        this.pivotMotor.restoreFactoryDefaults();
        
        this.velocityController = speedMotor.getPIDController();
        this.velocityController.setP(0.01);

        this.pivotEncoder = pivotMotor.getEncoder();
        this.pivotEncoder.setPosition(0); // Zero position

        this.position = position;
        this.corToPosition = position.minus(centerOfRotation);
        this.cwPerpDirection = corToPosition.normalize().cwPerp();


        // Continuous across angles (degrees)
        this.pivotController.enableContinuousInput(-180, 180);
    }

    public void updateLocalVelocity(Vector2 targetChassisVelocity, double rotationSpeed) {
        this.rotationSpeed = rotationSpeed;
        targetLocalVelocity = targetChassisVelocity.plus(cwPerpDirection.times(rotationSpeed));

        double currentAngle = getCurrentAngleRadians();
        double targetAngleRadians = Math.atan2(targetLocalVelocity.getY(), targetLocalVelocity.getX());
        if (Math.abs(currentAngle - targetAngleRadians) > Math.PI / 2) {
            // targetLocalVelocity = targetLocalVelocity.unaryMinus();
            targetAngleRadians = -targetAngleRadians;
        }

        this.pivotController.setSetpoint(Math.toDegrees(targetAngleRadians));
    }

    public void updateCenterOfRotation(Vector2 newCenterOfRotation) {
        corToPosition = position.minus(newCenterOfRotation);
        updateLocalVelocity(newCenterOfRotation, rotationSpeed);
    }

    public double getCurrentAngleRadians() {
        double angle = (shouldFlipAngle ? -1 : 1) * pivotEncoder.getPosition() * DriveConstants.kSteeringGearRatio * 2 * Math.PI
                + DriveConstants.kSteeringInitialAngleRadians;
        double moddedAngle = MathUtil.angleModulus(angle);
        return moddedAngle;
    }

    public Vector2 getTargetLocalVelocity() {
        return targetLocalVelocity;
    }

    public void setTargetSpeed(double speed) {
        targetLocalVelocity = targetLocalVelocity.normalize().times(speed);
    }

    public double getTargetSpeed() {
        return targetLocalVelocity.norm();
    }

    @Override
    public void periodic() {
        double currentAngleRadians = getCurrentAngleRadians();

        // Update motor velocity based on dot product between
        // current heading and target local velocity
        double speed = Math.cos(currentAngleRadians) * targetLocalVelocity.getX()
                + Math.sin(currentAngleRadians) * targetLocalVelocity.getY();
        // velocityController.setReference(speed, ControlType.kVelocity);
        speedMotor.set(speed); // TODO: Use velocity

        // Control motor to optimal heading
        double currentAngleDegrees = Math.toDegrees(currentAngleRadians);
        double pivotOutput = pivotController.calculate(currentAngleDegrees);
        pivotMotor.set(pivotOutput);

        // System.out.print("Target local velocity: " + targetLocalVelocity.toString());
    }
}
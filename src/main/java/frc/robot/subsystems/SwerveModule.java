package frc.robot.subsystems;

import javax.sql.rowset.serial.SerialArray;

import com.ctre.phoenixpro.configs.CANcoderConfiguration;
import com.ctre.phoenixpro.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.util.Vector2;

public class SwerveModule extends SubsystemBase {
    private CANSparkMax driveMotor, steeringMotor;
    public CANcoder canCoder;

    // Position vectors for updating speed
    private Vector2 position, corToPosition, cwPerpDirection;

    private Vector2 targetLocalVelocity = new Vector2(0, 0);

    private PIDController pivotController = new PIDController(0.0045, 0.0008, 0);
    private SparkMaxPIDController velocityController;

    public RelativeEncoder pivotEncoder;

    private double rotationSpeed = 0;

    private boolean shouldFlipAngle = false;

    public void shouldFlipAngle(boolean flipAngle) {
        this.shouldFlipAngle = flipAngle;
    }

    public RelativeEncoder getPivotEncoder() {
        return pivotEncoder;
    }

    public void setIdleMode(IdleMode idleMode) {
        steeringMotor.setIdleMode(idleMode);
        driveMotor.setIdleMode(idleMode);
    }

    public SwerveModule(CANSparkMax steeringMotor, CANSparkMax driveMotor, CANcoder canCoder, Vector2 position,
            Vector2 centerOfRotation) {
        this.driveMotor = driveMotor;
        this.steeringMotor = steeringMotor;
        this.canCoder = canCoder;

        this.driveMotor.restoreFactoryDefaults();
        this.steeringMotor.restoreFactoryDefaults();
        
        this.velocityController = driveMotor.getPIDController();
        this.velocityController.setP(0.01);

        this.pivotEncoder = steeringMotor.getEncoder();
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
            // targetAngleRadians = -targetAngleRadians;
        }

        this.pivotController.setSetpoint(Math.toDegrees(targetAngleRadians));
    }

    public void updateCenterOfRotation(Vector2 newCenterOfRotation) {
        corToPosition = position.minus(newCenterOfRotation);
        updateLocalVelocity(newCenterOfRotation, rotationSpeed);
    }

    public double getCurrentAngleRadians() {
        // return canCoder.getAbsolutePosition().getValue();
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
        driveMotor.set(MathUtil.clamp(speed, -0.1, 0.1)); // TODO: Use velocity

        // Control motor to optimal heading
        double currentAngleDegrees = Math.toDegrees(currentAngleRadians);
        double pivotOutput = pivotController.calculate(currentAngleDegrees);
        steeringMotor.set(pivotOutput);

        // System.out.print("Target local velocity: " + targetLocalVelocity.toString());
    }

    public static class SwerveModuleBuilder {
        CANSparkMax steeringMotor = null, driveMotor = null;
        CANcoder canCoder = null;
        Vector2 position = null, centerOfRotation = null;

        public SwerveModuleBuilder setSteeringMotorId(int id) {
            this.steeringMotor = new CANSparkMax(id, MotorType.kBrushless);
            return this;
        }

        public SwerveModuleBuilder setDriveMotorId(int id) {
            this.driveMotor = new CANSparkMax(id, MotorType.kBrushless);
            return this;
        }

        public SwerveModuleBuilder setCanCoderId(int id) {
            this.canCoder = new CANcoder(id);
            this.canCoder.getPosition().setUpdateFrequency(100);
            return this;
        }

        public SwerveModuleBuilder setPosition(Vector2 position) {
            this.position = position;
            return this;
        }

        public SwerveModuleBuilder setCenterOfRotation(Vector2 centerOfRotation) {
            this.centerOfRotation = centerOfRotation;
            return this;
        }

        public SwerveModule build() {
            // TODO: Maybe this should be handled in
            // SwerveModule? Good enough for now
            if (steeringMotor == null
                    || driveMotor == null
                    || canCoder == null
                    || position == null
                    || centerOfRotation == null) {
                throw new IllegalArgumentException("Tried to build SwerveModule with incomplete parameters");
            }
            return new SwerveModule(steeringMotor,
                    driveMotor,
                    canCoder,
                    position,
                    centerOfRotation);
        }
    }
}
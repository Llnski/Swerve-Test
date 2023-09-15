package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.util.Vector2;

public class SwerveModule extends SubsystemBase {
    private CANSparkMax driveMotor, steeringMotor;
    public CANCoder CANCoder;

    private SparkMaxPIDController driveMotorController;

    // Position vectors for updating speed
    private Vector2 position, corToPosition, cwPerpDirection;

    private Vector2 targetLocalVelocity = new Vector2(0, 0);

    private PIDController pivotController = new PIDController(0.005, 0.0, 0.0001);
    private SparkMaxPIDController velocityController;

    public RelativeEncoder pivotEncoder;
    public RelativeEncoder driveEncoder;

    private Vector2 fieldPosition;

    private double rotationSpeed = 0;
    private double CANCoderAngleOffset;

    private boolean shouldFlipAngle = false;

    private String name;

    public double getPositionError() {
        return pivotController.getPositionError();
    }

    public Vector2 getFieldPosition() {
        return this.fieldPosition;
    }

    public void shouldFlipAngle(boolean flipAngle) {
        this.shouldFlipAngle = flipAngle;
    }

    public RelativeEncoder getPivotEncoder() {
        return pivotEncoder;
    }

    public void setIdleMode(IdleMode driveIdleMode, IdleMode steeringIdleMode) {
        driveMotor.setIdleMode(driveIdleMode);
        steeringMotor.setIdleMode(steeringIdleMode);
    }

    public SwerveModule(String name, CANSparkMax steeringMotor, CANSparkMax driveMotor, CANCoder CANCoder, Vector2 position,
            Vector2 centerOfRotation, double CANCoderAngleOffset) {
        this.name = name;
        this.driveMotor = driveMotor;
        this.driveMotorController = driveMotor.getPIDController();

        this.steeringMotor = steeringMotor;
        this.CANCoder = CANCoder;
        this.CANCoderAngleOffset = CANCoderAngleOffset;

        this.driveEncoder = this.driveMotor.getEncoder();

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
        // If within 0.5 degrees, don't care about velocity
        // this.pivotController.setTolerance(0.5, Double.POSITIVE_INFINITY);
    }

    public void updateLocalVelocity(Vector2 targetChassisVelocity, double rotationSpeed) {
        this.rotationSpeed = rotationSpeed;
        targetLocalVelocity = targetChassisVelocity.plus(cwPerpDirection.times(rotationSpeed));
        // System.out.println("Mod " + name + ": CW Perp Direction: " + cwPerpDirection);

        // TODO: Clean up
        double currentAngle = getCurrentAngleRadians();
        double targetAngleRadians = Math.abs(targetChassisVelocity.getX()) > 1e-6
                && Math.abs(targetChassisVelocity.getY()) > 1e-6
                        ? Math.atan2(targetLocalVelocity.getY(), targetLocalVelocity.getX())
                        : Math.PI / 2.0;
        // double targetAngleRadians = Math.atan2(targetLocalVelocity.getY(), targetLocalVelocity.getX());
        if (Math.abs(targetAngleRadians - currentAngle)
            < Math.abs(2 * Math.PI + targetAngleRadians - currentAngle)
        ) {
            targetLocalVelocity = targetLocalVelocity.unaryMinus();
            targetAngleRadians = -targetAngleRadians;
        }

        this.pivotController.setSetpoint(Math.toDegrees(targetAngleRadians));
    }

    public void updateCenterOfRotation(Vector2 newCenterOfRotation) {
        corToPosition = position.minus(newCenterOfRotation);
        updateLocalVelocity(newCenterOfRotation, rotationSpeed);
    }

    public double getCurrentAngleRadians() {
        // double angle = (shouldFlipAngle ? -1 : 1) * pivotEncoder.getPosition() *
        // DriveConstants.kSteeringGearRatio * 2 * Math.PI
        // + DriveConstants.kSteeringInitialAngleRadians;
        // double moddedAngle = MathUtil.angleModulus(angle);
    
        double angle = Math.toRadians(-(CANCoder.getAbsolutePosition() - CANCoderAngleOffset));
        return angle;
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
        // Update velocity control PID
        this.driveMotorController.setP(DriveConstants.kDriveVelocityP);
        this.driveMotorController.setI(DriveConstants.kDriveVelocityI);
        this.driveMotorController.setD(DriveConstants.kDriveVelocityD);

        System.out.printf("Mod %s: %.2f\n", name, pivotController.getPositionError());

        double currentAngleRadians = getCurrentAngleRadians();

        // Update motor velocity based on dot product between
        // current heading and target local velocity
        double speed = Math.cos(currentAngleRadians) * targetLocalVelocity.getX()
                + Math.sin(currentAngleRadians) * targetLocalVelocity.getY();
        // velocityController.setReference(speed, ControlType.kVelocity);
        if (speed < 0.0) {
            System.out.println("Mod " + name + ": Going backwards");
        }
        // driveMotor.set(MathUtil.clamp(speed, -0.1, 0.1)); // TODO: Use velocity
        double velocity = MathUtil.clamp(speed, -0.1, 0.1) * 3000;
        
        // Velocity control
        // driveMotorController.setReference(velocity, ControlType.kVelocity);

        // Control motor to optimal heading
        double currentAngleDegrees = Math.toDegrees(currentAngleRadians);
        double pivotOutput = pivotController.calculate(currentAngleDegrees);
        // atSetpoint() doesn't work?
        steeringMotor.set(pivotOutput);

        // System.out.printf("Target angle: %.2f; current angle: %.2f; error %.2f.  ", currentAngleDegrees, pivotController.getSetpoint(), pivotController.getPositionError());
        // System.out.print("Target local velocity: " + targetLocalVelocity.toString() + "; ");
    }

    public static class SwerveModuleBuilder {
        CANSparkMax steeringMotor = null, driveMotor = null;
        CANCoder canCoder = null;
        Vector2 position = null, centerOfRotation = null;
        Double CANCoderOffset = null;
        String name = "";

        SwerveModuleBuilder(String name) {
            this.name = name;
        }

        SwerveModuleBuilder() {}

        public SwerveModuleBuilder setSteeringMotorId(int id) {
            this.steeringMotor = new CANSparkMax(id, MotorType.kBrushless);
            return this;
        }

        public SwerveModuleBuilder setDriveMotorId(int id) {
            this.driveMotor = new CANSparkMax(id, MotorType.kBrushless);
            return this;
        }

        public SwerveModuleBuilder setCanCoderId(int id) {
            this.canCoder = new CANCoder(id);
            // this.canCoder.getPosition().setUpdateFrequency(100);
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

        public SwerveModuleBuilder setCANCoderOffset(double offset) {
            this.CANCoderOffset = offset;
            return this;
        }

        public SwerveModule build() {
            // TODO: Maybe this should be handled in
            // SwerveModule? Good enough for now
            if (steeringMotor == null
                    || driveMotor == null
                    || canCoder == null
                    || position == null
                    || centerOfRotation == null
                    || CANCoderOffset == null) {
                throw new IllegalArgumentException("Tried to build SwerveModule with incomplete parameters");
            }
            return new SwerveModule(name,
                    steeringMotor,
                    driveMotor,
                    canCoder,
                    position,
                    centerOfRotation, CANCoderOffset);
        }
    }
}
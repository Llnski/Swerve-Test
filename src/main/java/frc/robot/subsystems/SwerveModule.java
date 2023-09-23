package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.DriveConstants;

public class SwerveModule extends SubsystemBase {
    private final static GenericEntry RPMSoftLimit = Robot.tab.add("Max Voltage", 0.5).getEntry();

    private CANSparkMax driveMotor, steeringMotor;
    public CANCoder CANCoder;

    private SparkMaxPIDController driveMotorController;

    // Position vectors for updating speed
    private Translation2d position;

    private SwerveModuleState targetState;

    private PIDController pivotController = new PIDController(0.005, 0.0, 0.0001);
    private SparkMaxPIDController velocityController;

    public RelativeEncoder pivotEncoder;
    public RelativeEncoder driveEncoder;

    private Translation2d fieldPosition;

    private double CANCoderAngleOffset;

    private String name;

    public double getPositionError() {
        return pivotController.getPositionError();
    }

    public RelativeEncoder getPivotEncoder() {
        return pivotEncoder;
    }

    public void setIdleMode(IdleMode driveIdleMode, IdleMode steeringIdleMode) {
        driveMotor.setIdleMode(driveIdleMode);
        steeringMotor.setIdleMode(steeringIdleMode);
    }

    public SwerveModule(String name, CANSparkMax steeringMotor, CANSparkMax driveMotor, CANCoder CANCoder, Translation2d position,
            Translation2d centerOfRotation, double CANCoderAngleOffset) {
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
        
        // Continuous across angles (degrees)
        this.pivotController.enableContinuousInput(-180, 180);
        // If within 0.5 degrees, don't care about velocity
        // this.pivotController.setTolerance(0.5, Double.POSITIVE_INFINITY);
    }

    public void updateTargetState(SwerveModuleState state) {
        this.targetState = state;
        double speedRotations = state.speedMetersPerSecond * Constants.DriveConstants.kDriveGearRatio;
        this.driveMotorController.setReference(speedRotations, ControlType.kVelocity);
        this.pivotController.setSetpoint(state.angle.getDegrees());
    }

    public Rotation2d getRotation() {
        // double angle = (shouldFlipAngle ? -1 : 1) * pivotEncoder.getPosition() *
        // DriveConstants.kSteeringGearRatio * 2 * Math.PI
        // + DriveConstants.kSteeringInitialAngleRadians;
        // double moddedAngle = MathUtil.angleModulus(angle);
    
        double angleRadians = Math.toRadians(-(CANCoder.getAbsolutePosition() - CANCoderAngleOffset));
        Rotation2d rotation = new Rotation2d(angleRadians);
        return rotation;
    }

    public SwerveModulePosition getPosition() {
        double distanceMeters = driveEncoder.getPosition() * DriveConstants.kDriveGearRatio;
        return new SwerveModulePosition(distanceMeters, getRotation());
    }

    @Override
    public void periodic() {
        // Update velocity control PID
        this.driveMotorController.setP(DriveConstants.kDriveVelocityP);
        this.driveMotorController.setI(DriveConstants.kDriveVelocityI);
        this.driveMotorController.setD(DriveConstants.kDriveVelocityD);

        double limit = RPMSoftLimit.getDouble(0.0);
        // this.driveMotorController.setOutputRange(-limit, limit);

        double currentAngleDegrees = getRotation().getRadians();
        double pivotOutput = pivotController.calculate(currentAngleDegrees);

        steeringMotor.set(pivotOutput);
    }

    public static class SwerveModuleBuilder {
        CANSparkMax steeringMotor = null, driveMotor = null;
        CANCoder canCoder = null;
        Translation2d position = null, centerOfRotation = null;
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

        public SwerveModuleBuilder setPosition(Translation2d position) {
            this.position = position;
            return this;
        }

        public SwerveModuleBuilder setCenterOfRotation(Translation2d centerOfRotation) {
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
            return new SwerveModule(name,
                    steeringMotor,
                    driveMotor,
                    canCoder,
                    position,
                    centerOfRotation, CANCoderOffset);
        }
    }
}
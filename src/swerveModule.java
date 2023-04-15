import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class swerveModule {
    CANSparkMax angleMotor;
    CANSparkMax speedMotor;
    public swerveModule(int angleMID, int speedMID){
        angleMotor = new CANSparkMax(angleMID, MotorType.kBrushless);
        speedMotor = new CANSparkMax(speedMID, MotorType.kBrushless);
        pidController = new PIDController (1, 0, 0, new AnalogInput (encoder), this.angleMotor);
        pidController.setOutputRange (-1, 1);
        pidController.setContinuous ();
        pidController.enable ();
    }
    public setSpeed(double s){
        
    }

    public setAngle(double a){

    }
}

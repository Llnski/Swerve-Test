package frc.robot.util;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/*
    Temp class for organizing SmartDashboard logic
    TODO: Write custom dashboard
*/ 
public class ShuffleBoardManager {
    
    private static ShuffleBoardManager instance = null;

    public static ShuffleBoardManager getInstance() {
        if (instance == null) instance = new ShuffleBoardManager();
        return instance;
    }

    ShuffleboardTab tab = Shuffleboard.getTab("Swerve Test");

    GenericEntry kP = tab.add("kP", 0.0005).getEntry();
    GenericEntry kI = tab.add("kI", 0.0).getEntry();
    GenericEntry kD = tab.add("kD", 0.0).getEntry();

    public void updatePIDController(PIDController controller) {
        controller.setPID(kP.getDouble(0), kI.getDouble(0), kD.getDouble(0));
    }

    ShuffleBoardManager() {
        
        // SmartDashboard.putNumber("kP", 0.005);
        // SmartDashboard.putNumber("kI", 0.000);
        // SmartDashboard.putNumber("kD", 0.000);
    }


}

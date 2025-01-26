package frc.robot.subsystems.elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ElevatorConstants; 

public class ElevatorIOReal implements ElevatorIO {

    public static SparkMax elevatorMotorController;
    public static RelativeEncoder elevatorEncoder;
    private static final SparkMaxConfig sparkMaxConfig = new SparkMaxConfig();

    public ElevatorIOReal()
    {
        elevatorMotorController = new SparkMax(ElevatorConstants.ELEVATOR_MOTOR_ID, MotorType.kBrushless);
            // Constants.NEO_CURRENT_LIMIT, false, true, 0.1);
        
        sparkMaxConfig.encoder.positionConversionFactor(ElevatorConstants.METERS_PER_REVOLUTION)
        .velocityConversionFactor(ElevatorConstants.METERS_PER_REVOLUTION / 60);
    }

    @Override
    public double getEncoderPosition() {
        return elevatorEncoder.getPosition();
    }
    @Override
    public double getEncoderSpeed() {
        return elevatorEncoder.getVelocity();
    }
    
    @Override
    public void setMotorSpeed(double speed) {
        elevatorMotorController.set(speed);
    }

    @Override
    public void setEncoderPosition(double position) {
        elevatorEncoder.setPosition(position);
    }

    @Override
    public double getElevatorCurrent() {
        return elevatorMotorController.getOutputCurrent();
    }

    @Override
    public void periodicUpdate() {
        // Only code in here that relates a physical subsystem
        SmartDashboard.putNumber("elevator/Real motor temp (C)", elevatorMotorController.getMotorTemperature());
    }

}

package SimulationMath;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;

public class WheelVelocityCalculator {
    public WPI_TalonFX motor;
    public double lastPosition;
    public double lastTime;
    public double lastVelocity;

    
    public WheelVelocityCalculator(WPI_TalonFX motorID) {
        motor = motorID;
        lastPosition = calculatePosition(); 
        lastTime = Timer.getFPGATimestamp();
        lastVelocity = 0.0; 
    }

    public double calculateWheelVelocity() {
        double currentPosition = calculatePosition();
        double currentTime = Timer.getFPGATimestamp();
        double deltaTime = currentTime - lastTime;
        double deltaPosition = currentPosition - lastPosition;
        double velocity;

        if (deltaTime >= 0.1) { 
            velocity = deltaPosition / deltaTime;

            lastPosition = currentPosition;
            lastTime = currentTime;
            lastVelocity = velocity;
        } else {
            velocity = lastVelocity;
        }
        return (Math.PI * Units.inchesToMeters(6) * (((velocity * 10)/ 2048) / 8.45) / 60);
    }

    private double calculatePosition() {
        return motor.getSelectedSensorPosition();
    }
}
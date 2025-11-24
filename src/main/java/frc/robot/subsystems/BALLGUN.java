package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;

public class BALLGUN {
    private TalonFX upMotor = new TalonFX(0);
    private TalonFX rightMotor = new TalonFX(1);
    private TalonFX downMotor = new TalonFX(2);
    private TalonFX leftMotor = new TalonFX(3);

    private double upSpeed = 0;
    private double rightSpeed = 0;
    private double downSpeed = 0;
    private double leftSpeed = 0;

    //sets the speed of the motors
    public void setSpeeds(double xSpeed, double ySpeed, double totalSpeed) {
        upSpeed = totalSpeed * ((ySpeed + 1) / 2);
        rightSpeed = totalSpeed * ((xSpeed + 1) / 2);
        downSpeed = totalSpeed * ((1 - ySpeed) / 2);
        leftSpeed = totalSpeed * ((1 - ySpeed) / 2);

        upMotor.set(MathUtil.clamp(upSpeed * 0.25, -0.25, 0.25));
        rightMotor.set(MathUtil.clamp(rightSpeed * 0.25, -0.25, 0.25));
        downMotor.set(MathUtil.clamp(downSpeed * 0.25, -0.25, 0.25));
        leftMotor.set(MathUtil.clamp(leftSpeed * 0.25, -0.25, 0.25));
    }
}
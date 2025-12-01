package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;

public class BALLGUN {
    private TalonFX upMotor = new TalonFX(1);
    private TalonFX rightMotor = new TalonFX(2);
    private TalonFX downMotor = new TalonFX(3);
    private TalonFX leftMotor = new TalonFX(4);




    private double upSpeed = 0;
    private double rightSpeed = 0;
    private double downSpeed = 0;
    private double leftSpeed = 0;

    //sets the speed of the motors
    public void setSpeeds(double xSpeed, double ySpeed, double totalSpeed, double maxSpeed, double spinAmount) {
        upSpeed = totalSpeed * ((ySpeed * spinAmount + 1) / 2);
        rightSpeed = totalSpeed * ((xSpeed * spinAmount + 1) / 2);
        downSpeed = totalSpeed * ((1 - ySpeed * spinAmount) / 2);
        leftSpeed = totalSpeed * ((1 - xSpeed * spinAmount) / 2);

        upMotor.set(MathUtil.clamp(upSpeed * maxSpeed, -maxSpeed, maxSpeed));
        rightMotor.set(MathUtil.clamp(rightSpeed * maxSpeed, -maxSpeed, maxSpeed));
        downMotor.set(MathUtil.clamp(downSpeed * maxSpeed, -maxSpeed, maxSpeed));
        leftMotor.set(MathUtil.clamp(leftSpeed * maxSpeed, -maxSpeed, maxSpeed));

        System.out.println(MathUtil.clamp(upSpeed * maxSpeed, -maxSpeed, maxSpeed));
        System.out.println(MathUtil.clamp(rightSpeed * maxSpeed, -maxSpeed, maxSpeed));
        System.out.println(MathUtil.clamp(downSpeed * maxSpeed, -maxSpeed, maxSpeed));
        System.out.println(MathUtil.clamp(leftSpeed * maxSpeed, -maxSpeed, maxSpeed));
    }
    public void testSpeeds() {
        upMotor.set(0.1);
        rightMotor.set(0.1);
        downMotor.set(0.1);
        leftMotor.set(0.1);
    }
}
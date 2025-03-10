package frc.robot.subsystems;
//The deviceIds are currently null. Also check PID values because I don't know what I'm doing.-Sam
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pivot extends SubsystemBase{
    public TalonFX leftMotor = Ascender.pivotMotorLeft();
    public TalonFX rightMotor = new TalonFX(10);
    public MotorOutputConfigs leftConfig = new MotorOutputConfigs();
    public MotorOutputConfigs rightConfig = new MotorOutputConfigs();
    public PIDController mainPID = new PIDController(0.001, 0, 0);
    public PWM encoder = new PWM(0);
    public double defaultPosition = 0.1;
    public double goToPosition = defaultPosition;

    public Pivot() {
        leftConfig.NeutralMode = NeutralModeValue.Brake;
        rightConfig.NeutralMode = NeutralModeValue.Brake;
    
        leftMotor.getConfigurator().apply(leftConfig);
        rightMotor.getConfigurator().apply(rightConfig);
    
        leftMotor.setPosition(0);
        rightMotor.setPosition(0);
      }

      public void setPivot(double position) {
        mainPID.setSetpoint(position);
        if(position == 0) {
          goToPosition = defaultPosition;
        } else {
          goToPosition = 0;
        }
      }

      @Override
  public void periodic() {
    // This method will be called once per scheduler run
    var speed = mainPID.calculate(leftMotor.getPosition().getValueAsDouble());
    speed = MathUtil.clamp(speed, -0.01, 0.01);

    leftMotor.set(speed);
    rightMotor.set(-speed);
  }
  
  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

}

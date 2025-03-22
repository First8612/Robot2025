// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import com.ctre.phoenix6.configs.MotorOutputConfigs;
// import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.signals.NeutralModeValue;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.wpilibj.PWM;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// public class AlgaeExtender extends SubsystemBase {
//   public TalonFX extendMotor = new TalonFX(0);
//   public MotorOutputConfigs extendConfig = new MotorOutputConfigs();

//   public PIDController extendPID = new PIDController(0.1, 0, 0);

//   public PWM encoder = new PWM(0);
//   public double goToPosition = 5.3;
//   /** Creates a new AlgeaExtender. */
//   public AlgaeExtender() {
//     extendConfig.NeutralMode = NeutralModeValue.Brake;

//     extendMotor.getConfigurator().apply(extendConfig);

//     extendMotor.setPosition(0);
//   }

//   public void setAlgae(double position) {
//     extendPID.setSetpoint(position);
//     if(position == 0) {
//       goToPosition = 5.3;
//     } else {
//       goToPosition = 0;
//     }
//   }
//   /**
//    * Example command factory method.
//    *
//    * @return a command
//    */
//   public Command exampleMethodCommand() {
//     // Inline construction of command goes here.
//     // Subsystem::RunOnce implicitly requires `this` subsystem.
//     return runOnce(
//         () -> {
//           /* one-time action goes here */
//         });
//   }

//   /**
//    * An example method querying a boolean state of the subsystem (for example, a digital sensor).
//    *
//    * @return value of some boolean subsystem state, such as a digital sensor.
//    */
//   public boolean exampleCondition() {
//     // Query some boolean state, such as a digital sensor.
//     return false;
//   }

//   @Override
//   public void periodic() {
//     // This method will be called once per scheduler run
//     var speed = extendPID.calculate(extendMotor.getPosition().getValueAsDouble());
//     speed = MathUtil.clamp(speed, -0.1, 0.1);

//     extendMotor.set(speed);

//     SmartDashboard.putNumber("Algae/Encoder", encoder.getPosition());

//     SmartDashboard.putNumber("Extender Number", extendMotor.getPosition().getValueAsDouble());
//   }

//   @Override
//   public void simulationPeriodic() {
//     // This method will be called once per scheduler run during simulation
//   }
// }

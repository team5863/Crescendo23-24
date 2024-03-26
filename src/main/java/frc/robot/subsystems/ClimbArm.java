// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants;

public class ClimbArm extends SubsystemBase {
  public static CANSparkMax climbMotor;
  public static PIDController kPID_armrotation;
  public static DigitalInput rotateLimit;
  RelativeEncoder Encoder;

  //R for rotation
  double
      r_kP = 1,
      r_kI = 0,
      r_kD = 0;

    public ClimbArm() {

    //Motor
      climbMotor = new CANSparkMax(Constants.Climber.climberID, MotorType.kBrushless);
      climbMotor.restoreFactoryDefaults();
      // is inverted so that positive extends and negative retracts
      climbMotor.setInverted(true);
      climbMotor.setIdleMode(IdleMode.kBrake);
    //Sensor
      Encoder = climbMotor.getEncoder();

    //PID
      kPID_armrotation = new PIDController(r_kP, r_kI, r_kD);
      kPID_armrotation.setTolerance(1);
        kPID_armrotation.setP(r_kP);
        kPID_armrotation.setI(r_kI);
        kPID_armrotation.setD(r_kD);

    //Limit Switch
      rotateLimit = new DigitalInput(Constants.Climber.armLimitSwitch);


  }

  public void set(double speed) {

    climbMotor.set(speed);
  
}

  public void Positions(double target) {
        kPID_armrotation.setSetpoint(target);
        double speedOutput = MathUtil.clamp(kPID_armrotation.calculate(Encoder.getPosition()), -0.5, 0.5);
        set(-speedOutput);
    }
  @Override
  public void periodic() {
      //SmartDashboard.putNumber("Arm Position", Encoder.getPosition());
    // This method will be called once per scheduler run
  }
}

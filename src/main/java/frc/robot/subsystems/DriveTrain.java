// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;



public class DriveTrain extends SubsystemBase {
//Motors
public static CANSparkMax leftFrontMotor;
public static CANSparkMax leftBackMotor;
public static CANSparkMax rightFrontMotor;
public static CANSparkMax rightBackMotor;
  
//Sensors
 // public static Pigeon2 gyro;
  RelativeEncoder rightEncoder;
  RelativeEncoder leftEncoder;
  
//Drive Function
  DifferentialDrive drive;
  double gravityVec[] = new double[3];
  
//Distance targeting PIDController
  public static  PIDController d_PidController;

   double
      d_kp = 0,
      d_ki = 0,
      d_kd = 0;


  //PID values for heading targeting
  public static PIDController hLeft_PidController;

    double 
      hL_kp = 0, 
      hL_ki = 0, 
      hL_kd = 0;

  public static PIDController hRight_PidController;

    double 
      hR_kp = 0, 
      hR_ki = 0, 
      hR_kd = 0;

  public static PIDController dLeft_PidController;

    double 
      dL_kp = 0, 
      dL_ki = 0, 
      dL_kd = 0;

      
  public static PIDController dRight_PidController;

    double 
      dR_kp = 0, 
      dR_ki = 0, 
      dR_kd = 0;


  public static PIDController b_PidController;

    double
      b_kp = 0,
      b_ki = 0,
      b_kd = 0;

  public DriveTrain() {

    //Basic DriveTrain
        leftFrontMotor = new CANSparkMax(Constants.Drivetrain.leftFrontSparkiD, MotorType.kBrushless);
        leftBackMotor = new CANSparkMax(Constants.Drivetrain.leftBackSparkiD, MotorType.kBrushless);
        rightFrontMotor = new CANSparkMax(Constants.Drivetrain.rightFrontSparkiD, MotorType.kBrushless);
        rightBackMotor = new CANSparkMax(Constants.Drivetrain.rightBackSparkiD, MotorType.kBrushless);

        leftFrontMotor.restoreFactoryDefaults();
        leftBackMotor.restoreFactoryDefaults();
        rightFrontMotor.restoreFactoryDefaults();
        rightBackMotor.restoreFactoryDefaults();
        
        rightFrontMotor.setInverted(true);
        leftFrontMotor.setInverted(false);
        leftBackMotor.setInverted(false);
        rightBackMotor.setInverted(true);
        
        rightFrontMotor.setIdleMode(IdleMode.kBrake);
        leftFrontMotor.setIdleMode(IdleMode.kBrake);
        rightBackMotor.setIdleMode(IdleMode.kBrake);
        leftBackMotor.setIdleMode(IdleMode.kBrake);


        
        leftBackMotor.follow(leftFrontMotor);
        rightBackMotor.follow(rightFrontMotor);
        drive = new DifferentialDrive(leftFrontMotor, rightFrontMotor);
        drive.setSafetyEnabled(false);

        //Sensors
      //   gyro = new Pigeon2(Constants.Drivetrain.GyroID);
      //   gyro.setYaw(0);

         rightEncoder = rightFrontMotor.getEncoder();
         leftEncoder = leftFrontMotor.getEncoder();

         rightEncoder.setPositionConversionFactor(Constants.Drivetrain.driveTrainRatio);
         leftEncoder.setPositionConversionFactor(Constants.Drivetrain.driveTrainRatio);
         
         d_PidController = new PIDController(d_kp, d_ki, d_kd);
         hLeft_PidController = new PIDController(hL_kp, hL_ki, hL_kd);
         hRight_PidController = new PIDController(hR_kp, hR_ki, hR_kd);

         dLeft_PidController = new PIDController(dL_kp, dL_ki, dL_kd);
         dRight_PidController = new PIDController(dR_kp, dR_ki, dR_kd);

         b_PidController = new PIDController(b_kp, b_ki, b_kd);

         d_PidController.setTolerance(0.1);

         //working tolerance for TankDrivePID at RedStick -> 0.5
         //working speed for TankDrivePID at RedStick -> 0.5
         //working measurement to drive approximately 4 ft -> 2.46(overshoot and undershoot of 4ft is ok if under 1 ft)
         hLeft_PidController.setTolerance(0.1);
         hRight_PidController.setTolerance(0.1);

         dLeft_PidController.setTolerance(0.1);
         dRight_PidController.setTolerance(0.1);

         b_PidController.setTolerance(1);

         d_PidController.setP(d_kp);
         d_PidController.setI(d_ki);
         d_PidController.setD(d_kd);

         hLeft_PidController.setP(hL_kp);
         hLeft_PidController.setI(hL_ki);
         hLeft_PidController.setD(hL_kd);

         hRight_PidController.setP(hR_kp);
         hRight_PidController.setI(hR_ki);
         hRight_PidController.setD(hR_kd);

         
         dLeft_PidController.setP(dL_kp);
         dLeft_PidController.setI(dL_ki);
         dLeft_PidController.setD(dL_kd);

         dRight_PidController.setP(dR_kp);
         dRight_PidController.setI(dR_ki);
         dRight_PidController.setD(dR_kd);
         
         b_PidController.setP(b_kp);
         b_PidController.setI(b_ki);
         b_PidController.setD(b_kd);
      
  }
/* 
public double getYaw() {
  double yaw = gyro.getYaw().getValueAsDouble();

  if(gyro.getYaw().getValueAsDouble() >= 360 ) {

    gyro.setYaw(0);

  }

    return yaw;

}

public double getRoll() {
  double roll = gyro.getRoll().getValueAsDouble();

    return roll;
}

public double getPitch() {
  double pitch = gyro.getPitch().getValueAsDouble();

    return pitch;
}

/*public double[] gravityVector() {
    gyro.getGravityVector(gravityVec);
    
    //SmartDashboard.putNumberArray("GravityVector", gravityVec);

    return gravityVec;
}*/

/*public double getRotations() {
  double rotations = rightEncoder.getPosition();

  return rotations;
}*/



public void driveDistance(double distance, double heading) {

  

 double rotation = (((distance*12)/Constants.Drivetrain.wheelCircumference));

  d_PidController.setSetpoint(rotation);
  

  double speedOutput = MathUtil.clamp(d_PidController.calculate(rightEncoder.getPosition()), -0.4, 0.4);

 // double rotationOutput = MathUtil.clamp(hRight_PidController.calculate(gyro.getYaw().getValueAsDouble()), -0.4, 0.4);

 // drive.arcadeDrive(speedOutput, rotationOutput);
}

/*public void turn(double heading) {
  hRight_PidController.setSetpoint(heading);
  hLeft_PidController.setSetpoint(heading);

 double leftOutput = MathUtil.clamp(hLeft_PidController.calculate(gyro.getYaw().getValueAsDouble()), -0.3, 0.3);

  double rightOutput = MathUtil.clamp(hRight_PidController.calculate(gyro.getYaw().getValueAsDouble()), -0.3, 0.3);

  drive.tankDrive(-leftOutput, rightOutput);

}*/
public void distanceTank(double distance){

  double rotation = (((distance*12)/Constants.Drivetrain.wheelCircumference));

  dRight_PidController.setSetpoint(rotation);
  dLeft_PidController.setSetpoint(rotation);

  double leftOutput = MathUtil.clamp(dLeft_PidController.calculate(leftEncoder.getPosition()), -0.5, 0.5);

  double rightOutput = MathUtil.clamp(dRight_PidController.calculate(rightEncoder.getPosition()), -0.5, 0.5);

  drive.tankDrive(leftOutput, rightOutput);

}
public void arcadeDrive(double speed, double rotation) {
  drive.arcadeDrive(-speed, rotation);
}

  @Override
  public void periodic() { 

    //SmartDashboard.putNumberArray("GravityVector", gravityVector());
      SmartDashboard.putNumber("Rotations", rightEncoder.getPosition());
      SmartDashboard.putNumber("TargetRotations", dRight_PidController.getSetpoint());
  }

  @Override
  public void simulationPeriodic() {
  }
}

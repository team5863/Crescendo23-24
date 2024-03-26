// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
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
  public static Pigeon2 gyro;
  public static RelativeEncoder rightEncoder;
  public static RelativeEncoder leftEncoder;
  
//Drive Function
  DifferentialDrive drive;

//PID values for heading targeting
  public static PIDController kPID_headingLeft;
    double 
      kP_hL = 1, 
      kI_hL = 0, 
      kD_hL = 0;
  public static PIDController kPID_headingRight;
    double 
      kP_hR = 1, 
      kI_hR = 0, 
      kD_hR = 0;

  public static PIDController kPID_distanceLeft;
    double 
      kP_dL = 1, 
      kI_dL = 0, 
      kD_dL = 0;
    
  public static PIDController kPID_distanceRight;
    double 
      kP_dR = 1, 
      kI_dR = 0, 
      kD_dR = 0;

  public DriveTrain() {

    //DriveTrain
        //Left
            leftFrontMotor = new CANSparkMax(Constants.DifferentialConstants.leftFrontSparkiD, MotorType.kBrushless);
            leftBackMotor = new CANSparkMax(Constants.DifferentialConstants.leftBackSparkiD, MotorType.kBrushless);

            leftFrontMotor.restoreFactoryDefaults();
            leftBackMotor.restoreFactoryDefaults();

            leftFrontMotor.setInverted(false);
            leftBackMotor.setInverted(false);

            leftFrontMotor.setIdleMode(IdleMode.kBrake);
            leftBackMotor.setIdleMode(IdleMode.kBrake);

            leftBackMotor.follow(leftFrontMotor);
        //Right
            rightFrontMotor = new CANSparkMax(Constants.DifferentialConstants.rightFrontSparkiD, MotorType.kBrushless);
            rightBackMotor = new CANSparkMax(Constants.DifferentialConstants.rightBackSparkiD, MotorType.kBrushless);

            rightFrontMotor.restoreFactoryDefaults();
            rightBackMotor.restoreFactoryDefaults();

            rightFrontMotor.setInverted(true);
            rightBackMotor.setInverted(true);

            rightFrontMotor.setIdleMode(IdleMode.kBrake);
            rightBackMotor.setIdleMode(IdleMode.kBrake);

            rightBackMotor.follow(rightFrontMotor);
        //Differential
            drive = new DifferentialDrive(leftFrontMotor, rightFrontMotor);
            drive.setSafetyEnabled(false);
        //Gyro
            gyro = new Pigeon2(Constants.DifferentialConstants.GyroID);
            gyro.setYaw(0);

        //Encoders
            //Left
                leftEncoder = leftFrontMotor.getEncoder();
                leftEncoder.setPositionConversionFactor(Constants.DifferentialConstants.driveTrainRatio);

                kPID_headingLeft = new PIDController(kP_hL, kI_hL, kD_hL);
                kPID_headingLeft.setTolerance(0.1);
                    kPID_headingLeft.setP(kP_hL);
                    kPID_headingLeft.setI(kI_hL);
                    kPID_headingLeft.setD(kD_hL);

                kPID_distanceLeft = new PIDController(kP_dL, kP_dL, kD_dL);
                kPID_distanceLeft.setTolerance(0.1);
                    kPID_distanceLeft.setP(kP_dL);
                    kPID_distanceLeft.setI(kI_dL);
                    kPID_distanceLeft.setD(kD_dL);

            //Right
                rightEncoder = rightFrontMotor.getEncoder();
                rightEncoder.setPositionConversionFactor(Constants.DifferentialConstants.driveTrainRatio);

                kPID_headingRight = new PIDController(kP_hR, kI_hR, kD_hR);
                kPID_headingRight.setTolerance(0.1);
                    kPID_headingRight.setP(kP_hR);
                    kPID_headingRight.setI(kI_hR);
                    kPID_headingRight.setD(kD_hR);
                
                kPID_distanceRight = new PIDController(kP_dR, kI_dR, kD_dR);
                kPID_distanceRight.setTolerance(0.1);
                    kPID_distanceRight.setP(kP_dR);
                    kPID_distanceRight.setI(kI_dR);
                    kPID_distanceRight.setD(kD_dR);        
}

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

    public double getRotations() {
        double rotations = rightEncoder.getPosition();

        return rotations;
    }

    public void turn(double heading) {
        kPID_headingRight.setSetpoint(heading);
        kPID_headingLeft.setSetpoint(heading);

        double leftOutput = MathUtil.clamp(kPID_headingLeft.calculate(gyro.getYaw().getValueAsDouble()), -0.3, 0.3);
        double rightOutput = MathUtil.clamp(kPID_headingRight.calculate(gyro.getYaw().getValueAsDouble()), -0.3, 0.3);

        drive.tankDrive(-leftOutput, rightOutput);
    }

    public void distanceTank(double distance){
        double rotation = (((distance*12)/Constants.DifferentialConstants.wheelCircumference));
            kPID_distanceRight.setSetpoint(rotation);
            kPID_distanceLeft.setSetpoint(rotation);

        double leftOutput = MathUtil.clamp(kPID_distanceLeft.calculate(leftEncoder.getPosition()), -0.5, 0.5);
        double rightOutput = MathUtil.clamp(kPID_distanceRight.calculate(rightEncoder.getPosition()), -0.5, 0.5);

        drive.tankDrive(leftOutput, rightOutput);
    }

    public void arcadeDrive(double speed, double rotation) {
        drive.arcadeDrive(speed, rotation);
    }

    @Override
    public void periodic() { 
        SmartDashboard.putNumber("Yaw", getYaw());
        //SmartDashboard.putNumber("Roll", getRoll());
        SmartDashboard.putNumber("Pitch", getPitch());
        //SmartDashboard.putNumber("Rotations", rightEncoder.getPosition());
        //SmartDashboard.putNumber("TargetRotations", kPID_distanceRight.getSetpoint());
    }

    @Override
    public void simulationPeriodic() {
    }

}

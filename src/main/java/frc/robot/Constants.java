// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

public final class Constants {

  public static class Shooter {
    public static final int shooterSparkMax = 6;
  }

  public final class Serializer {
    public static final int serializerSparkMax = 5;
  }

  public static class Intake {
    public static final int intakeSparkMax = 9;
  }

  public static class Climber {
    public static final int climberID = 7;
    public static final int armLimitSwitch = 0;

  }

  public static class DifferentialConstants {
    //Spark IDs
    public static final int leftFrontSparkiD = 2;
    public static final int leftBackSparkiD = 1;
    public static final int rightFrontSparkiD = 3;
    public static final int rightBackSparkiD = 4;

    //Sensors
    public static final int GyroID = 8;

    //Positions
  
    //Ratios
    public static final double driveTrainRatio = (1/10.71);
    public static final double wheelCircumference = (Math.PI * 6);

  }

  /* Testing drive trajectory auto control
  public static class DriveAutoConstants {
    public static final double kTrackwidthMeters = 0.69;
    public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(kTrackwidthMeters);
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 1;
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;  
   

  }*/
}

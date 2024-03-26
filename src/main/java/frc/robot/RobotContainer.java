package frc.robot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.autos.OneNoteLeave;
import frc.robot.autos.ShootAuto;
import frc.robot.autos.ThreeNoteLeave;
import frc.robot.autos.TwoNoteLeaveBlue;
import frc.robot.autos.TwoNoteLeaveRed;
import frc.robot.commands.Intake;
import frc.robot.commands.LEDControl;
import frc.robot.commands.LimeDrive;
import frc.robot.commands.SpeakerShoot;
import frc.robot.commands.TeleOPDrive;
import frc.robot.commands.ClimbControl;
import frc.robot.subsystems.Serializer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ClimbArm;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intaker;
import frc.robot.subsystems.LED;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class RobotContainer {
  //subsystem
    public static DriveTrain m_driveTrain = new DriveTrain();
    public static Serializer m_serializer = new Serializer();
    public static Shooter m_shooter = new Shooter();
    public static Intaker m_intake = new Intaker();
    public static ClimbArm m_climbArm = new ClimbArm();
    public static LED m_LEDcontrol = new LED();
  
  //driver
    private final Joystick driver = new Joystick(0);
      private final int drivetrainSpeed = XboxController.Axis.kLeftY.value;
      private final int drivetrainRotation = XboxController.Axis.kLeftX.value;
      private final JoystickButton limeDriveButton = new JoystickButton(driver, XboxController.Button.kX.value);
      private final JoystickButton intakeButton = new JoystickButton(driver, XboxController.Button.kA.value);
      private final JoystickButton outtakeButtonDriver = new JoystickButton(driver, XboxController.Button.kB.value);
      private final JoystickButton setLEDredButton = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
      private final JoystickButton setLEDblueButton = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
  //operator
    public final Joystick operator = new Joystick(1);

      private final JoystickButton sourceIntakeButton = new JoystickButton(operator, XboxController.Button.kB.value);
      private final JoystickButton outtakeButtonOperator = new JoystickButton(operator, XboxController.Button.kA.value);
      private final JoystickButton ampShootButton = new JoystickButton(operator, XboxController.Button.kLeftBumper.value);
      private final JoystickButton speakerShootButton = new JoystickButton(operator, XboxController.Button.kRightBumper.value);
      private final JoystickButton climbLowerButton = new JoystickButton(operator, XboxController.Button.kX.value);
      private final JoystickButton climbRaiseButton = new JoystickButton(operator, XboxController.Button.kY.value);
    

  //auto
    SendableChooser<Command> autoChooser = new SendableChooser<>();

  //Factors
    public static double heading;
    public static double distance;

  public RobotContainer() {
     m_driveTrain.setDefaultCommand(
      new TeleOPDrive(
          m_driveTrain,
          () -> driver.getRawAxis(drivetrainSpeed),
          () -> driver.getRawAxis(drivetrainRotation)
      )
  );

  //autos
  autoChooser.setDefaultOption("OneNoteLeave", new OneNoteLeave(m_driveTrain, m_shooter, m_serializer, m_LEDcontrol, m_intake));
  autoChooser.addOption("TwoNoteAutoBlue", new TwoNoteLeaveBlue(m_driveTrain, m_shooter, m_serializer, m_LEDcontrol, m_intake));
  autoChooser.addOption("ThreeNoteAuto", new ThreeNoteLeave(m_driveTrain, m_shooter, m_serializer, m_LEDcontrol));
  autoChooser.addOption("TwoNoteAutoRed", new TwoNoteLeaveRed(m_driveTrain, m_shooter, m_serializer, m_LEDcontrol, m_intake));
  autoChooser.addOption("ShootAuto", new ShootAuto(m_shooter, m_serializer, m_LEDcontrol, m_intake));
  SmartDashboard.putData(autoChooser);
 
    configureBindings();
  }

  private void configureBindings() {
    //Driver
    limeDriveButton.whileTrue(new LimeDrive(m_driveTrain, ()-> driver.getRawAxis(drivetrainSpeed), ()-> driver.getRawAxis(drivetrainRotation)));

    intakeButton.whileTrue(new Intake(m_intake, -0.3));

    outtakeButtonDriver.whileTrue(new Intake(m_intake, 0.3));
   
    setLEDblueButton.whileTrue(new LEDControl(m_LEDcontrol, 0.87));
    setLEDredButton.whileTrue(new LEDControl(m_LEDcontrol, 0.61));

    //Operator
    sourceIntakeButton.whileTrue(new SpeakerShoot(m_shooter, 0.15, m_serializer, 0.15));

    outtakeButtonOperator.whileTrue(new Intake(m_intake, 0.3));

    ampShootButton.whileTrue(new ParallelDeadlineGroup(
      new WaitCommand(1),
      new SpeakerShoot(m_shooter, -0.09, m_serializer, -0.7))
      .andThen(
      new ParallelCommandGroup(
        new LEDControl(m_LEDcontrol, 0.77),
        new Intake(m_intake, -0.4))));

    speakerShootButton.whileTrue(new ParallelDeadlineGroup(
      new WaitCommand(1),
      new SpeakerShoot(m_shooter, -1.0, m_serializer, 0.0))
      .andThen(
      new ParallelCommandGroup(
        new LEDControl(m_LEDcontrol, 0.77),
        new SpeakerShoot(m_shooter, -1.0, m_serializer, -0.9),
        new Intake(m_intake, -0.3))));

    climbLowerButton.whileTrue(new ClimbControl(m_climbArm, 0.9));
    climbRaiseButton.whileTrue(new ClimbControl(m_climbArm, -0.9));
  }    
 
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}

package frc.robot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.autos.LeaveStudents;
import frc.robot.autos.OneNoteLeave;
import frc.robot.commands.TeleOPDrive;
//import frc.robot.commands.TestMotorRun;
import frc.robot.subsystems.DriveTrain;
//import frc.robot.subsystems.Test;

public class RobotContainer {


  //test
  //public static Test m_TestMotor = new Test();
  //subsystem
    public static DriveTrain m_driveTrain = new DriveTrain();

  //driver
    private final Joystick driver = new Joystick(0);

  //operator
    public final Joystick operator = new Joystick(1);

  //auto
    SendableChooser<Command> autoChooser = new SendableChooser<>();

  //Factors
    private final int drivetrainSpeed = XboxController.Axis.kLeftY.value;
    private final int drivetrainRotation = XboxController.Axis.kLeftX.value;
    public static double heading;
    public static double distance;

    //buttons
     private final JoystickButton testButton = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);

  public RobotContainer() {
     m_driveTrain.setDefaultCommand(
      new TeleOPDrive(
          m_driveTrain,
          () -> driver.getRawAxis(drivetrainSpeed),
          () -> driver.getRawAxis(drivetrainRotation)
      )
  );
  
  //autos
  autoChooser.setDefaultOption("OneNoteLeave", new OneNoteLeave(m_driveTrain));
  autoChooser.addOption("LeaveStudents", new LeaveStudents(m_driveTrain));
  SmartDashboard.putData(autoChooser);
 
    configureBindings();
  }

  private void configureBindings() {
    //testButton.whileTrue(new TestMotorRun(m_TestMotor, 0.4));
    //Driver

    //Operator
  }
 
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}

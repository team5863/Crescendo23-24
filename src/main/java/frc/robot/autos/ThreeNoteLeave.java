package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.DistanceDriveTank;
import frc.robot.commands.LEDControl;
import frc.robot.commands.LimeDrive;
import frc.robot.commands.SpeakerShoot;
import frc.robot.commands.TeleOPDrive;
import frc.robot.commands.TurnDrive;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Serializer;
import frc.robot.subsystems.Shooter;

public class ThreeNoteLeave extends SequentialCommandGroup{

    public ThreeNoteLeave(DriveTrain m_driveTrain, Shooter m_shooter, Serializer m_serializer, LED m_LEDcontrol){

        addCommands(

        new ParallelDeadlineGroup(
            new WaitCommand(3),
            new ParallelDeadlineGroup(
                new WaitCommand(1.5),
                new SpeakerShoot(m_shooter, -1.0, m_serializer, 0.0))
            .andThen(
                new ParallelCommandGroup(
                new LEDControl(m_LEDcontrol, 0.77),
                new SpeakerShoot(m_shooter, -1.0, m_serializer, -0.9))
            )
        ),
        
        
        new ParallelCommandGroup(
            new DistanceDriveTank(m_driveTrain, 6)
         ),

        new ParallelDeadlineGroup(
            new WaitCommand(2),
            new TeleOPDrive(m_driveTrain, () -> 0, () -> 0),
            new SpeakerShoot(m_shooter, 0.15, m_serializer, 0.15)
        ),

        new ParallelDeadlineGroup(
            new WaitCommand(4),
            new ParallelCommandGroup(
            new LimeDrive(m_driveTrain, () -> 0, () -> 0))
        ),

        new ParallelDeadlineGroup(
            new WaitCommand(3),
            new ParallelDeadlineGroup(
                new WaitCommand(1.5),
                new SpeakerShoot(m_shooter, -1.0, m_serializer, 0.0))
            .andThen(
                new ParallelCommandGroup(
                new LEDControl(m_LEDcontrol, 0.77),
                new SpeakerShoot(m_shooter, -1.0, m_serializer, -0.9))
            )
        ),


        new ParallelCommandGroup(
            new TurnDrive(m_driveTrain, -26)
        ),

        new ParallelDeadlineGroup(
            new WaitCommand(3),
            new ParallelCommandGroup(
                new DistanceDriveTank(m_driveTrain, 3)
            )
        ),
        new ParallelDeadlineGroup(
            new WaitCommand(3),
            new ParallelCommandGroup(
                new TurnDrive(m_driveTrain, -26)
            )
        ),


        new ParallelDeadlineGroup(
            new WaitCommand(2),
            new TeleOPDrive(m_driveTrain, () -> 0, () -> 0),
            new SpeakerShoot(m_shooter, 0.15, m_serializer, 0.15)
        ),


        new ParallelDeadlineGroup(
            new WaitCommand(4),
            new ParallelCommandGroup(
            new LimeDrive(m_driveTrain, () -> 0, () -> 0))
        ),

        new ParallelDeadlineGroup(
            new WaitCommand(3),
            new ParallelDeadlineGroup(
                new WaitCommand(1.5),
                new SpeakerShoot(m_shooter, -1.0, m_serializer, 0.0))
            .andThen(
                new ParallelCommandGroup(
                new LEDControl(m_LEDcontrol, 0.77),
                new SpeakerShoot(m_shooter, -1.0, m_serializer, -0.9))
            )
        )
    );

        
        

    }


    
}
    
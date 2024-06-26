package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.DistanceDriveTank;
import frc.robot.commands.Intake;
import frc.robot.commands.LEDControl;
import frc.robot.commands.SpeakerShoot;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intaker;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Serializer;
import frc.robot.subsystems.Shooter;

public class OneNoteLeave extends SequentialCommandGroup{

    public OneNoteLeave(DriveTrain m_driveTrain, Shooter m_shooter, Serializer m_serializer, LED m_LEDcontrol, Intaker m_intake){

        addCommands(

         new ParallelDeadlineGroup(
            new WaitCommand(2),
            new ParallelDeadlineGroup(
                new WaitCommand(1),
                new SpeakerShoot(m_shooter, -1.0, m_serializer, 0.0))
        .andThen(
            new ParallelCommandGroup(
                new LEDControl(m_LEDcontrol, 0.77),
                new SpeakerShoot(m_shooter, -1.0, m_serializer, -0.9),
                new Intake(m_intake, -0.3)
                )
            )
        ),
        
        new ParallelCommandGroup(
           
                new DistanceDriveTank(m_driveTrain, 9)
        )
    );

        
        

    }


    
}
    
package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.DistanceDriveTank;
import frc.robot.commands.TurnDrive;
import frc.robot.subsystems.DriveTrain;

public class OneNoteLeave extends SequentialCommandGroup{

    public OneNoteLeave(DriveTrain m_driveTrain){

        addCommands(

        new SequentialCommandGroup(
            new DistanceDriveTank(m_driveTrain, .5),
            new TurnDrive(m_driveTrain, 180),
            new DistanceDriveTank(m_driveTrain, .5)
        ),
            
        new ParallelCommandGroup(
            new WaitCommand(3)
        ),

        new SequentialCommandGroup(
             new DistanceDriveTank(m_driveTrain, -.5),
             new TurnDrive(m_driveTrain, -180),
             new DistanceDriveTank(m_driveTrain,12)
        ),

         new ParallelCommandGroup(
             new WaitCommand(2)
        )

        );
        

    }


    
}
    
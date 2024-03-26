package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intaker extends SubsystemBase {
    CANSparkMax intakeMotor;

    public Intaker() {
        intakeMotor = new CANSparkMax(Constants.Intake.intakeSparkMax, MotorType.kBrushless);
    }

    public void intake(double speed) {
        intakeMotor.set(speed);
    }

    @Override
    public void periodic() {

    }
}

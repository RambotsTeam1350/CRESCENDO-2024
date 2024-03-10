package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    private final CANSparkMax m_motor1;
    private final CANSparkMax m_motor2;

    public Intake() {
        this.m_motor1 = new CANSparkMax(Constants.Intake.MOTOR_1_ID, MotorType.kBrushless);
        this.m_motor2 = new CANSparkMax(Constants.Intake.MOTOR_2_ID, MotorType.kBrushless);
    }
}

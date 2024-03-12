package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    private final CANSparkMax m_intakeMotor;
    private final CANSparkMax m_rotationMotor;

    public Intake() {
        this.m_intakeMotor = new CANSparkMax(Constants.Intake.INTAKE_MOTOR_ID, MotorType.kBrushless);
        this.m_rotationMotor = new CANSparkMax(Constants.Intake.ROTATION_MOTOR_ID, MotorType.kBrushless);
    }
}

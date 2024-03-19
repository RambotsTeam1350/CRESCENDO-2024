package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drivers.uppermech.PIDCANSparkMax;
import frc.robot.constants.Constants;

public class Climber extends SubsystemBase {
    private final CANSparkMax leftMotor;
    private final CANSparkMax rightMotor;

    public Climber() {
        // it is IMPERATIVE that these are in brake mode, or the robot will fall to the
        // ground.
        this.leftMotor = new PIDCANSparkMax(Constants.Climber.LEFT_MOTOR_ID, MotorType.kBrushless, IdleMode.kBrake,
                false, Constants.Climber.LEFT_SPARK_PIDF_CONFIG);
        this.rightMotor = new PIDCANSparkMax(Constants.Climber.RIGHT_MOTOR_ID, MotorType.kBrushless, IdleMode.kBrake,
                false, Constants.Climber.RIGHT_SPARK_PIDF_CONFIG);
    }

    public void setVoltageSetpoint(double voltage) {
        this.leftMotor.getPIDController().setReference(voltage, ControlType.kVoltage);
        this.rightMotor.getPIDController().setReference(voltage, ControlType.kVoltage);
    }
}

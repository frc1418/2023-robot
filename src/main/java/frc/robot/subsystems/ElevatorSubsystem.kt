package frc.robot.subsystems

import com.revrobotics.CANSparkMax
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase

class ElevatorSubsystem(private val elevatorMotor: CANSparkMax) : SubsystemBase() {
    // TODO plug these limit switches into the spark max directly
    private val topLimitSwitch = DigitalInput(3)
    private val bottomLimitSwitch = DigitalInput(4)

    // TODO add encoder + hall effect sensor to elevator to allow for precise positioning
    init {
        defaultCommand = runOnce { setElevatorMotor(0.0) }
    }

    private fun setElevatorMotor(speed: Double) {
        elevatorMotor.set(speed)
    }

    // Prevent the motor from moving when the top limit switch is hit.
    // until() is only checked after running for 1 cycle, so we pair with unless()
    fun raiseElevatorCommand(): Command {
        return run { setElevatorMotor(1.0) }
            .until { topLimitSwitch.get() }
            .unless { topLimitSwitch.get() }
    }

    fun lowerElevatorCommand(): Command {
        return run { setElevatorMotor(-1.0) }
            .until { bottomLimitSwitch.get() }
            .unless { bottomLimitSwitch.get() }
    }
}

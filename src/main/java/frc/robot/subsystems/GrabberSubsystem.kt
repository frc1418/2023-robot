package frc.robot.subsystems

import edu.wpi.first.wpilibj.DoubleSolenoid
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase

class GrabberSubsystem(
    private val leftSolenoid: DoubleSolenoid,
    private val rightSolenoid: DoubleSolenoid
) : SubsystemBase() {
    init {
        openGrabber()
    }

    val release: Command
        get() = runOnce { openGrabber() }.andThen(Commands.waitSeconds(0.2))

    val grab: Command
        get() = runOnce { closeGrabber() }.andThen(Commands.waitSeconds(0.2))

    val toggle: Command
        get() = runOnce { toggleGrabber() }.andThen(Commands.waitSeconds(0.2))

    private fun closeGrabber() {
        leftSolenoid.set(DoubleSolenoid.Value.kReverse)
        rightSolenoid.set(DoubleSolenoid.Value.kReverse)
    }

    private fun openGrabber() {
        leftSolenoid.set(DoubleSolenoid.Value.kForward)
        rightSolenoid.set(DoubleSolenoid.Value.kForward)
    }

    private fun toggleGrabber() {
        leftSolenoid.toggle()
        rightSolenoid.toggle()
    }

    val isOpen: Boolean
        get() = leftSolenoid.get() == DoubleSolenoid.Value.kForward
}

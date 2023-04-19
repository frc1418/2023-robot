package frc.robot.subsystems

import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants.LimelightDirections

class LimelightSubsystem : SubsystemBase() {
    private val table = NetworkTableInstance.getDefault().getTable("limelight")
    private val ntRobotposeTargetspace = table.getEntry("botpose_targetspace")

    // returns 0 if no target, 1, if target
    private val ntIsDetecting = table.getEntry("tv")
    private val ntID = table.getEntry("tid")
    var xDistance = 0.0
        private set
    var yDistance = 0.0
        private set
    var targetRotation = LimelightDirections.SUBSTATION_SIDE
        private set

    override fun periodic() {
        val botPosArray = ntRobotposeTargetspace.getDoubleArray(DoubleArray(6))
        xDistance = botPosArray[0]
        yDistance = botPosArray[2]
        val ntIdValue = ntID.getDouble(0.0)
        if (DriverStation.getAlliance() == Alliance.Blue) {
            if (ntIdValue == 6.0 || ntIdValue == 7.0 || ntIdValue == 8.0) {
                targetRotation = LimelightDirections.GRID_SIDE
            } else if (ntIdValue == 4.0) {
                targetRotation = LimelightDirections.SUBSTATION_SIDE
            }
        } else {
            if (ntIdValue == 3.0 || ntIdValue == 2.0 || ntIdValue == 1.0) {
                targetRotation = LimelightDirections.GRID_SIDE
            } else if (ntID.getDouble(0.0) == 5.0) {
                targetRotation = LimelightDirections.SUBSTATION_SIDE
            }
        }
    }

    val isDetecting: Boolean
        get() {
            val id = ntID.getInteger(Int.MAX_VALUE.toLong()).toInt()
            return ntIsDetecting.getInteger(0) == 1L && id >= 1 && id <= 8
        }
}

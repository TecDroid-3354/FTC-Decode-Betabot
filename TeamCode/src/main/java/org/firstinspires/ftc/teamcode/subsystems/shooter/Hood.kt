package org.firstinspires.ftc.teamcode.subsystems.shooter

import Angle
import com.qualcomm.robotcore.hardware.HardwareMap
import com.seattlesolvers.solverslib.command.SubsystemBase
import com.seattlesolvers.solverslib.hardware.ServoEx
import com.seattlesolvers.solverslib.util.MathUtils
import org.firstinspires.ftc.robotcore.external.Telemetry

/**
 * Class intended to control the Hood Subsystem, which is mounted in the shooter.
 * @param hardwareMap contains the hardware devices of the robot. Configured through Driver Hub.
 * @param telemetry to print important data in real-time through Driver Hub.
 */
@Suppress("JoinDeclarationAndAssignment")
class Hood(val hardwareMap: HardwareMap, val telemetry: Telemetry): SubsystemBase() {
    // Servo controlling the hood. Is a SWYFT servo, check config with SWYFT servo programmer.
    private val servo: ServoEx
    // As SWYFT servos do not store their position, we store the commanded angle (Telemetry purposes)
    var currentAngle: Angle

    // Initialization code //
    init {
        servo = ServoEx(hardwareMap, HoodConstants.Identification.hoodId)
        servoConfig()
        setHoodPosition(HoodConstants.Positions.homePosition)

        // servo.position returns a value from 0.0 to 1.0, we take it as rotations.
        currentAngle = Angle.fromRotations(servo.servo.position)
    }

    // Code called every robot loop //
    override fun periodic() {
        // Telemetry to retrieve useful data
        telemetry.addData("HoodPositionRotations", currentAngle.rotations)
        currentAngle = Angle.fromRotations(servo.servo.position)
    }

    // Sets the desired angle to the servo (in radians) and updates the currentAngle variable //
    fun setHoodPosition(position: Angle) {
        val clampedPosition = MathUtils.clamp(position.rotations, HoodConstants.Positions.minPosition.rotations,
            HoodConstants.Positions.homePosition.rotations)
        servo.set(clampedPosition) // Per documentation, servo.set() requires radians
    }

    fun setHoodPosition(position: Double) {
        val clampedPosition = MathUtils.clamp(position, HoodConstants.Positions.minPosition.rotations,
            HoodConstants.Positions.homePosition.rotations)
        servo.set(clampedPosition) // Per documentation, servo.set() requires radians
    }

    fun modifyCurrentPositionBy(factor: Angle) {
        setHoodPosition(Angle.fromRotations(servo.servo.position) + factor)
    }

    // Setup code //
    private fun servoConfig() {
        // Configures whether to invert the servo direction
        servo.inverted = HoodConstants.Configuration.hoodServoInverted
    }
}
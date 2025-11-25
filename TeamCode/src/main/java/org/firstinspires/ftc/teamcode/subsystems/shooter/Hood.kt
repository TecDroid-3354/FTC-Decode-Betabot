package org.firstinspires.ftc.teamcode.subsystems.shooter

import Angle
import com.qualcomm.robotcore.hardware.HardwareMap
import com.seattlesolvers.solverslib.command.SubsystemBase
import com.seattlesolvers.solverslib.hardware.ServoEx
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
    private var currentAngle: Angle

    // Initialization code //
    init {
        servo = ServoEx(hardwareMap, hardwareMap.get(HoodConstants.Identification.hoodId).deviceName)
        servoConfig()

        // servo.position returns a value from 0.0 to 1.0, we take it as rotations.
        currentAngle = Angle.fromRotations(servo.servo.position)
    }

    // Code called every robot loop //
    override fun periodic() {
        // Telemetry to retrieve useful data
        telemetry.addData("HoodPositionDegrees", currentAngle.degrees)
    }

    // Sets the desired angle to the servo (in radians) and updates the currentAngle variable //
    fun setHoodAngle(angle: Angle) {
        servo.set(angle.radians) // Per documentation, servo.set() requires radians.
        currentAngle = angle
    }

    // Setup code //
    private fun servoConfig() {
        // Configures whether to invert the servo direction
        servo.inverted = HoodConstants.Configuration.hoodServoInverted
    }
}
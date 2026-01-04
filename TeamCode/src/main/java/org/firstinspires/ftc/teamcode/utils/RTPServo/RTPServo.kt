package org.firstinspires.ftc.teamcode.utils.RTPServo

import Angle
import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.PIDFCoefficients
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeDegrees
import org.firstinspires.ftc.teamcode.utils.controllers.PIDFAngleController
import kotlin.math.max
import kotlin.math.min

data class RTPServoConfig(
    val servoId: String,
    val absoluteId: String,
    val direction: RTPServo.Direction,
    val encoderOffset: Angle,
    val maxPower: Double = 1.0,
    val pidfCoefficients: PIDFCoefficients
)

@Suppress("JoinDeclarationAndAssignment")
class RTPServo(hw: HardwareMap, val telemetry: Telemetry, val config: RTPServoConfig) {

    enum class Direction {
        FORWARD, REVERSE;
    }

    private var servo: CRServo
    var servoEncoder: AnalogInput

    private val pidfController = PIDFAngleController(config.pidfCoefficients)

    init {
        /* INITIALIZATION CODE */

        // Initialize both the CR servo and absolute encoder
        servo = hw.get(CRServo::class.java, config.servoId)
        servoEncoder = hw.get(AnalogInput::class.java, config.absoluteId)

        pidfController.setTolerance(2.0)

        setPower(0.0)
    }

    fun setPower(output: Double) {
        val power = max(-config.maxPower, min(config.maxPower, output))
        servo.power = power * (if (config.direction == Direction.REVERSE) -1 else 1)
    }

    fun stop() {
        setPower(0.0)
    }

    // Sets an absolute angle target to the servo, needs to be called in a loop for PIDF Feedback
    fun setTargetAngle(target: Angle) {
        val normalizedAngle: Angle = Angle.fromDegrees(normalizeDegrees(target.degrees))

        val output = pidfController.calculate(getAbsoluteAngle().degrees, normalizedAngle.degrees)

        setPower(output)
    }

    // Gets the current angle and adds the desired change in it
    fun changeTargetAngle(change: Angle) {
        setTargetAngle(getAbsoluteAngle() + change)
    }

    // Gets the absolute position considering gear ratios
    fun getAbsoluteAngle(): Angle {
        val currentAngle = Angle.fromDegrees(
            (servoEncoder.voltage / 3.2) * (if (config.direction == Direction.REVERSE) -360 else 360)
        )
        val transformedAngle = Angle.fromDegrees(
            (currentAngle.degrees - config.encoderOffset.degrees)
        )

        return Angle.fromDegrees(normalizeDegrees(transformedAngle.degrees))
    }

    fun isAtSetPoint(): Boolean {
        return pidfController.atSetPoint()
    }

    fun setPIDFTolerance(tolerance: Angle) {
        pidfController.setTolerance(tolerance.degrees)
    }


    fun setPIDF(pidfCoefficients: PIDFCoefficients) {
        pidfController.setPIDF(
            pidfCoefficients.p, pidfCoefficients.i,
            pidfCoefficients.d, pidfCoefficients.f
        )
    }
}
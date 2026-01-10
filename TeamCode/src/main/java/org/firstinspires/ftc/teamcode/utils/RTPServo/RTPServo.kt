package org.firstinspires.ftc.teamcode.utils.RTPServo

import Angle
import Voltage
import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.PIDFCoefficients
import com.seattlesolvers.solverslib.controller.PIDFController
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeDegrees
import org.firstinspires.ftc.teamcode.utils.controllers.PIDFAngleController
import kotlin.math.max
import kotlin.math.min

data class RTPServoConfig(
    val servoId: String,
    val absoluteId: String,
    val absoluteMaxVoltage: Voltage,
    val direction: RTPServo.Direction,
    val encoderOffset: Angle,
    val maxPower: Double = 1.0,
    val gearRatio: Double = 1.0,
    val pidfCoefficients: PIDFCoefficients
)

@Suppress("JoinDeclarationAndAssignment")
class RTPServo(hw: HardwareMap, val telemetry: Telemetry, val config: RTPServoConfig) {

    enum class Direction {
        FORWARD, REVERSE;
    }

    private var servo: CRServo
    private var servoEncoder: AnalogInput

    private var totalRotation: Angle = Angle.fromDegrees(0.0)
    private var previousAngle: Angle = Angle.fromDegrees(0.0)
    private var targetRotation: Angle = Angle.fromDegrees(0.0)
    private var fullRotations = 0

    private val pidfController = PIDFController(config.pidfCoefficients)

    init {
        /* INITIALIZATION CODE */

        // Initialize both the CR servo and absolute encoder
        servo = hw.get(CRServo::class.java, config.servoId)
        servoEncoder = hw.get(AnalogInput::class.java, config.absoluteId)

        previousAngle = getAbsoluteAngle()

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

    fun setTargetAngle(target: Angle) {
        targetRotation = target
        pidfController.clearTotalError()
    }

    // Gets the current angle and adds the desired change in it
    fun changeTargetAngle(change: Angle) {
        setTargetAngle(totalRotation + change)
    }

    // Gets the absolute position of the servo, not considering gear ratios
    fun getAbsoluteAngle(): Angle {
        val currentAngle = Angle.fromDegrees(
            (servoEncoder.voltage / config.absoluteMaxVoltage.volts) * (if (config.direction == Direction.REVERSE) -360 else 360)
        )
        val transformedAngle = Angle.fromDegrees(
            (currentAngle.degrees - config.encoderOffset.degrees)
        )

        return Angle.fromDegrees(normalizeDegrees(transformedAngle.degrees))
    }

    fun getTotalRotation(): Angle {
        return totalRotation
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

    fun update() {
        val currentAngle = getAbsoluteAngle()
        val angleDifference = Angle.fromDegrees(currentAngle.degrees - previousAngle.degrees)

        if (angleDifference.degrees > Angle.fromDegrees(180.0).degrees) {
            fullRotations--
        } else if (angleDifference.degrees < Angle.fromDegrees(-180.0).degrees) {
            fullRotations++
        }

        totalRotation = getAbsoluteAngle() + Angle.fromDegrees(fullRotations * 360.0)
        previousAngle = currentAngle

        telemetry.addData("Total Rotation", totalRotation.degrees)
        telemetry.addData("Full rotations", fullRotations)
        telemetry.addData("Current Angle", currentAngle.degrees)
        telemetry.addData("Angle Difference", angleDifference.degrees)

        val output = pidfController.calculate(totalRotation.degrees, targetRotation.degrees)

        if (isAtSetPoint().not()) {
            setPower(output)
        } else {
            stop()
        }
    }
}
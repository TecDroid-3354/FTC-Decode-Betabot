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
import org.firstinspires.ftc.teamcode.utils.controllers.PIDFCircularController
import kotlin.math.max
import kotlin.math.min

data class RTPServoConfig(
    val servoId: String,
    val absoluteId: String = "",
    val absoluteMaxVoltage: Voltage,
    val direction: RTPServo.Direction,
    val encoderOffset: Angle,
    val maxPower: Double = 1.0,
    val gearRatio: Double = 1.0,
    val pidfCoefficients: PIDFCoefficients
)

@Suppress("JoinDeclarationAndAssignment")
class RTPServo(hw: HardwareMap, val telemetry: Telemetry, val config: RTPServoConfig, absoluteEncoder: AnalogInput) {

    /**
     *[Direction] used for setting a servo direction
     */
    enum class Direction {
        FORWARD, REVERSE;
    }

    // Creating the servo
    private var servo: CRServo

    // Creating the absolute encoder
    private var servoEncoder: AnalogInput? = null

    // Keeps track of the total servo rotation without considering gear ratio
    private var totalRotation: Angle = Angle.fromDegrees(0.0)
    // Keeps track of a previous angle in order to perform a subtraction in each iteration
    private var previousAngle: Angle = Angle.fromDegrees(0.0)
    // Used for setting a target rotation
    private var targetRotation: Angle = Angle.fromDegrees(0.0)
    // Keeps track of the number of full rotations in order to obtain the correct servo position along time
    private var fullRotations = 0

    // The turret controller
    private val pidfController = PIDFController(config.pidfCoefficients)

    init {
        /* INITIALIZATION CODE */

        // Initialize both the CR servo and absolute encoder
        servo = hw.get(CRServo::class.java, config.servoId)
        this.servoEncoder = absoluteEncoder

        // Getting the previous angle reading
        previousAngle = getServoAbsoluteAngle()

        // Setting a default position tolerance
        pidfController.setTolerance(1.0)

        // Must call servo.setPower() for correct servo working
        setPower(0.0)
    }

    constructor(hw: HardwareMap, telemetry: Telemetry, config: RTPServoConfig):
            this(hw, telemetry, config, hw.get(AnalogInput::class.java, config.absoluteId))

    /**
     * Sets an output considering the maximum power set in the servo's configuration
     * @param output the desired output from - 1 to 1
     */
    fun setPower(output: Double) {
        val power = max(-config.maxPower, min(config.maxPower, output))
        servo.power = power * (if (config.direction == Direction.REVERSE) -1 else 1)
    }

    /**
     * Completely stops the servo's movement
     */
    fun stop() {
        setPower(0.0)
    }

    /**
     * Sets a target angle and reassigns the [targetRotation] value so it can be called in [periodic]
     * Clears the PIDF total error for better position tracking
     * @param target the desired target angle
     */
    fun setTargetAngle(target: Angle) {
        targetRotation = target / config.gearRatio
        pidfController.clearTotalError()
    }

    /**
     * Changes the [targetRotation] considering gear ratios and the requested [change]
     * For example, if a negative 360 deg rotation is needed, the [changeTargetAngle] method does the job for getting
     * the target in terms of the servo.
     * @param change the desired change in the target rotation
     */
    fun changeTargetAngle(change: Angle) {
        targetRotation += (change / config.gearRatio)
    }

    /**
     * Manually sets a PIDF position tolerance
     * @param tolerance the desired position tolerance
     */
    fun setPIDFTolerance(tolerance: Angle) {
        pidfController.setTolerance(tolerance.degrees)
    }

    /**
     * Manually sets a new [PIDFCoefficients] to the servo's controller
     */
    fun setPIDF(pidfCoefficients: PIDFCoefficients) {
        pidfController.setPIDF(
            pidfCoefficients.p, pidfCoefficients.i,
            pidfCoefficients.d, pidfCoefficients.f
        )
    }

    /**
     * Gets the absolute position of the servo, not considering gear ratios
     */
    private fun getServoAbsoluteAngle(): Angle {
        val currentAngle = Angle.fromDegrees(
            (servoEncoder!!.voltage / config.absoluteMaxVoltage.volts) * (if (config.direction == Direction.REVERSE) -360 else 360)
        )
        val transformedAngle = Angle.fromDegrees(
            (currentAngle.degrees - config.encoderOffset.degrees)
        )

        return Angle.fromDegrees(normalizeDegrees(transformedAngle.degrees))
    }

    /**
     * Based on how many full rotations the servo has achieved, it returns an absolute angle
     * @return the total rotation the servo has achieved while considering gear ratios.
     */
    fun getAngle(): Angle {
        val transformedAngle = totalRotation.degrees * config.gearRatio
        val normalizedAngle = normalizeDegrees(transformedAngle)
        return Angle.fromDegrees(normalizedAngle)
    }

    /**
     * @return true is PID is at set point, false if not
     */
    fun isAtSetPoint(): Boolean {
        return pidfController.atSetPoint()
    }

    /**
     * Logs the servo's useful data
     */
    fun log() {
        // Servo absolute angle -180° to 180°
        telemetry.addData("Current Servo Absolute Angle", getServoAbsoluteAngle().degrees)
        // Total of rotations the servo has achieved
        telemetry.addData("Full rotations", fullRotations)
        // Encoder's voltage (useful when debugging)
        telemetry.addData("Encoder Voltage", servoEncoder?.voltage)
        // If the servo is at set point
        telemetry.addData("Is at set point", isAtSetPoint())
        // System's angle (considering gear ratios)
        telemetry.addData("Absolute Angle (considering gear ratios)", getAngle().degrees)
    }

    /**
     * Updates the servo's PID, and calculates the achieved full rotations
     * Necessary for the servo's correct functioning
     */
    fun periodic() {
        // Retrieves the current servo angle
        val currentAngle = getServoAbsoluteAngle()
        // Calculates the difference between the current and past servo angle
        val angleDifference = Angle.fromDegrees(currentAngle.degrees - previousAngle.degrees)

        // Calculates whether the servo has achieved one rotation, when the angle difference is below or above
        // 360 degrees and updates the full rotations
        if (angleDifference.degrees > Angle.fromDegrees(180.0).degrees) {
            fullRotations--
        } else if (angleDifference.degrees < Angle.fromDegrees(-180.0).degrees) {
            fullRotations++
        }

        // Calculates the total rotation considering the absolute angle and the achieved rotations
        totalRotation = currentAngle + Angle.fromDegrees(fullRotations * 360.0)

        // keeps track of the previous servo angle
        previousAngle = currentAngle

        // PID control
        val output = pidfController.calculate(totalRotation.degrees, targetRotation.degrees)

        // If at set point, no power is requested.
        if (isAtSetPoint().not()) {
            setPower(output)
        }
    }
}
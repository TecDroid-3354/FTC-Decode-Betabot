package org.firstinspires.ftc.teamcode.utils.positionMotorEx

import Angle
import AngularVelocity
import Distance
import com.qualcomm.robotcore.hardware.PIDFCoefficients
import com.seattlesolvers.solverslib.controller.PIDFController
import com.seattlesolvers.solverslib.hardware.motors.Motor
import kotlin.math.abs

// Custom class to declare motors that take positions
class PositionMotorEx(
    private val motor: Motor,
    override var config: PositionMotorExConfig
) : IPositionMotorEx {

    // Defining a few variables
    private var lastPower = 0.0

    // Setting up the PID controller that will manage the position-achieving
    val pidfController: PIDFController = PIDFController(config.pidfCoefficients)


    /* ! CONFIG METHODS ! */

    init {
        applyConfig()
    }

    // The following functions, applyConfig() allow us to apply a custom configuration in case of needed
    override fun applyConfig() {
        // Setting up the motor & encoder default behavior
        motor.setZeroPowerBehavior(config.zeroPowerBehavior)
        motor.encoder.setDirection(config.direction)
        motor.resetEncoder()

        // Arranging PIDs
        setPIDFCoefficients(config.pidfCoefficients)
    }

    override fun applyConfig(config: PositionMotorExConfig) {
        // This method takes a custom configuration and applies it
        this.config = config
        applyConfig()
    }


    /* ! FUNCTIONAL METHODS ! */

    // Sets raw power using a number between [-1.0, 1.0]
    override fun setPower(power: Double) {
        // Basically, a condition that evaluates that the power passed isn't too low or equal
        // to the past power used
        if ((abs(this.lastPower - power) > config.powerThreshold) || (power == 0.0 && lastPower != 0.0)) {
            lastPower = power
            motor.set(power)
        }
    }

    // The following two methods allow us to set a position based on a given angle according to the parameters
    override fun setPosition(setPoint: Angle) {
        pidfController.setPoint = setPoint.rotations

        // Closed PID looped control
        while (!pidfController.atSetPoint()) {
            val voltage = pidfController.calculate(getPosition().rotations)
            setPower(voltage)
        }

        stopMotor()
    }

    override fun setPosition(distance: Distance, circumference: Distance) {
        // Transforms the distance to rotations
        val rotations = distance.meters / circumference.meters
        setPosition(Angle.fromRotations(rotations))
    }

    // Stops the motor using two instructions
    override fun stopMotor() {
        motor.set(0.0)
    }


    /* ! SETTER METHODS ! */

    // This method takes a custom gearRatio & applies it
    override fun setGearRatio(gearRatio: Double) {
        config.gearRatio = gearRatio
    }

    // Takes custom PID Coefficients and applies them
    override fun setPIDFCoefficients(pidfCoefficients: PIDFCoefficients) {
        // Setting independent PIDs
        pidfController.setPIDF(
            pidfCoefficients.p,
            pidfCoefficients.i,
            pidfCoefficients.d,
            pidfCoefficients.f)
    }

    // Changes the direction of a motor
    override fun setDirection(direction: Motor.Direction) {
        motor.encoder.setDirection(direction)
    }

    // The following two functions change the PID tolerance according to the given parameters
    override fun setPidfTolerance(tolerance: Angle) {
        pidfController.setTolerance(tolerance.rotations)
    }

    override fun setPidfTolerance(tolerance: Distance, circumference: Distance) {
        val rotations = tolerance.meters / circumference.meters
        setPidfTolerance(Angle.fromRotations(rotations))
    }

    // Sets the run mode
    override fun setMode(mode: Motor.RunMode) = motor.setRunMode(mode)


    /* ! GETTER METHODS ! */

    // Returns the current position
    override fun getPosition(): Angle =
        Angle.fromRotations(motor.currentPosition / config.ticksPerRevolution * config.gearRatio)

    // Returns the current velocity
    override fun getVelocity(): AngularVelocity =
        AngularVelocity.fromRps(motor.get() / config.ticksPerRevolution * config.gearRatio)
}
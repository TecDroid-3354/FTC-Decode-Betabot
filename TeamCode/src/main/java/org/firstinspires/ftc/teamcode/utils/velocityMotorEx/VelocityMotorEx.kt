package org.firstinspires.ftc.teamcode.utils.velocityMotorEx

import Angle
import AngularVelocity
import Distance
import LinearVelocity
import com.seattlesolvers.solverslib.controller.PIDFController
import com.seattlesolvers.solverslib.hardware.motors.Motor
import kotlin.math.abs

// Custom class to declare custom PID motors that take velocities
class VelocityMotorEx(
    private val motor: Motor,
    override var config: VelocityMotorConfig
) : IVelocityMotorEx {

    // Defining a few variables
    private var lastPower = 0.0
    private var wheelCircumference = Distance.fromCm(7.5) //todo: check placeholder value


    /* ! CONFIG METHODS ! */

    init {
        applyConfig()
    }

    // The following functions, applyConfig()
    override fun applyConfig() {
        // Setting up the motor & encoder default behavior
        motor.setZeroPowerBehavior(config.zeroPowerBehavior)
        motor.motor.direction = config.direction

        // Arranging velocity PIDs
        val coefficients = config.pidfCoefficients
        motor.setVeloCoefficients(coefficients.p, coefficients.i, coefficients.d)
    }

    override fun applyConfig(config: VelocityMotorConfig) {
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

    // The following three methods declare a velocity according to the given params
    override fun setVelocity(angularVelocity: AngularVelocity) {
        // with FTCLib: motor.velocity = angularVelocity.rps * config.ticksPerRevolution * gearRatio
        motor.set(angularVelocity.rps * config.ticksPerRevolution * config.gearRatio)
    }

    override fun setVelocity(linearVelocity: LinearVelocity) {
        val rps = linearVelocity.mps / wheelCircumference.meters
        setVelocity(AngularVelocity.fromRps(rps))
    }

    // Takes into consideration a distance in the form of circumference
    // This is especially useful for subsytems like sliders, which move a certain distance
    // based on how much their pulley-motor system has moved
    override fun setVelocity(linearVelocity: LinearVelocity, circumference: Distance) {
        val rps = linearVelocity.mps / circumference.meters
        setVelocity(AngularVelocity.fromRps(rps))
    }

    // Stops the motor using two instructions
    override fun stopMotor() {
        motor.set(0.0)
        motor.stopMotor()
    }


    /* ! SETTER METHODS ! */

    override fun setGearRatio(gearRatio: Double) {
        config.gearRatio = gearRatio
    }

    override fun setPIDFCoefficients(pidfCoefficients: PIDFController) {
        // TODO: Consider F
        val coefficients = config.pidfCoefficients
        motor.setVeloCoefficients(coefficients.p, coefficients.i, coefficients.d)
    }

    // Sets the direction of the motor
    override fun setDirection(direction: Motor.Direction) {
        motor.encoder.setDirection(direction)
    }

    override fun setCircumference(circumference: Distance) {
        wheelCircumference = circumference
    }

    // Sets the run mode
    override fun setMode(mode: Motor.RunMode) = motor.setRunMode(mode)


    /* ! GETTER METHODS ! */

    // Returns a position in degrees. Takes into consideration reductions + ticksPerRev
    override fun getPosition(): Angle =
        Angle.fromRotations(motor.currentPosition / config.ticksPerRevolution * config.gearRatio)

    // Returns the current velocity
    override fun getVelocity(): AngularVelocity = AngularVelocity.fromRps(motor.get() / config.ticksPerRevolution * config.gearRatio)

    override fun getLinearVelocity(): LinearVelocity =
        LinearVelocity.fromMps(wheelCircumference.meters * (motor.get() / config.ticksPerRevolution * config.gearRatio))

    override fun getLinearVelocity(circumference: Distance): LinearVelocity =
        LinearVelocity.fromMps(circumference.meters * (motor.get() / config.ticksPerRevolution * config.gearRatio))


}
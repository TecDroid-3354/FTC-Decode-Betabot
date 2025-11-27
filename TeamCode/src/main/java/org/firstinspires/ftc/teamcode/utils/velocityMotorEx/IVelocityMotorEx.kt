package org.firstinspires.ftc.teamcode.utils.velocityMotorEx

import Angle
import AngularVelocity
import Distance
import LinearVelocity
import com.seattlesolvers.solverslib.controller.PIDFController
import com.seattlesolvers.solverslib.hardware.motors.Motor

data class VelocityMotorConfig(
    val zeroPowerBehavior: Motor.ZeroPowerBehavior = Motor.ZeroPowerBehavior.FLOAT,
    val direction: Motor.Direction = Motor.Direction.FORWARD,
    val ticksPerRevolution: Double = 1.0,
    val pidfCoefficients: PIDFController,
    var gearRatio: Double = 1.0,
    val powerThreshold: Double = 0.01,
)

data class SVACoefficients(
    val kS: Double,
    val kV: Double,
    val kA: Double
)

interface IVelocityMotorEx {
    /* ! CONFIG METHODS ! */
    var config: VelocityMotorConfig
    fun applyConfig()
    fun applyConfig(config: VelocityMotorConfig)

    /* ! FUNCTIONAL METHODS ! */
    fun setPower(power: Double)
    fun setVelocity(angularVelocity: AngularVelocity)
    fun setVelocity(linearVelocity: LinearVelocity)
    fun setVelocity(linearVelocity: LinearVelocity, circumference: Distance)
    fun stopMotor()

    /* ! SETTER METHODS ! */
    fun setGearRatio(gearRatio: Double)
    fun setPIDFCoefficients(pidfCoefficients: PIDFController)
    fun setDirection(direction: Motor.Direction)
    fun setCircumference(circumference: Distance)
    fun setMode(mode: Motor.RunMode)
    fun setInverted(isInverted: Boolean)

    /* ! GETTER METHODS ! */
    fun getPosition(): Angle
    fun getVelocity(): AngularVelocity
    fun getLinearVelocity(): LinearVelocity
    fun getLinearVelocity(circumference: Distance): LinearVelocity
}
package org.firstinspires.ftc.teamcode.subsystems.turret

import Angle
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.PIDCoefficients
import com.qualcomm.robotcore.hardware.PIDFCoefficients
import com.seattlesolvers.solverslib.command.SubsystemBase
import com.seattlesolvers.solverslib.controller.PIDController
import com.seattlesolvers.solverslib.controller.PIDFController
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.utils.RTPServo.RTPServo
import org.firstinspires.ftc.teamcode.utils.RTPServo.RTPServoConfig
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeDegrees
import org.firstinspires.ftc.teamcode.axonTurretTest.AxonTurretConstants.Companion

data class AxonTurretConfig(
    val rightServoConfig: RTPServoConfig,
    val leftServoConfig: RTPServoConfig,
    val limits: ClosedFloatingPointRange<Double>,
    val pidCoefficients: PIDFCoefficients
)

class AxonTurret(val hw: HardwareMap, val telemetry: Telemetry, val config: AxonTurretConfig): SubsystemBase() {

    lateinit var rightServo: RTPServo
   // lateinit var leftServo: RTPServo

    val pidController: PIDFController = PIDFController(config.pidCoefficients.p, config.pidCoefficients.i, config.pidCoefficients.d, config.pidCoefficients.f)

    init {
        servoConfig()
    }

    override fun periodic() {
        telemetry.addData("Absolute reading", rightServo.getAbsoluteAngle().degrees)
        pidController.setPIDF(Companion.pidCoefficients.p, Companion.pidCoefficients.i, Companion.pidCoefficients.d,
            Companion.pidCoefficients.f)
    }

    private fun setPower(servo: RTPServo, output: Double = 1.0) {
        servo.getServo().power = output
    }

    fun stopTurret() {
        setPower(rightServo, 0.0)
        //setPower(leftServo, 0.0)
    }

    fun setTurretPower(output: Double) {
        when {
            getAbsoluteAngle().degrees in config.limits -> {
                setPower(rightServo, output)
             //   setPower(leftServo, output)
            }
            else -> stopTurret()
        }
    }

    // Needs to be called inside the opMode loop in order to correctly update target
    fun alignToTarget(target: Angle) {
        // TODO Need to test if this actually works
        val normalizedAngle: Angle = Angle.fromDegrees(normalizeDegrees(target.degrees))

        val output = pidController.calculate(getAbsoluteAngle().degrees, normalizedAngle.degrees)

        setTurretPower(output)

        // TODO Try tuning extremely well the servo's PID in case option one does not work
        //return InstantCommand({ rightServo.setTargetRotation(target) })
    }

    // Just need one encoder's reading
    fun getAbsoluteAngle(): Angle {
        return rightServo.getAbsoluteAngle()
    }

    fun update() {
        rightServo.update()
        //leftServo.update()
    }

    fun servoConfig() {
        rightServo = RTPServo(hw, telemetry, config.rightServoConfig)
        //leftServo = RTPServo(hw, telemetry, config.leftServoConfig)
    }
}
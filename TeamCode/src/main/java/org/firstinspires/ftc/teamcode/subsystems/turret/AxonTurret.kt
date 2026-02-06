package org.firstinspires.ftc.teamcode.subsystems.turret

import Angle
import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.PIDFCoefficients
import com.seattlesolvers.solverslib.command.Command
import com.seattlesolvers.solverslib.command.CommandBase
import com.seattlesolvers.solverslib.command.InstantCommand
import com.seattlesolvers.solverslib.command.RunCommand
import com.seattlesolvers.solverslib.command.SubsystemBase
import com.seattlesolvers.solverslib.util.MathUtils
import org.firstinspires.ftc.robotcore.external.Supplier
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.utils.RTPServo.RTPServo
import org.firstinspires.ftc.teamcode.utils.RTPServo.RTPServoConfig

data class AxonTurretConfig(
    val rightServoConfig: RTPServoConfig,
    val leftServoConfig: RTPServoConfig,
    val limits: Limits
)

class AxonTurret(val hw: HardwareMap, val telemetry: Telemetry, val config: AxonTurretConfig): SubsystemBase() {

    lateinit var rightServo: RTPServo
    lateinit var leftServo: RTPServo
    lateinit var absoluteEncoder: AnalogInput

    init {
        servoConfig()
    }

    override fun periodic() {
        telemetry.addData("Turret current angle", getAbsoluteAngle().degrees)
        setPIDFCoefficients(rightServo, AxonTurretConstants.turretControllerCoefficients)
        setPIDFCoefficients(leftServo, AxonTurretConstants.turretControllerCoefficients)
        rightServo.periodic()
        leftServo.periodic()
    }

    fun stopTurret(): Command {
        return InstantCommand({
            rightServo.stop()
            leftServo.stop()
        })
    }

    // Needs to be called inside a loop in order to correctly update target
    fun setTurretAngle(target: Supplier<Angle>): Command {
        return RunCommand({
            val clampedAngle = MathUtils.clamp(target.get().degrees, config.limits.minVal.degrees, config.limits.maxVal.degrees)

            rightServo.setTargetAngle(Angle.fromDegrees(clampedAngle))
            leftServo.setTargetAngle(Angle.fromDegrees(clampedAngle))

            if (getAbsoluteAngle().degrees !in config.limits.minVal.degrees..config.limits.maxVal.degrees) {
                stopTurret()
            }
        }, this)
    }

    // Just need one encoder's reading
    fun getAbsoluteAngle(): Angle {
        return rightServo.getAngle()
    }

    fun setPIDFCoefficients(servo: RTPServo, pidfCoefficients: PIDFCoefficients) {
        servo.setPIDF(pidfCoefficients)
    }

    fun servoConfig() {
        // Encoder initialization
        absoluteEncoder = hw.get(AnalogInput::class.java, "abs")

        /* SERVO INITIALIZATION */
        rightServo = RTPServo(hw, telemetry, config.rightServoConfig, absoluteEncoder)
        rightServo.setPIDFTolerance(Angle.fromDegrees(0.2))


        leftServo = RTPServo(hw, telemetry, config.leftServoConfig, absoluteEncoder)
        leftServo.setPIDFTolerance(Angle.fromDegrees(0.2))
    }
}
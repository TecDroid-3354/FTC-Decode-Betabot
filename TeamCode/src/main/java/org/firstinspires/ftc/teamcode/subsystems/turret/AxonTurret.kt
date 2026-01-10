package org.firstinspires.ftc.teamcode.subsystems.turret

import Angle
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.PIDFCoefficients
import com.seattlesolvers.solverslib.command.SubsystemBase
import org.firstinspires.ftc.robotcore.external.Supplier
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.utils.RTPServo.RTPServo
import org.firstinspires.ftc.teamcode.utils.RTPServo.RTPServoConfig

data class AxonTurretConfig(
    val rightServoConfig: RTPServoConfig,
    val leftServoConfig: RTPServoConfig,
    val limits: ClosedFloatingPointRange<Double>
)

enum class TurretState {
    LockAngle, Off
}

class AxonTurret(val hw: HardwareMap, val telemetry: Telemetry, val config: AxonTurretConfig, val turretTarget: Supplier<Angle>): SubsystemBase() {

    lateinit var rightServo: RTPServo
    lateinit var leftServo: RTPServo

    var turretState: TurretState = TurretState.Off

    init {
        servoConfig()
        rightServo.setPIDFTolerance(Angle.fromDegrees(0.5))
        leftServo.setPIDFTolerance(Angle.fromDegrees(0.5))
    }

    override fun periodic() {
        setPIDFCoefficients(rightServo, AxonTurretConstants.rightPIDCoefficients)
        setPIDFCoefficients(leftServo, AxonTurretConstants.leftPIDCoefficients)
        rightServo.periodic()
        leftServo.periodic()
    }

    fun stopTurret() {
        rightServo.stop()
        leftServo.stop()
    }

    // Needs to be called inside the opMode loop in order to correctly update target
    fun setTurretAngle(target: Angle) {
        if (getAbsoluteAngle().degrees in config.limits) {
            rightServo.setTargetAngle(target)
            leftServo.setTargetAngle(target)
        }
    }

    // Just need one encoder's reading
    fun getAbsoluteAngle(): Angle {
        return rightServo.getTotalRotation()
    }

    fun setPIDFCoefficients(servo: RTPServo, pidfCoefficients: PIDFCoefficients) {
        servo.setPIDF(pidfCoefficients)
    }

    fun servoConfig() {
        rightServo = RTPServo(hw, telemetry, config.rightServoConfig)
        leftServo = RTPServo(hw, telemetry, config.leftServoConfig)
    }
}
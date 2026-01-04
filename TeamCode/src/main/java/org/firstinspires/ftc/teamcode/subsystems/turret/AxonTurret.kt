package org.firstinspires.ftc.teamcode.subsystems.turret

import Angle
import androidx.core.util.Supplier
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.PIDFCoefficients
import com.seattlesolvers.solverslib.command.SubsystemBase
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.utils.RTPServo.RTPServo
import org.firstinspires.ftc.teamcode.utils.RTPServo.RTPServoConfig
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeDegrees
import org.firstinspires.ftc.teamcode.axonTurretTest.AxonTurretConstants.Companion
import org.firstinspires.ftc.teamcode.utils.controllers.PIDFAngleController

data class AxonTurretConfig(
    val rightServoConfig: RTPServoConfig,
    val leftServoConfig: RTPServoConfig,
    val limits: ClosedFloatingPointRange<Double>,
    val pidCoefficients: PIDFCoefficients
)

enum class TurretState {
    LockAngle, Off
}

class AxonTurret(val hw: HardwareMap, val telemetry: Telemetry, val config: AxonTurretConfig): SubsystemBase() {

    lateinit var rightServo: RTPServo
    //lateinit var leftServo: RTPServo

    var turretState: TurretState = TurretState.Off

    init {

        servoConfig()
    }

    override fun periodic() {
        telemetry.addData("Absolute reading right", rightServo.getAbsoluteAngle().degrees)
        telemetry.addData("voltage", rightServo.servoEncoder.voltage)
//        telemetry.addData("Absolute reading left", leftServo.getAbsoluteAngle().degrees)
    }

    fun stopTurret() {
        rightServo.stop()
        //leftServo.stop()
    }

    // Needs to be called inside the opMode loop in order to correctly update target
    private fun setTurretAngle(target: Angle) {
        if (getAbsoluteAngle().degrees in config.limits) {
            rightServo.setTargetAngle(target)
            //leftServo.setTargetAngle(target)
        }
    }

    // Must be called within a loop
    fun setTargetTurretAngle(target: Angle) {
        when (turretState) {
            TurretState.LockAngle -> {
                setTurretAngle(target)
            }
            TurretState.Off-> {
                setTurretAngle(Angle.fromDegrees(0.0))
            }
        }
    }

    fun toggleState() {
        turretState = if (turretState == TurretState.Off) TurretState.LockAngle else TurretState.Off
    }

    // Just need one encoder's reading
    fun getAbsoluteAngle(): Angle {
        return rightServo.getAbsoluteAngle()
    }

    fun servoConfig() {
        rightServo = RTPServo(hw, telemetry, config.rightServoConfig)
        //leftServo = RTPServo(hw, telemetry, config.leftServoConfig)
    }
}
package org.firstinspires.ftc.teamcode.subsystems.turret

import Angle
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.PIDCoefficients
import com.seattlesolvers.solverslib.command.Command
import com.seattlesolvers.solverslib.command.InstantCommand
import com.seattlesolvers.solverslib.command.SubsystemBase
import com.seattlesolvers.solverslib.controller.PIDController
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.axonTurretTest.AprilTagLocationInDegrees
import org.firstinspires.ftc.teamcode.axonTurretTest.AxonConstants
import org.firstinspires.ftc.teamcode.axonTurretTest.rightServoConfig
import org.firstinspires.ftc.teamcode.utils.Alliance
import org.firstinspires.ftc.teamcode.utils.RTPServo.RTPAxon
import org.firstinspires.ftc.teamcode.utils.RTPServo.RTPServo
import org.firstinspires.ftc.teamcode.utils.RTPServo.RTPServoConfig

data class AxonTurretConfig(
    val rightServoConfig: RTPServoConfig,
    val leftServoConfig: RTPServoConfig,
    val limits: ClosedFloatingPointRange<Double>,
    val pidController: PIDController
)

class AxonTurret(val hw: HardwareMap, val telemetry: Telemetry, val config: AxonTurretConfig): SubsystemBase() {

    lateinit var rightServo: RTPServo
    //lateinit var leftServo: RTPServo

    val isRTP = false

    init {
        //config.pidController.setIntegrationBounds()

        servoConfig()
    }

    override fun periodic() {
        rightServo.getServo().rtp = isRTP
        telemetry.addData("Total Rotation", rightServo.getServo().totalRotation)
        telemetry.addData("Current Absolute Angle", rightServo.getCurrentAbsoluteAngle())
    }

    fun setPower(output: Double = 1.0) {
        rightServo.getServo().power = output
    }

    fun alignToGoal(target: Angle): Command {
        return InstantCommand({ rightServo.setTargetRotation(target) })
    }

    fun update() {
        rightServo.update()
        //leftServo.update()
    }

    fun servoConfig() {
        rightServo = RTPServo(hw, config.rightServoConfig)
        //leftServo = RTPServo(hw, config.leftServoConfig)
    }
}
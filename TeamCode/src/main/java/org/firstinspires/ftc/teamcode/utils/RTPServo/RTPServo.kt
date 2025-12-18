package org.firstinspires.ftc.teamcode.utils.RTPServo

import Angle
import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.PIDCoefficients


data class RTPServoConfig(
    val servoId: String,
    val absoluteId: String,
    val gearRatio: Double,
    val maxPower: Double,
    val pidCoefficients: PIDCoefficients
)

@Suppress("JoinDeclarationAndAssignment")
class RTPServo(hw: HardwareMap, val config: RTPServoConfig) {

    private var servo: CRServo
    private var analogInput: AnalogInput
    private var rtpServo: RTPAxon

    init {
        servo = hw.get(CRServo::class.java, config.servoId)
        analogInput = hw.get(AnalogInput::class.java, config.absoluteId)

        rtpServo = RTPAxon(servo, analogInput)

        rtpServo.maxPower = config.maxPower
        rtpServo.setPidCoeffs(config.pidCoefficients.p, config.pidCoefficients.i, config.pidCoefficients.d)
    }

    fun setTargetRotation(target: Angle) {
        rtpServo.targetRotation = target.degrees / config.gearRatio
    }

    fun changeTargetRotation(change: Angle) {
        rtpServo.changeTargetRotation(change.degrees / config.gearRatio)
    }

    fun update() {
        rtpServo.update()
        rtpServo.setPidCoeffs(config.pidCoefficients.p, config.pidCoefficients.i, config.pidCoefficients.d)
    }

    fun getServo(): RTPAxon {
        return rtpServo
    }
}
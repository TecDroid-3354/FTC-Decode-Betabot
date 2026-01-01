package org.firstinspires.ftc.teamcode.utils.RTPServo

import Angle
import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.PIDCoefficients


data class RTPServoConfig(
    val servoId: String,
    val absoluteId: String,
    val absoluteOffset: Angle,
    val direction: RTPAxon.Direction,
    val gearRatio: Double = 1.0,
    val limits: ClosedFloatingPointRange<Double>,
    val maxPower: Double = 1.0,
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
        rtpServo.setDirection(config.direction)
        //rtpServo.setPidCoeffs(config.pidCoefficients.p, config.pidCoefficients.i, config.pidCoefficients.d)

        rtpServo.forceResetTotalRotation()
    }

    fun update() {
        rtpServo.update()
        //rtpServo.setPidCoeffs(config.pidCoefficients.p, config.pidCoefficients.i, config.pidCoefficients.d)
    }

    fun setTargetRotation(target: Angle) {
        rtpServo.targetRotation = target.degrees / config.gearRatio
    }

    fun changeTargetRotation(change: Angle) {
        rtpServo.changeTargetRotation(change.degrees / config.gearRatio)
    }


    fun getCurrentAbsoluteAngle(): Double {
        return if (rtpServo.currentAngle + config.absoluteOffset.degrees < 0.0) {
            rtpServo.currentAngle + config.absoluteOffset.degrees + 360.0
        } else {
            rtpServo.currentAngle + config.absoluteOffset.degrees
        }
    }

    fun getServo(): RTPAxon {
        return rtpServo
    }
}
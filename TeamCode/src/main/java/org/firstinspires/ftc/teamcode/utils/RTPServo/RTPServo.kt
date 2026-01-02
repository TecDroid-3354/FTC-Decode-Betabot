package org.firstinspires.ftc.teamcode.utils.RTPServo

import Angle
import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.PIDCoefficients


data class RTPServoConfig(
    val servoId: String,
    val absoluteId: String,
    val encoderDirection: RTPAxon.Direction,
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
        /* INITIALIZATION CODE */

        // Initialize both the CR servo and absolute encoder
        servo = hw.get(CRServo::class.java, config.servoId)
        analogInput = hw.get(AnalogInput::class.java, config.absoluteId)

        // Actually initialize the RTP Servo
        rtpServo = RTPAxon(servo, analogInput)

        //Set the maximum power the servo can achieve
        rtpServo.maxPower = config.maxPower
        // Sets the direction of the encoder, not the servo. REVERSE will just make the values negative
        rtpServo.setDirection(config.encoderDirection)
        // Sets the Servo's PID Coefficients, not doing so will result in normal servo's behavior
        rtpServo.setPidCoeffs(config.pidCoefficients.p, config.pidCoefficients.i, config.pidCoefficients.d)
        // Reset the rotation tracker and PID timer's
        rtpServo.forceResetTotalRotation()
    }

    // Sets an absolute angle target to the servo
    fun setTargetRotation(target: Angle) {
        rtpServo.targetRotation = target.degrees / config.gearRatio
    }

    // Gets the current angle and adds the desired change in it
    fun changeTargetRotation(change: Angle) {
        rtpServo.changeTargetRotation(change.degrees / config.gearRatio)
    }

    // Gets the absolute position considering gear ratios
    fun getAbsoluteAngle(): Angle {
        return Angle.fromDegrees(rtpServo.currentAngle * config.gearRatio)
    }

    // Returns the servo, provides more methods regarding servo's PID and behavior
    fun getServo(): RTPAxon {
        return rtpServo
    }

    // Must be called once in a loop in order to update PIDs and actually move the servo
    fun update() {
        rtpServo.update()
        rtpServo.setPidCoeffs(config.pidCoefficients.p, config.pidCoefficients.i, config.pidCoefficients.d)
    }
}
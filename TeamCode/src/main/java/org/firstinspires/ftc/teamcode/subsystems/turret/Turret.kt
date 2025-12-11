package org.firstinspires.ftc.teamcode.subsystems.turret

import Angle
import com.qualcomm.robotcore.hardware.HardwareMap
import com.seattlesolvers.solverslib.command.Command
import com.seattlesolvers.solverslib.command.InstantCommand
import com.seattlesolvers.solverslib.command.SubsystemBase
import com.seattlesolvers.solverslib.controller.PIDFController
import com.seattlesolvers.solverslib.hardware.motors.Motor
import com.seattlesolvers.solverslib.util.MathUtils
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.utils.positionMotorEx.PositionMotorEx
import org.firstinspires.ftc.teamcode.subsystems.turret.TurretConstants.Limits
import org.firstinspires.ftc.teamcode.vision.Limelight

/**
 * Intended to control the [Turret] subsystem.
 * @param hardwareMap contains the information of the hardware. Configurable through Driver Hub.
 * @param telemetry to print real-time data through Driver Hub.
 */
class Turret(
    val hardwareMap: HardwareMap,
    val telemetry: Telemetry
): SubsystemBase() {

    // Motor that controls the Turret. Motor: GoBilda 435 RPM
    private val motorController: PositionMotorEx
    private val turretAnglePIDFController = PIDFController(TurretConstants.PIDF.turretAnglePIDFController)

    // Initialization //
    init {
        // Ensure functional limits
        require(Limits.maximumLimit.degrees > Limits.minimumLimit.degrees)
        motorController = PositionMotorEx(
            Motor(hardwareMap, TurretConstants.Identification.turretId), turretMotorConfig)

        motorController.setMode(Motor.RunMode.RawPower)
    }

    // Code ccalled every robot loop //
    override fun periodic() {
        // Un-comment this line if you want to modify in real-time the turret PIDF Coefficients
        motorController.setPIDFCoefficients(TurretConstants.PIDF.pidLLCoefficients)

        telemetry.addData("TurretPositionDegrees", motorController.getPosition().degrees)
    }

    /**
     * Receives a power and sets it to the motor if the [Turret] is inside its physical limits defined in
     * [TurretConstants.Limits], if the range is not met, then the motors are not commanded to move
     * @param power the desired motor's power to be set to
     */
    fun setTurretVoltage(power: Double) {
        if ((motorController.getPosition().degrees <= Limits.minimumLimit.degrees && power < 0.0) ||
            (motorController.getPosition().degrees >= Limits.maximumLimit.degrees && power > 0.0)) {
            motorController.stopMotor()
        } else {
            motorController.setPower(power)
        }
    }

    /**
     * Sets the angle of the subsystem. Takes into account limits defined in [TurretConstants.Limits]
     * @param angle must be the angle of the subsystem, not the motor.
     */
    fun setTurretAngle(angle: Angle): Command {
        val coercedAngle = Angle.fromDegrees(
            MathUtils.clamp(angle.degrees,
            Limits.minimumLimit.degrees,
            Limits.maximumLimit.degrees)
        )

        val power = turretAnglePIDFController.calculate(motorController.getPosition().degrees, coercedAngle.degrees)
        return InstantCommand({ setTurretVoltage(power) })
    }

    /**
     * Receives the current offset angle from the [Limelight] and utilizes the turret pidfController to calculate
     * the motor's necessary output to align the whole turret
     * @param tx the current offset angle from the [Limelight] to the April Tag
     */
    fun alignToAprilTag(tx: Double, offset: Double = 0.0): Command {
        val power = motorController.pidfController.calculate(tx, 0.0 + offset)
        return InstantCommand({ setTurretVoltage(-power) })
    }
}
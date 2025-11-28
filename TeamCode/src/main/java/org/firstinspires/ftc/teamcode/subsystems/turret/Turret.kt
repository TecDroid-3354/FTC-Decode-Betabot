package org.firstinspires.ftc.teamcode.subsystems.turret

import Angle
import com.qualcomm.robotcore.hardware.HardwareMap
import com.seattlesolvers.solverslib.command.SubsystemBase
import com.seattlesolvers.solverslib.hardware.motors.Motor
import com.seattlesolvers.solverslib.util.MathUtils
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.utils.positionMotorEx.PositionMotorEx
import org.firstinspires.ftc.teamcode.subsystems.turret.TurretConstants.Limits

/**
 * Intended to control the [Turret] subsystem.
 * @param hardwareMap contains the information of the hardware. Configurable through Driver Hub.
 * @param telemetry to print real-time data through Driver Hub.
 */
@Suppress("JoinDeclarationAndAssignment")
class Turret(val hardwareMap: HardwareMap, val telemetry: Telemetry): SubsystemBase() {
    // Motor that controls the Turret. Motor: GoBilda 435 RPM
    private val motorController: PositionMotorEx

    // Initialization //
    init {
        // Ensure functional limits
        require(Limits.maximumLimit.degrees > Limits.minimumLimit.degrees)
        motorController = PositionMotorEx(
            Motor(hardwareMap, TurretConstants.Identification.turretId), turretMotorConfig)

        motorController.setMode(Motor.RunMode.RawPower)
    }

    override fun periodic() {
        motorController.setPIDFCoefficients(TurretConstants.PIDF.pidfCoefficients)
        telemetry.addData("TurretPositionDegrees", motorController.getPosition().degrees)
    }

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
    fun setTurretAngle(angle: Angle) {
        val coercedAngle = Angle.fromDegrees(
            MathUtils.clamp(angle.degrees,
            Limits.minimumLimit.degrees,
            Limits.maximumLimit.degrees)
        )

        val power = motorController.pidfController.calculate(motorController.getPosition().degrees, coercedAngle.degrees)
        setTurretVoltage(power)

    }

    fun alignToAprilTag(tx: Double) {
        val power = motorController.pidfController.calculate(tx, 0.0)
        setTurretVoltage(-power)
    }
}
package org.firstinspires.ftc.teamcode.subsystems.intake

import LinearVelocity
import com.qualcomm.robotcore.hardware.HardwareMap
import com.seattlesolvers.solverslib.command.SubsystemBase
import com.seattlesolvers.solverslib.controller.PIDFController
import com.seattlesolvers.solverslib.hardware.motors.Motor
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.utils.velocityMotorEx.VelocityMotorConfig
import org.firstinspires.ftc.teamcode.utils.velocityMotorEx.VelocityMotorEx

class Intake(
    val hardwareMap: HardwareMap,
    val telemetry: Telemetry
) : SubsystemBase() {
    // Consider that right & left motors refer to the motors as seen from the turret
    lateinit var rightMotor: VelocityMotorEx
    lateinit var leftMotor: VelocityMotorEx

    init {
        motorConfig()
    }

    override fun periodic() {
        // Telemetry to retrieve useful subsystem data
    }


    // Functional code //

    // This is the method called in the OpMode. We basically give it the direction we're intaking
    // from, and the subsystem moves the motors accordingly
    fun enableIntake(direction: IntakeDirection) {
        when (direction) {
            IntakeDirection.RIGHT -> enableRollers(rightMotor)
            IntakeDirection.LEFT -> enableRollers(leftMotor)
        }
    }

    // The following method moves the rollers, i.e. "intakes", from a given side
    // It is able to take a custom velocity, or use the default 0.8
    private fun enableRollers(motor: VelocityMotorEx, velocity: Double = IntakeConstants.General.velocity) {
        motor.setVelocity(LinearVelocity.fromMps(velocity))
    }

    // Quite literally stops the motors
    fun stopIntake(direction: IntakeDirection) {
        when (direction) {
            IntakeDirection.RIGHT -> rightMotor.stopMotor()
            IntakeDirection.LEFT -> leftMotor.stopMotor()
        }
    }


    // Setup code //

    // Configuring motors with the custom VelocityEx class
    private fun motorConfig() {
        rightMotor = VelocityMotorEx(
            Motor(hardwareMap, "rightIntakeMotor", 28.0, 6000.0),
            VelocityMotorConfig(
                direction = Motor.Direction.REVERSE,
                pidfCoefficients = PIDFController(
                    IntakeConstants.Motor.p,
                    IntakeConstants.Motor.i,
                    IntakeConstants.Motor.d,
                    IntakeConstants.Motor.f))
        )


        leftMotor = VelocityMotorEx(
            Motor(hardwareMap, "leftIntakeMotor", 28.0, 6000.0),
            VelocityMotorConfig(
                direction = Motor.Direction.REVERSE,
                pidfCoefficients = PIDFController(
                    IntakeConstants.Motor.p,
                    IntakeConstants.Motor.i,
                    IntakeConstants.Motor.d,
                    IntakeConstants.Motor.f))
        )
    }
}
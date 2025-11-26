package org.firstinspires.ftc.teamcode.subsystems.intake

import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.seattlesolvers.solverslib.command.Command
import com.seattlesolvers.solverslib.command.InstantCommand
import com.seattlesolvers.solverslib.command.SubsystemBase
import com.seattlesolvers.solverslib.controller.PIDFController
import com.seattlesolvers.solverslib.hardware.motors.Motor
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.utils.velocityMotorEx.VelocityMotorConfig
import org.firstinspires.ftc.teamcode.utils.velocityMotorEx.VelocityMotorEx

enum class IntakeDirection {
    RIGHT,
    LEFT
}

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

    // Functional code //

    // This is the method called in the OpMode. We basically give it the direction we're intaking
    // from, and the subsystem moves the motors accordingly
    fun enableIntake(direction: IntakeDirection, output: Double = 1.0) {
        when (direction) {
            IntakeDirection.RIGHT -> rightMotor.setPower(output)
            IntakeDirection.LEFT -> leftMotor.setPower(output)
        }
    }

    // Quite literally stops the motors
    fun stopIntake(direction: IntakeDirection) {
        when (direction) {
            IntakeDirection.RIGHT -> rightMotor.setPower(0.0) // Check
            IntakeDirection.LEFT -> leftMotor.setPower(0.0) // Check
        }
    }

    fun enableBothIntakes(): Command {
        return InstantCommand({
            enableIntake(IntakeDirection.RIGHT)
            enableIntake(IntakeDirection.LEFT)
        })
    }

    fun stopBothIntakes(): Command {
        return InstantCommand({
            stopIntake(IntakeDirection.RIGHT)
            stopIntake(IntakeDirection.LEFT)
        })

    }


    // Setup code //

    // Configuring motors with the custom VelocityEx class
    private fun motorConfig() {
        rightMotor = VelocityMotorEx(
            Motor(hardwareMap, "rightIntakeMotor", 28.0, 6000.0),
            VelocityMotorConfig(
                direction = DcMotorSimple.Direction.FORWARD,
                pidfCoefficients = PIDFController(0.5, 0.0, 0.0, 0.0))
        )

        leftMotor = VelocityMotorEx(
            Motor(hardwareMap, "leftIntakeMotor", 28.0, 6000.0),
            VelocityMotorConfig(
                direction = DcMotorSimple.Direction.REVERSE,
                pidfCoefficients = PIDFController(0.5, 0.0, 0.0, 0.0))
        )
    }
}
package org.firstinspires.ftc.teamcode.subsystems.intake

import org.firstinspires.ftc.teamcode.CMDOpMode
import com.qualcomm.robotcore.hardware.HardwareMap
import com.seattlesolvers.solverslib.command.Command
import com.seattlesolvers.solverslib.command.InstantCommand
import com.seattlesolvers.solverslib.command.SubsystemBase
import com.seattlesolvers.solverslib.hardware.motors.MotorEx
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.shooter.IntakeConstants
import org.firstinspires.ftc.teamcode.utils.velocityMotorEx.VelocityMotorConfig
import org.firstinspires.ftc.teamcode.utils.velocityMotorEx.VelocityMotorEx

// Intake sides in our robot
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

    /**
     * This is the method called in the [CMDOpMode]. We basically give it the direction we're intaking and
     * it enables that motor
     * @param direction the side of the intake it will enable
     */
    fun enableIntake(direction: IntakeDirection, output: Double = 1.0) {
        when (direction) {
            IntakeDirection.RIGHT -> rightMotor.setPower(output)
            IntakeDirection.LEFT -> leftMotor.setPower(output)
        }
    }

    /**
     * This is the method called in the [CMDOpMode]. It calls [enableIntake] twice and turns on both intakes
     */
    fun enableBothIntakes(output: Double = 1.0): Command {
        return InstantCommand({
            enableIntake(IntakeDirection.RIGHT, output)
            enableIntake(IntakeDirection.LEFT, output)
        })
    }

    /**
     * Quite literally stops the motor
     * @param direction The [Intake] side you want to stop
     */
    private fun stopIntake(direction: IntakeDirection) {
        when (direction) {
            IntakeDirection.RIGHT -> rightMotor.setPower(0.0) // Check
            IntakeDirection.LEFT -> leftMotor.setPower(0.0) // Check
        }
    }

    /**
     * Quite literally stops both motors
     */
    fun stopBothIntakes(): Command {
        return InstantCommand({
            stopIntake(IntakeDirection.RIGHT)
            stopIntake(IntakeDirection.LEFT)
        })
    }

    fun isActive(): Boolean {
        return rightMotor.motor.motor.power > 0.1 || leftMotor.motor.motor.power > 0.1
    }

    // This code executes indefinitely during our robot's program
    override fun periodic() {
        rightMotor.setPIDFCoefficients(IntakeConstants.PIDF.pidfController)
        leftMotor.setPIDFCoefficients(IntakeConstants.PIDF.pidfController)
    }

    // Setup code //

    // Configuring motors with the custom VelocityEx class
    private fun motorConfig() {
        rightMotor = VelocityMotorEx(
            MotorEx(hardwareMap, IntakeConstants.Identification.rightIntakeMotorId,
                IntakeConstants.Configuration.ticksPerRevolution, IntakeConstants.Configuration.rpm),
            VelocityMotorConfig(
                IntakeConstants.Configuration.zeroPowerBehavior,
                IntakeConstants.Configuration.rightIsInverted,
                pidfCoefficients = IntakeConstants.PIDF.pidfController)
        )

        leftMotor = VelocityMotorEx(
            MotorEx(hardwareMap, IntakeConstants.Identification.leftIntakeMotorId,
                IntakeConstants.Configuration.ticksPerRevolution, IntakeConstants.Configuration.rpm),
            VelocityMotorConfig(
                IntakeConstants.Configuration.zeroPowerBehavior,
                IntakeConstants.Configuration.leftIsInverted,
                pidfCoefficients = IntakeConstants.PIDF.pidfController)
        )
    }
}
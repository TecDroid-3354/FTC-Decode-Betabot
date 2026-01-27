package org.firstinspires.ftc.teamcode.subsystems.intake

import org.firstinspires.ftc.teamcode.OpModes.CMDOpMode
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
    FRONT, BACK
}

class Intake(
    val hardwareMap: HardwareMap,
    val telemetry: Telemetry
) : SubsystemBase() {
    // Consider that right & left motors refer to the motors as seen from the turret
    lateinit var frontMotor: VelocityMotorEx
    lateinit var backMotor: VelocityMotorEx

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
            IntakeDirection.FRONT -> frontMotor.setPower(output)
            IntakeDirection.BACK -> backMotor.setPower(output)
        }
    }

    /**
     * This is the method called in the [CMDOpMode]. It calls [enableIntake] twice and turns on both intakes
     */
    fun enableBothIntakes(): Command {
        return InstantCommand({
            enableIntake(IntakeDirection.FRONT, 1.0)
            enableIntake(IntakeDirection.BACK, 1.0)
        })
    }

    fun enableBothOuttakes(): Command {
        return InstantCommand({
            enableIntake(IntakeDirection.FRONT, -1.0)
            enableIntake(IntakeDirection.BACK, -1.0)
        })
    }

    /**
     * Quite literally stops the motor
     * @param direction The [Intake] side you want to stop
     */
    private fun stopIntake(direction: IntakeDirection) {
        when (direction) {
            IntakeDirection.FRONT -> frontMotor.setPower(0.0) // Check
            IntakeDirection.BACK -> backMotor.setPower(0.0) // Check
        }
    }

    /**
     * Quite literally stops both motors
     */
    fun stopBothIntakes(): Command {
        return InstantCommand({
            stopIntake(IntakeDirection.FRONT)
            stopIntake(IntakeDirection.BACK)
        })
    }

    // This code executes indefinitely during our robot's program
    override fun periodic() {
        // Uncomment these lines for real-time PID tuning
//        frontMotor.setPIDFCoefficients(IntakeConstants.PIDF.pidfCoefficients)
//        backMotor.setPIDFCoefficients(IntakeConstants.PIDF.pidfCoefficients)
    }

    // Setup code //

    // Configuring motors with the custom VelocityEx class
    private fun motorConfig() {
        frontMotor = VelocityMotorEx(
            MotorEx(hardwareMap, IntakeConstants.Identification.rightIntakeMotorId,
                IntakeConstants.Configuration.ticksPerRevolution, IntakeConstants.Configuration.rpm),
            VelocityMotorConfig(
                IntakeConstants.Configuration.zeroPowerBehavior,
                IntakeConstants.Configuration.rightIsInverted,
                pidfCoefficients = IntakeConstants.PIDF.pidfCoefficients)
        )

        backMotor = VelocityMotorEx(
            MotorEx(hardwareMap, IntakeConstants.Identification.leftIntakeMotorId,
                IntakeConstants.Configuration.ticksPerRevolution, IntakeConstants.Configuration.rpm),
            VelocityMotorConfig(
                IntakeConstants.Configuration.zeroPowerBehavior,
                IntakeConstants.Configuration.leftIsInverted,
                pidfCoefficients = IntakeConstants.PIDF.pidfCoefficients)
        )
    }
}
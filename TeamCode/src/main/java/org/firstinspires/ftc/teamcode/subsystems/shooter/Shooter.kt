package org.firstinspires.ftc.teamcode.shooter

import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.PIDFCoefficients
import com.seattlesolvers.solverslib.command.Command
import com.seattlesolvers.solverslib.command.InstantCommand
import com.seattlesolvers.solverslib.command.SubsystemBase
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import kotlin.time.Instant

/**
 * This is the code for controlling the shooter wheels on our robot.
 *
 */
@Suppress("JoinDeclarationAndAssignment")
class Shooter(
    hw: HardwareMap,
    val telemetry: Telemetry
): SubsystemBase() {

    // This is where the motor intended to control the shooter is declared
    //val motor: VelocityMotorEx
    val motor:  DcMotorEx

    // Initialization //

    // This is the code that will execute when the class is initialized
    init {
        motor = hw.get(DcMotorEx::class.java, ShooterConstants.Identification.shooterMotorId)
        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODERS, ShooterConstants.PIDF.pidController)
        motorConfiguration()
    }

    // Periodic method //
    override fun periodic() {
        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODERS, ShooterConstants.PIDF.pidController)
    }

    // Functional code //
    // Setters //

    /**
     * Sets the motor's velocity to a desired angular velocity
     */

    fun shoot() {
        motor.mode = DcMotor.RunMode.RUN_USING_ENCODERS
        motor.setVelocity(20000.0, AngleUnit.DEGREES)
    }

    fun shootCMD() : Command {
        return InstantCommand({
            shoot()
        })
    }

    /**
     * Calls the super class method for stopping the motor
      */
    fun stop() {
        motor.power = 0.0
        motor.velocity = 0.0
    }

    // Getters //

    /**
     * Checks if the shooter motor is active and running, useful for logic control
     * @return true if the motor is running
     */
    fun isActive(): Boolean {
        return motor.power > 0.1
    }

    /**
     * Configures the motor with the given values in the constant sheet
     */
    fun motorConfiguration() {
        // The motor's configuration is grabbed from the constant's file
        motor.mode = ShooterConstants.Configuration.runMode
        motor.direction = ShooterConstants.Configuration.direction
        motor.zeroPowerBehavior = ShooterConstants.Configuration.zeroPowerBehavior
    }
}
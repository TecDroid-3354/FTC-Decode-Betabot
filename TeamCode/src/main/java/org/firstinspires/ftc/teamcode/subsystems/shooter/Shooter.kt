package org.firstinspires.ftc.teamcode.subsystems.shooter

import AngularVelocity
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import com.seattlesolvers.solverslib.command.Command
import com.seattlesolvers.solverslib.command.InstantCommand
import com.seattlesolvers.solverslib.command.SubsystemBase
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.shooter.ShooterConstants

/**
 * This is the code for controlling the shooter wheels on our robot.
 *
 */
class Shooter(
    val hw: HardwareMap,
    val telemetry: Telemetry
): SubsystemBase() {

    // This is where the motor intended to control the shooter is declared
    private lateinit var firstMotor: DcMotorEx
    private lateinit var secondMotor: DcMotorEx

    // Initialization //

    // This is the code that will execute when the class is initialized
    init {
        motorConfiguration()
    }

    // Periodic method //
    override fun periodic() {
        firstMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODERS, ShooterConstants.PIDF.firstMotorPIDCoefficients)
        secondMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODERS, ShooterConstants.PIDF.secondMotorPIDCoefficients)
    }

    // Functional code //
    // Setters //

    /**
     * Sets the motor's velocity to a desired angular velocity
     */

    private fun shoot() {
        firstMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        secondMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER

        firstMotor.setVelocity(Angle.fromRotations(100.0).degrees, AngleUnit.DEGREES)
        secondMotor.setVelocity(Angle.fromRotations(100.0).degrees, AngleUnit.DEGREES)
    }

    fun shootCMD(): Command {
        return InstantCommand({
            shoot()
        })
    }

    /**
     * Calls the super class method for stopping the motor
      */
    fun stop() {
        firstMotor.power = 0.0
        firstMotor.velocity = 0.0

        secondMotor.power = 0.0
        secondMotor.velocity = 0.0
    }

    fun stopCMD(): Command {
        return InstantCommand({
            stop();
        })
    }

    // Getters //

    /**
     * Checks if the shooter motor is active and running, useful for logic control
     * @return true if the motor is running
     */
    fun isActive(): Boolean {
        return firstMotor.power > 0.2 || firstMotor.velocity > 0.2
    }

    fun getVelocity(): AngularVelocity {
        return AngularVelocity(firstMotor.velocity / 28.0)
    }

    /**
     * Configures the motor with the given values in the constant sheet
     */
    fun motorConfiguration() {
        // The motor's configuration is grabbed from the constant's file
        firstMotor = hw.get(DcMotorEx::class.java, ShooterConstants.Identification.firstMotorId)
        firstMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODERS, ShooterConstants.PIDF.firstMotorPIDCoefficients)
        firstMotor.mode = ShooterConstants.Configuration.runMode
        firstMotor.direction = ShooterConstants.Configuration.direction
        firstMotor.zeroPowerBehavior = ShooterConstants.Configuration.zeroPowerBehavior

        secondMotor = hw.get(DcMotorEx::class.java, ShooterConstants.Identification.secondMotorId)
        secondMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODERS, ShooterConstants.PIDF.secondMotorPIDCoefficients)
        secondMotor.mode = ShooterConstants.Configuration.runMode
        secondMotor.direction = ShooterConstants.Configuration.direction
        secondMotor.zeroPowerBehavior = ShooterConstants.Configuration.zeroPowerBehavior
    }
}
package org.firstinspires.ftc.teamcode.subsystems.shooter

import AngularVelocity
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.PIDCoefficients
import com.qualcomm.robotcore.hardware.PIDFCoefficients
import com.seattlesolvers.solverslib.command.Command
import com.seattlesolvers.solverslib.command.InstantCommand
import com.seattlesolvers.solverslib.command.SubsystemBase
import com.seattlesolvers.solverslib.controller.PIDController
import com.seattlesolvers.solverslib.controller.PIDFController
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

    private var pidController = PIDFController(10.0, 0.0, 0.0, 0.0)
    private var desiredVelocity = AngularVelocity.fromRpm(0.0)

    // Initialization //

    // This is the code that will execute when the class is initialized
    init {
        pidController.setTolerance(AngularVelocity.fromRpm(2.0).rpm)
        motorConfiguration()
    }

    // Periodic method //
    override fun periodic() {
        setPIDCoefficients(ShooterConstants.PIDF.firstMotorPIDCoefficients)
        telemetry.addData("Shooter velocity", getVelocity().rpm)
        telemetry.addData("Shooter desired velocity", desiredVelocity.rpm)

        desiredVelocity = AngularVelocity.fromRpm(ShooterConstants.Velocity.shooterDesiredVelocity)

//        val output = pidController.calculate(getVelocity().rpm, desiredVelocity.rpm)
//
//        firstMotor.power = output
//        secondMotor.power = output
    }

    // Functional code //
    // Setters //

    /**
     * Sets the motor's velocity to a desired angular velocity
     */

//    private fun setFlyWheelVelocity(velocity: AngularVelocity) {
//        firstMotor.setVelocity(velocity.degPerSec, AngleUnit.DEGREES)
//        secondMotor.setVelocity(velocity.degPerSec, AngleUnit.DEGREES)
//    }

    fun setFlyWheelVelocity() {
//        desiredVelocity = velocity
        firstMotor.power = 1.0
        secondMotor.power = 1.0
    }

    fun shootCMD(): Command {
        return InstantCommand({
//            setFlyWheelVelocity(AngularVelocity.fromRpm(ShooterConstants.Velocity.shooterDesiredVelocity))
        })
    }

    /**
     * Calls the super class method for stopping the motor
      */
    fun stop(motor: DcMotorEx) {
        motor.velocity = 0.0
    }

    fun stopCMD(): Command {
        return InstantCommand({
            stop(firstMotor)
            stop(secondMotor)
        })
    }

    private fun setPIDCoefficients(pidfCoefficients: PIDFCoefficients) {
        pidController.setPIDF(pidfCoefficients.p, pidfCoefficients.i, pidfCoefficients.d, pidfCoefficients.f)
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
        firstMotor.mode = ShooterConstants.Configuration.runMode
        firstMotor.direction = ShooterConstants.Configuration.direction
        firstMotor.zeroPowerBehavior = ShooterConstants.Configuration.zeroPowerBehavior

        secondMotor = hw.get(DcMotorEx::class.java, ShooterConstants.Identification.secondMotorId)
        secondMotor.mode = ShooterConstants.Configuration.runMode
        secondMotor.direction = ShooterConstants.Configuration.direction
        secondMotor.zeroPowerBehavior = ShooterConstants.Configuration.zeroPowerBehavior
    }
}
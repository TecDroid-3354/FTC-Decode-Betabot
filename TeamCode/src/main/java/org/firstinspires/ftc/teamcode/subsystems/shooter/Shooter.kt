package org.firstinspires.ftc.teamcode.subsystems.shooter

import AngularVelocity
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.PIDFCoefficients
import com.seattlesolvers.solverslib.command.Command
import com.seattlesolvers.solverslib.command.InstantCommand
import com.seattlesolvers.solverslib.command.SubsystemBase
import org.firstinspires.ftc.robotcore.external.Telemetry
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

    private var desiredVelocity = AngularVelocity.fromRpm(0.0)

    // Initialization //

    // This is the code that will execute when the class is initialized
    init {
        motorConfiguration()
    }

    // Periodic method //
    override fun periodic() {
        // Try tuning the F value first to a low velocity, not 0, cause it will ruin the use of feed forward
        // After you've tuned that, you can start tuning the p value by transitioning from a hiw velocity to a low one
        // Try to reach to the minimum error possible and a high output when ramping down when a ball passes through.
        setPIDCoefficients(ShooterConstants.PIDF.pidfCoefficients)

        desiredVelocity = AngularVelocity.fromRpm(ShooterConstants.Velocity.shooterDesiredVelocity)

//        setFlyWheelVelocity(desiredVelocity)

        telemetry.addData("Shooter velocity", getVelocity().rpm)
        telemetry.addData("Shooter desired velocity", desiredVelocity.rpm)
        telemetry.addData("Error", desiredVelocity.rpm - getVelocity().rpm)
    }

    // Functional code //
    // Setters //


    private fun setVelocity(motor: DcMotorEx, velocity: AngularVelocity) {
        motor.setVelocity(velocity.rotPerSec * ShooterConstants.Configuration.ticksPerRotation)
    }
    /**
     * Sets the motor's velocity to a desired angular velocity
     */
    fun setFlyWheelVelocity(velocity: AngularVelocity) {
        setVelocity(firstMotor, velocity)
        setVelocity(secondMotor, velocity)
    }

    fun setFlyWheelPower(outPut: Double) {
        firstMotor.power = outPut
        secondMotor.power = outPut
    }

    fun shootCMD(): Command {
        return InstantCommand({
//            setFlyWheelVelocity(AngularVelocity.fromRpm(ShooterConstants.Velocity.shooterDesiredVelocity))
            setFlyWheelPower(1.0)
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
        firstMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients)
        secondMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients)
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
        return AngularVelocity(firstMotor.velocity / ShooterConstants.Configuration.ticksPerRotation)
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

        setPIDCoefficients(ShooterConstants.PIDF.pidfCoefficients)
    }
}
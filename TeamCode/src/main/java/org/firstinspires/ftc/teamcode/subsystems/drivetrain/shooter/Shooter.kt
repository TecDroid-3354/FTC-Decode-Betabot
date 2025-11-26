package org.firstinspires.ftc.teamcode.shooter

import Angle
import AngularVelocity
import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.robotcore.hardware.HardwareMap
import com.seattlesolvers.solverslib.command.SubsystemBase
import com.seattlesolvers.solverslib.hardware.motors.Motor
import com.seattlesolvers.solverslib.hardware.motors.MotorEx
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.utils.velocityMotorEx.VelocityMotorConfig
import org.firstinspires.ftc.teamcode.utils.velocityMotorEx.VelocityMotorEx

/**
 * Creates a [VelocityMotorConfig] in order to store the desired configurations for the
 * shooter's velocity motor
 */
val shooterMotorConfig = VelocityMotorConfig(
    ShooterConstants.zeroPowerBehavior,
    ShooterConstants.direction,
    ShooterConstants.ticksPerRevolution,
    ShooterConstants.revPerMin,
    ShooterConstants.pidController,
    ShooterConstants.reduction
)
/**
 * This is the code for controlling the shooter wheels on our robot.
 *
 */
@Suppress("JoinDeclarationAndAssignment")
class Shooter(hw: HardwareMap, val telemetry: Telemetry): SubsystemBase() {

    // This is where the motor intended to control the shooter is declared
    //val motor: VelocityMotorEx
    val motor:  VelocityMotorEx

    // Initialization //

    // This is the code that will execute when the class is initialized
    init {
//        motor = MotorEx(hw, ShooterConstants.shooterMotorId, ShooterConstants.motorType)
//        motor.setRunMode(Motor.RunMode.VelocityControl)
//        motor.setVeloCoefficients(1.0, 0.0, 0.0)
        // The shooter motor controller is initialized by creating an instance of Motor and passing a velocity motor config

        motor = VelocityMotorEx(Motor(hw, ShooterConstants.shooterMotorId, 28.0, 6000.0), shooterMotorConfig)
    }

    // Periodic method //

    override fun periodic() {
        telemetry.addData("position", motor.motor.rate)
    }

    // Functional code //
    // Setters //

    /**
     * Sets the motor's velocity to a desired angular velocity
     */
    fun setVelocity(velocity: AngularVelocity) {
        //motor.setVelocity(velocity)
        motor.setPower(2.0)
    }

    /**
     * Calls the super class method for stopping the motor
      */
    fun stop() {
        motor.stopMotor()
    }

    // Getters //

    /**
     * Checks if the shooter motor is active and running, useful for logic control
     * @return true if the motor is running
     */
//    fun isActive(): Boolean {
//        return motor.get() > 0.05 || motor.get() < -0.05
//        //return motor.getVelocity().rotPerSec >= 0.01
//    }
//
//    /**
//     * Gets the current motor's velocity in degrees / second
//     * @return the current velocity in deg / sec
//     */
//    fun getVelocity(): Double {
//        //return motor.getVelocity().rpm
//        return motor.velocity
//    }
}
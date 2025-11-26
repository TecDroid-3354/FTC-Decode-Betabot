package org.firstinspires.ftc.teamcode.shooter

import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.seattlesolvers.solverslib.controller.PIDController
import com.seattlesolvers.solverslib.hardware.motors.Motor

object ShooterConstants {

    // Shooter constants //

    // These are constant values that the shooter subsystem has, these values cannot be changed unless design
    // says the opposite.

    // This is the identifier you need to specify in the Control Hub's configuration
    const val shooterMotorId = "shooterMotor"
    // This value needs to be measured physically
    const val ticksPerRevolution = 28.0
    // If the rotations of the motor are not equal to the subsystem's output, we need to add a reductio
    // Ask design for new value
    const val reduction = 27.0 / 18.0
    // Revolutions per minute
    const val revPerMin = 6000.0

    // These are also constants, but in Kotlin you can only declare as constants variables of primitive types

    // The motor's behavior when is not given any output
    val zeroPowerBehavior = Motor.ZeroPowerBehavior.BRAKE
    // The motor's direction
    val direction = DcMotorSimple.Direction.REVERSE
    // The PID controller
    val pidController = PIDController(1.0, 0.0, 0.0)

    val motorType = Motor.GoBILDA.BARE
}
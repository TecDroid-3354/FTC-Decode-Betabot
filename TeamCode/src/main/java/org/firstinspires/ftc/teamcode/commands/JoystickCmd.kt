package org.firstinspires.ftc.teamcode.commands

import com.seattlesolvers.solverslib.command.CommandBase
import com.seattlesolvers.solverslib.geometry.Rotation2d
import com.seattlesolvers.solverslib.kinematics.wpilibkinematics.ChassisSpeeds
import com.seattlesolvers.solverslib.util.MathUtils.clamp
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.MecanumConstants
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.SolversMecanum
import java.util.function.DoubleSupplier
import kotlin.math.pow


/**
 * Creates a new DefaultDrive.
 *
 * @param mecanum The drive subsystem this command will run on.
 * @param x Left joystick's X value
 * @param y Left joystick's Y value
 * @param heading Right joystick's X value
 */

class JoystickCmd(
    val x: DoubleSupplier,
    val y: DoubleSupplier,
    val heading: DoubleSupplier,
    val yawInDegrees: DoubleSupplier,
    val mecanum: SolversMecanum
) : CommandBase() {

    // Setup code //

    init {
        // addRequirements() tells the scheduler that the subsystem is being used
        addRequirements(mecanum)
    }


    // Functional code //

    override fun execute() {
        // This is the main body of the command. Is executed periodically while the command is scheduled

        // The value is raised to a power of 3.0 so that the chassis speeds is smoother
        val yVel = clamp(y.asDouble.pow(3.0), -1.0, 1.0)
        val xVel = clamp(x.asDouble.pow(3.0), -1.0, 1.0)
        val headingVel = clamp(heading.asDouble.pow(3.0), -1.0, 1.0)

        // Declaring a chassis speeds
        val speeds = ChassisSpeeds(
            yVel,
            xVel,
            headingVel)

        // Passing said chassis speeds so that the mecanum is able to use it
        mecanum.setChassisSpeedsFromFieldOriented(speeds, yawInDegrees.asDouble)
    }


    // Conditional code //

    override fun initialize() {
        // Piece of code to execute when the command is initially scheduled
    }

    override fun end(interrupted: Boolean) {
        // Piece of code to execute once the command ends, either normally or unscheduled
    }

    override fun isFinished(): Boolean {
        // Condition to determine whether the command is finished or not
        return super.isFinished()
    }
}
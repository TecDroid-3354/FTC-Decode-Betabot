package org.firstinspires.ftc.teamcode.subsystems.drivetrain

import AngularVelocity
import LinearVelocity
import kotlin.math.PI

class MecanumConstants {
    object Ids {
        const val frontRightId: String = "frontRight" // port 1 expansion
        const val frontLeftId: String = "frontLeft" // port 1 control
        const val backRightId: String = "backRight" // port 0 expansion
        const val backLeftId: String = "backLeft" // port 3 control
    }

    object Physics {
        const val ticksPerRevolution: Double = 2150.4
        val circumference = Distance.fromCm(7.5 * PI)
    }

    object Velocities {
        val maxVelocityY: LinearVelocity = LinearVelocity.fromMps(1.5)
        val maxVelocityX: LinearVelocity = LinearVelocity.fromMps(1.5)
        val maxRotationVelocity: AngularVelocity = AngularVelocity.fromDegPerSec(30.0)
    }
}
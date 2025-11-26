package org.firstinspires.ftc.teamcode.subsystems.drivetrain

import AngularVelocity
import LinearVelocity
import kotlin.math.PI

class MecanumConstants {
    object Ids {
        const val frontRightId: String = "frontRight" // port 1 expansion
        const val frontLeftId: String = "frontLeft" // port 4 control
        const val backRightId: String = "backRight" // port 0 expansion
        const val backLeftId: String = "backLeft" // port 3 control
    }

    object Physics {
        const val countPerRevolution: Double = 28.0 // ticks per revolution
        const val maxRPM: Double = 6000.0

    }
}
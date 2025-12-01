package org.firstinspires.ftc.teamcode.subsystems.drivetrain

class MecanumConstants {

    object Ids {
        // The chassis motor's Ids, they must be called inside the robot's configuration in the Control Hub
        const val frontRightId: String = "frontRight" // port 1 expansion
        const val frontLeftId: String = "frontLeft" // port 4 control
        const val backRightId: String = "backRight" // port 0 expansion
        const val backLeftId: String = "backLeft" // port 3 control
    }

    object Physics {
        const val countPerRevolution: Double = 28.0 // ticks per revolution
        const val maxRPM: Double = 6000.0 // max revolutions per minute
    }
}
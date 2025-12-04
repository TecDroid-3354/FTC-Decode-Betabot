package org.firstinspires.ftc.teamcode.subsystems.shooter

class HoodConstants {

    object Identification {
        // This is the motor's Id, it needs to be called in the Driver Hub's configuration
        val hoodId = "hoodServo"
    }

    object Configuration {
        // Whether the motor is inverted
        val hoodServoInverted = false
    }

    object Positions {
        // This positions must not be modified as they were obtained physically and can't be changed

        // The position were there is no clear movement of the servo
        val homePosition = 0.68
        // This is the position were the hood is lifted the most
        val minPosition = 0.53
    }
}
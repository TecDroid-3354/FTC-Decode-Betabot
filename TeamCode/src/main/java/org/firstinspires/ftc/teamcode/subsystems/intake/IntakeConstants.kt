package org.firstinspires.ftc.teamcode.shooter

import com.bylazar.configurables.annotations.Configurable
import com.seattlesolvers.solverslib.controller.PIDFController
import com.seattlesolvers.solverslib.hardware.motors.Motor

class IntakeConstants {

    object Identification {
        // This is the right motor's Id, it needs to be called in the Driver Hub's configuration
        const val rightIntakeMotorId = "rightIntakeMotor"
        // This is the left motor's Id, it needs to be called in the Driver Hub's configuration
        const val leftIntakeMotorId = "leftIntakeMotor"
    }

    object Configuration {

        // This value needs to be measured physically, the number of ticks per motor's revolution
        val ticksPerRevolution = Motor.GoBILDA.BARE.cpr
        // The maximum revolutions per minute the motor can achieve
        val rpm = Motor.GoBILDA.BARE.rpm
        // The motor's behavior when is not given any output
        val zeroPowerBehavior = Motor.ZeroPowerBehavior.FLOAT
        // If the right motor is inverted
        val rightIsInverted = false
        // If the left motor is inverted
        val leftIsInverted = true
    }

    // It will be used to give the motor the correct velocity to be set to, it can be configured through panels
    // in real time that is why it needs @Configurable
    @Configurable
    class PIDF {
        companion object {
            @JvmField
            // The PID controller used for the subsytem's motor
            var pidfController = PIDFController(0.5, 0.0, 0.0, 0.1)
        }
    }
}
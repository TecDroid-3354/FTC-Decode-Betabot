package org.firstinspires.ftc.teamcode.shooter

import com.bylazar.configurables.annotations.Configurable
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.PIDFCoefficients
import com.seattlesolvers.solverslib.controller.PIDController

class ShooterConstants {

    object Identification {
        // This is the motor's Id, it needs to be called in the Driver Hub's configuration
        const val shooterMotorId = "shooterMotor"
    }

    object Configuration {

        // This value needs to be measured physically, the number of ticks per motor's revolution
        const val ticksPerRevolution = 28.0
        // Whether the motor is inverted
        val direction = DcMotorSimple.Direction.REVERSE
        // The motor's behavior when is not given any output
        val zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
        // The motor's run mode, in this case as the shooter does not need any encoder position, is set to
        // run without encoder
        val runMode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
    }

    // It will be used to give the motor the correct velocity to be set to
    @Configurable
    class PIDF {
        companion object {
            @JvmField
            // The PID controller used for the subsytem's motor
            var pidController = PIDFCoefficients(15.0, 0.0, 0.0, 15.0)
        }
    }
}
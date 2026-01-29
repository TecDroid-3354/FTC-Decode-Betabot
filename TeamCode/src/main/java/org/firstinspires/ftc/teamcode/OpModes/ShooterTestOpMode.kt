package org.firstinspires.ftc.teamcode.OpModes

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.seattlesolvers.solverslib.command.CommandOpMode
import com.seattlesolvers.solverslib.command.CommandScheduler
import com.seattlesolvers.solverslib.command.InstantCommand
import com.seattlesolvers.solverslib.command.button.GamepadButton
import com.seattlesolvers.solverslib.gamepad.GamepadEx
import com.seattlesolvers.solverslib.gamepad.GamepadKeys
import com.seattlesolvers.solverslib.gamepad.whenActive
import com.seattlesolvers.solverslib.gamepad.whenInactive
import org.firstinspires.ftc.teamcode.subsystems.shooter.Hood
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter

@TeleOp(name = "ShooterTest", group = "Op Mode")
class ShooterTestOpMode: CommandOpMode() {

    /* ! SET UP CODE ! */

    lateinit var shooter: Shooter
    // Here, declare code to be executed right after pressing the INIT button
    override fun initialize() {

        shooter = Shooter(hardwareMap, telemetry)

        //hood = Hood(hardwareMap, telemetry)

        controller = GamepadEx(gamepad1)

        configureButtonBindings()
    }

    //lateinit var hood: Hood
    lateinit var controller: GamepadEx

    // All control bindings that involve command execution are declared here
    fun configureButtonBindings() {
//        GamepadButton(controller, GamepadKeys.Button.A)
//            .whenPressed(
//                InstantCommand({ hood.modifyCurrentPositionBy(Angle.fromRotations(0.01)) })
//            )
//
//        GamepadButton(controller, GamepadKeys.Button.B)
//            .whenPressed(
//                InstantCommand({ hood.modifyCurrentPositionBy(Angle.fromRotations(-0.01)) })
//            )
//
//        GamepadButton(controller, GamepadKeys.Button.X)
//                InstantCommand({ hood.setHoodPosition(Angle.fromRotations(0.0)) })
//            )
//            .whenPressed(

    }

    // Main code body
    override fun runOpMode() {
        // Code executed at the very beginning, right after hitting the INIT Button
        initialize()

        // Pauses OpMode until the START button is pressed on the Driver Hub
        waitForStart()

        // Run the scheduler
        while (!isStopRequested && opModeIsActive()) {

            // Command for actually running the scheduler
            CommandScheduler.getInstance().run()

            telemetry.update()
        }

        // Cancels all previous commands
        reset()
    }
}
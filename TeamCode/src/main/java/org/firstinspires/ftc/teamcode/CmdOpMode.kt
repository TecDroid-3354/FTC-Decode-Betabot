package org.firstinspires.ftc.teamcode

import com.bylazar.gamepad.Gamepad
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.seattlesolvers.solverslib.command.CommandOpMode
import com.seattlesolvers.solverslib.command.CommandScheduler
import com.seattlesolvers.solverslib.command.InstantCommand
import com.seattlesolvers.solverslib.command.ParallelCommandGroup
import com.seattlesolvers.solverslib.command.button.GamepadButton
import com.seattlesolvers.solverslib.gamepad.GamepadEx
import com.seattlesolvers.solverslib.gamepad.GamepadKeys
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.commands.JoystickCmd
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.SolversMecanum
import org.firstinspires.ftc.teamcode.subsystems.indexer.Indexer
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeDirection


// Personally, I chose to run my code using a command-based Op Mode since it works better for me
// In a regular LinearOpMode, processes are executed in a sequential workflow
// In an OpMode, on the other hand, code is executed through loops

/* To connect to the robot and deploy the code wirelessly, type the following in the terminal:
 *    adb connect 192.168.43.1:5555 (connects to Control Hub)
 *    adb connect 192.168.43.1:8080 (connects to FTC Dashboard)
 * To visit the FTC dashboard online (while connected to the Control Hub's internet)
 *    http://192.168.43.1:8080/?page=connection.html&pop=true
 */
@TeleOp(name = "CMD", group = "Op Mode")
class CMDOpMode : CommandOpMode() {

    /* ! SET UP CODE ! */

    // Declaring subsystems
    lateinit var mecanum: SolversMecanum
    lateinit var intake: Intake
    lateinit var indexer: Indexer

    // Declaring useful components
    lateinit var controller: GamepadEx

    // Here, declare code to be executed right after pressing the INIT button
    override fun initialize() {
        /* Subsystem initialization */

        // Initializing the mecanum & its default command
        mecanum = SolversMecanum(hardwareMap, telemetry)
        mecanum.defaultCommand = JoystickCmd(
            { controller.leftX },
            { controller.leftY },
            { controller.rightX * 0.8 },
            { mecanum.getRobotYaw() },
            mecanum
        )

        intake = Intake(hardwareMap, telemetry)

        indexer = Indexer(hardwareMap, telemetry)

        // Initializing controller & button bindings
        controller = GamepadEx(gamepad1)
        configureButtonBindings()
    }

    // All control bindings that involve command execution are declared here
    fun configureButtonBindings() {
        GamepadButton(controller, GamepadKeys.Button.START)
            .whenPressed(InstantCommand({
                mecanum.resetOtosYaw()
            }))

        GamepadButton(controller, GamepadKeys.Button.A)
            .whenPressed(InstantCommand({
                indexer.rightSlot.feed()
            }))

        GamepadButton(controller, GamepadKeys.Button.B)
            .whenPressed(InstantCommand({
                indexer.rightSlot.home()
            }))

        GamepadButton(controller, GamepadKeys.Button.RIGHT_BUMPER)
            .whenPressed(
                intake.enableBothIntakes()
            ).whenReleased (
                intake.stopBothIntakes()
            )
    }

    fun periodic() {

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
            periodic()

            telemetry.update()
        }

        // Cancels all previous commands
        reset()
    }
}
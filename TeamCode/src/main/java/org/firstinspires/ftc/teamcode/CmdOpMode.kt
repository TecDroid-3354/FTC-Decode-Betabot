package org.firstinspires.ftc.teamcode

import com.qualcomm.hardware.sparkfun.SparkFunOTOS
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.seattlesolvers.solverslib.command.CommandOpMode
import com.seattlesolvers.solverslib.command.CommandScheduler
import com.seattlesolvers.solverslib.command.InstantCommand
import com.seattlesolvers.solverslib.command.button.GamepadButton
import com.seattlesolvers.solverslib.gamepad.GamepadEx
import com.seattlesolvers.solverslib.gamepad.GamepadKeys
import org.firstinspires.ftc.teamcode.commands.JoystickCmd
import org.firstinspires.ftc.teamcode.shooter.Shooter
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.SolversMecanum
import org.firstinspires.ftc.teamcode.subsystems.indexer.Indexer
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake
import org.firstinspires.ftc.teamcode.subsystems.shooter.Hood
import org.firstinspires.ftc.teamcode.subsystems.turret.Turret
import org.firstinspires.ftc.teamcode.vision.Limelight


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
    lateinit var shooter: Shooter
    lateinit var turret: Turret
    lateinit var limelight: Limelight
    lateinit var hood: Hood
    lateinit var otos: SparkFunOTOS

    // Declaring useful components
    lateinit var controller: GamepadEx

    // Here, declare code to be executed right after pressing the INIT button
    override fun initialize() {
        /* Subsystem initialization */

        otos = hardwareMap.get(SparkFunOTOS::class.java, "otos")
        // Initializing the mecanum & its default command
        mecanum = SolversMecanum(hardwareMap, telemetry, otos)
        mecanum.defaultCommand = JoystickCmd(
            { controller.leftX },
            { controller.leftY },
            { controller.rightX * 0.8 },
            mecanum
        )

        intake = Intake(hardwareMap, telemetry)

        indexer = Indexer(hardwareMap, telemetry)

        shooter = Shooter(hardwareMap, telemetry)
        turret = Turret(hardwareMap, telemetry)
        limelight = Limelight(hardwareMap, telemetry, otos)
        limelight.start()

        hood = Hood(hardwareMap, telemetry)

        // Initializing controller & button bindings
        controller = GamepadEx(gamepad1)
        configureButtonBindings()
    }

    // All control bindings that involve command execution are declared here
    fun configureButtonBindings() {
        GamepadButton(controller, GamepadKeys.Button.START)
            .whenPressed(InstantCommand({
                otos.resetTracking()
            }))

//        GamepadButton(controller, GamepadKeys.Button.DPAD_DOWN)
//            .whenPressed(InstantCommand({
//                telemetry.addData("Pattern detected", limelight.getMotifPattern())
//            }))

        GamepadButton(controller, GamepadKeys.Button.LEFT_BUMPER)
            .whenPressed(
                intake.enableBothIntakes()
            ).whenReleased (
                intake.stopBothIntakes()
            )

        GamepadButton(controller, GamepadKeys.Button.RIGHT_BUMPER)
            .whenPressed(InstantCommand({
                //shooter.shoot()
                shooter.shootTest()
            })).whenReleased(InstantCommand({
                shooter.stop()
            }))

        GamepadButton(controller, GamepadKeys.Button.DPAD_UP)
            .whenPressed(indexer.feedAllShooter())

        GamepadButton(controller, GamepadKeys.Button.A)
            .whenPressed(
                InstantCommand({ hood.modifyCurrentPositionBy(0.01) })
            )

        GamepadButton(controller, GamepadKeys.Button.B)
            .whenPressed(
                InstantCommand({ hood.modifyCurrentPositionBy(0.01.unaryMinus()) })
            )

        GamepadButton(controller, GamepadKeys.Button.A)
            .whenPressed(
                InstantCommand({ hood.modifyCurrentPositionBy(0.01) })
            )

        GamepadButton(controller, GamepadKeys.Button.B)
            .whenPressed(
                InstantCommand({ hood.modifyCurrentPositionBy(0.01.unaryMinus()) })
            )
    }

    fun periodic() {
        turret.alignToAprilTag(limelight.getTx())
    }

    // Main code body
    override fun runOpMode() {
        // Code executed at the very beginning, right after hitting the INIT Button
        initialize()

        // Pauses OpMode until the START button is pressed on the Driver Hub
        waitForStart()

        //limelight.getMotifPattern()

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
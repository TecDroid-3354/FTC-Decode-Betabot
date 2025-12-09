package org.firstinspires.ftc.teamcode

import com.qualcomm.hardware.sparkfun.SparkFunOTOS
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.seattlesolvers.solverslib.command.CommandOpMode
import com.seattlesolvers.solverslib.command.CommandScheduler
import com.seattlesolvers.solverslib.command.InstantCommand
import com.seattlesolvers.solverslib.command.SequentialCommandGroup
import com.seattlesolvers.solverslib.command.button.GamepadButton
import com.seattlesolvers.solverslib.command.button.Trigger
import com.seattlesolvers.solverslib.gamepad.GamepadEx
import com.seattlesolvers.solverslib.gamepad.GamepadKeys
import com.seattlesolvers.solverslib.gamepad.whenInactive
import org.firstinspires.ftc.teamcode.commands.JoystickCmd
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.SolversMecanum
import org.firstinspires.ftc.teamcode.subsystems.indexer.MotifPatterns
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake
import org.firstinspires.ftc.teamcode.subsystems.turret.Turret
import org.firstinspires.ftc.teamcode.systems.ShooterSystem
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
enum class Alliance {
    Blue_Alliance,
    Red_Alliance
}
@TeleOp(name = "CMD", group = "Op Mode")
class CMDOpMode : CommandOpMode() {

    /* ! SET UP CODE ! */

    // Declaring subsystems
    lateinit var mecanum: SolversMecanum
    lateinit var intake: Intake
    lateinit var turret: Turret
    lateinit var limelight: Limelight

    lateinit var shooterSystem: ShooterSystem

    // Declaring useful components
    lateinit var controller: GamepadEx
    lateinit var otos: SparkFunOTOS
    lateinit var limelightIdFilter: IntArray
    var alliance: Alliance = Alliance.Blue_Alliance

    // Here, declare code to be executed right after pressing the INIT button
    override fun initialize() {
        /* Subsystem initialization */

        otos = hardwareMap.get(SparkFunOTOS::class.java, "otos")
        // Initializing the mecanum & its default command
        mecanum = SolversMecanum(hardwareMap, telemetry, otos)
        mecanum.defaultCommand = JoystickCmd(
            { controller.leftX },
            { controller.leftY },
            { controller.rightX },
            mecanum
        )

        intake = Intake(hardwareMap, telemetry)

        turret = Turret(hardwareMap, telemetry)
        limelight = Limelight(hardwareMap, telemetry, otos)
        limelight.start()

        shooterSystem = ShooterSystem(hardwareMap, telemetry,
            { limelight.getDistanceToGoal(limelightIdFilter).inches },
            { limelight.llResult != null && limelight.llResult!!.isValid }
        )
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

        GamepadButton(controller, GamepadKeys.Button.LEFT_BUMPER)
            .whenPressed(
                intake.enableBothIntakes(-1.0)
            ).whenReleased (
                intake.stopBothIntakes()
            )

        GamepadButton(controller, GamepadKeys.Button.RIGHT_BUMPER)
            .whenPressed(
                intake.enableBothIntakes()
            ).whenReleased (
                intake.stopBothIntakes()
            )

        GamepadButton(controller, GamepadKeys.Button.A)
            .whenPressed(
                InstantCommand({ shooterSystem.hood.modifyCurrentPositionBy(Angle.fromRotations(0.01)) })
            )

        GamepadButton(controller, GamepadKeys.Button.B)
            .whenPressed(
                InstantCommand({ shooterSystem.hood.modifyCurrentPositionBy(Angle.fromRotations(-0.01)) })
            )

        GamepadButton(controller, GamepadKeys.Button.Y)
            .whenPressed(
                shooterSystem.indexer.feedShooter(limelight.getMotifPattern())
            )

        Trigger({ controller.gamepad.right_trigger > 0.2 })
            .whenActive(shooterSystem.shooter.shootCMD())
            .whenInactive(shooterSystem.stopShooter())


//        Trigger({ controller.gamepad.right_trigger > 0.2 })
//            .whenActive(
//                SequentialCommandGroup(
//                    shooterSystem.ajustHood(),
//                    shooterSystem.shoot(limelight.getMotifPattern())
//                )
//            ).whenInactive(
//                shooterSystem.stopShooter()
//            )

//        Trigger({ controller.gamepad.left_trigger > 0.1 })
//            .whenActive(
//                SequentialCommandGroup(
//                    InstantCommand({ shooterSystem.hood.setHoodPosition(0.73) }),
//                    shooterSystem.shoot(limelight.getMotifPattern())
//                )
//
//            ).whenInactive(
//                shooterSystem.stopShooter()
//            )
    }

    fun periodic() {
        turret.alignToAprilTag(limelight.getAngleToGoal(limelightIdFilter), 3.0 * (if (alliance == Alliance.Blue_Alliance) -1.0 else 1.0))
    }

    // Main code body
    override fun runOpMode() {
        // Code executed at the very beginning, right after hitting the INIT Button
        initialize()

        // select side
        val options = listOf("BlueAlliance", "RedAlliance", "Test")
        var index = 0

        while (!isStarted && !isStopRequested) {
            if (gamepad1.y) index = (index - 1 + options.size) % options.size
            if (gamepad1.a) index = (index + 1) % options.size

            telemetry.addLine("Select the Alliance:")
            for (i in options.indices) {
                if (i == index)
                    telemetry.addLine(" ➤ ${options[i]}")  // seleccionado
                else
                    telemetry.addLine("   ${options[i]}")
            }
            telemetry.update()

            sleep(200) // evita múltiples cambios por una sola pulsación
        }

        limelightIdFilter = when (options[index]) {
            "BlueAlliance" -> intArrayOf(20)
            "RedAlliance" -> intArrayOf(24)
            "Test" -> intArrayOf(20, 24)
            else -> intArrayOf(20, 24)
        }

        alliance = when (options[index]) {
            "BlueAlliance" -> Alliance.Blue_Alliance
            "RedAlliance" -> Alliance.Red_Alliance
            "Test" -> Alliance.Blue_Alliance
            else -> Alliance.Blue_Alliance
        }

        // Pauses OpMode until the START button is pressed on the Driver Hub
        waitForStart()

        //limelight.getMotifPattern()

        // Run the scheduler
        while (!isStopRequested && opModeIsActive()) {

            // Command for actually running the scheduler
            CommandScheduler.getInstance().run()
            periodic()

            telemetry.addData("Pattern", limelight.getMotifPattern())
            telemetry.update()
        }

        // Cancels all previous commands
        reset()
    }
}
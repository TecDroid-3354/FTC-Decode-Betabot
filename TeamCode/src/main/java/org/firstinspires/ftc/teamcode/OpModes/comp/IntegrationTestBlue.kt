package org.firstinspires.ftc.teamcode.OpModes.comp

import Angle
import Distance
import com.qualcomm.hardware.sparkfun.SparkFunOTOS
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.seattlesolvers.solverslib.command.CommandOpMode
import com.seattlesolvers.solverslib.command.CommandScheduler
import com.seattlesolvers.solverslib.command.InstantCommand
import com.seattlesolvers.solverslib.command.ParallelCommandGroup
import com.seattlesolvers.solverslib.command.SequentialCommandGroup
import com.seattlesolvers.solverslib.command.button.GamepadButton
import com.seattlesolvers.solverslib.command.button.Trigger
import com.seattlesolvers.solverslib.gamepad.GamepadEx
import com.seattlesolvers.solverslib.gamepad.GamepadKeys
import com.seattlesolvers.solverslib.gamepad.whileActiveContinuous
import com.seattlesolvers.solverslib.geometry.Vector2d
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.OpModes.otosConfig
import org.firstinspires.ftc.teamcode.commands.JoystickCmd
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.SolversMecanum
import org.firstinspires.ftc.teamcode.subsystems.indexer.MotifPatterns
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake
import org.firstinspires.ftc.teamcode.subsystems.turret.AprilTagVectorLocations
import org.firstinspires.ftc.teamcode.subsystems.turret.AxonTurret
import org.firstinspires.ftc.teamcode.subsystems.turret.turretConfig
import org.firstinspires.ftc.teamcode.systems.shooterSystem.ShooterSystem
import org.firstinspires.ftc.teamcode.utils.Alliance
import org.firstinspires.ftc.teamcode.utils.gyroscopes.Otos
import org.firstinspires.ftc.teamcode.vision.Limelight
import kotlin.math.pow
import kotlin.math.sqrt

@TeleOp(name = "Integrated BLUE", group = "Op Mode")
class IntegrationTestBlue: CommandOpMode() {

    /* ! SET UP CODE ! */
    lateinit var otos: Otos

    lateinit var mecanum: SolversMecanum

    lateinit var intake: Intake

    lateinit var limelight: Limelight

    lateinit var turret: AxonTurret

    lateinit var shooterSystem: ShooterSystem

    var turretTarget: Angle = Angle.fromDegrees(0.0)

    // Change this line if the RED April tag location is needed
    val alliance = Alliance.BLUE

    var targetGoalPosition: Vector2d = Vector2d(0.0, 0.0)
    var targetAprilTagPosition: Vector2d = Vector2d(0.0, 0.0)

    var distanceToAprilTag: Distance = Distance.fromInches(0.0)

    var isLLCMDActive = false

    // Here, declare code to be executed right after pressing the INIT button
    override fun initialize() {

        otos = Otos(hardwareMap, telemetry, otosConfig)
        otos.sensorConfiguration()
        otos.setPosition(SparkFunOTOS.Pose2D(-8.0, -64.0, 0.0))

        mecanum = SolversMecanum(hardwareMap, telemetry, otos)
        mecanum.defaultCommand = JoystickCmd(
            { controller.leftX },
            { controller.leftY },
            { controller.rightX },
            mecanum
        )

        intake = Intake(hardwareMap, telemetry)

        turret = AxonTurret(hardwareMap, telemetry, turretConfig)

        limelight = Limelight(hardwareMap, telemetry, otos)
        limelight.start()

        shooterSystem = ShooterSystem(
            hardwareMap, telemetry,
            { limelight.getDistanceToGoal(intArrayOf(20)).inches },
            { limelight.llResult != null }
        )

        //shooterSystem.shooter.defaultCommand = shooterSystem.shooter.setFlyWheelVelocity(AngularVelocity.fromRpm(1000.0))
        shooterSystem.shooter.setFlyWheelVelocity(AngularVelocity.fromRpm(1000.0)).schedule()

        controller = GamepadEx(gamepad1)

        // Desired goal corner position, used to calculate turret target
        targetGoalPosition =
            if (alliance == Alliance.BLUE) AprilTagVectorLocations.blueGoalCornerVector
            else AprilTagVectorLocations.redGoalCornerVector

        // Desired april tag position, used for calculating inches to april tag
        targetAprilTagPosition =
            if (alliance == Alliance.BLUE) AprilTagVectorLocations.blueAprilTagLocationVector
            else AprilTagVectorLocations.redAprilTagLocationVector

        configureButtonBindings()
    }

    //lateinit var hood: Hood
    lateinit var controller: GamepadEx

    // All control bindings that involve command execution are declared here
    fun configureButtonBindings() {
//        GamepadButton(controller, GamepadKeys.Button.A)
//            .whenPressed(
//                InstantCommand({ shooterSystem.hood.modifyCurrentPositionBy(Angle.fromRotations(0.01)) })
//            )
//
//        GamepadButton(controller, GamepadKeys.Button.B)
//            .whenPressed(
//                InstantCommand({ shooterSystem.hood.modifyCurrentPositionBy(Angle.fromRotations(-0.01)) })
//            )

        GamepadButton(controller, GamepadKeys.Button.RIGHT_BUMPER)
            .whenPressed(
                intake.enableBothIntakes()
            ).whenReleased (
                intake.stopBothIntakes()
            )

        GamepadButton(controller, GamepadKeys.Button.LEFT_BUMPER)
            .whenPressed(
                intake.enableBothOuttakes()
            ).whenReleased (
                intake.stopBothIntakes()
            )

//        GamepadButton(controller, GamepadKeys.Button.DPAD_DOWN)
//            .whenPressed(
//                InstantCommand({ turret.setTurretVoltage(1.0) })
//            )
        // Shooting normally
        Trigger { controller.gamepad.right_trigger > 0.7 }
            .whenActive(
                shooterSystem.shoot(limelight.getMotifPattern())
            )

        Trigger { controller.gamepad.left_trigger > 0.7 }
            .whenActive(
                shooterSystem.shoot(limelight.getMotifPattern(), AngularVelocity.fromRpm(4000.0), Angle.fromRotations(0.90))
            )

        // Shooting from th far launch zone
//        Trigger { controller.gamepad.left_trigger > 0.5 }
//            .whenActive(
//                SequentialCommandGroup(
//                    shooterSystem.shoot(limelight.getMotifPattern(), AngularVelocity.fromRpm(4800.0)),
//                    InstantCommand({ turret.defaultCommand.cancel() }),
//                    InstantCommand({
//                        turret.defaultCommand =
//                            // TODO: Get the actual angle for shooting
//                            turret.setTurretAngle(Angle.fromDegrees(15.0))
//                    })
//                ), true
//            )

//        // Aligning with LL
        Trigger { limelight.llResultIsValid() }
            .whileActiveContinuous(
                SequentialCommandGroup(
                    InstantCommand({ isLLCMDActive = true }),
                    turret.alignToAprilTag { limelight.getFilteredTx(alliance) }
                )
            ).whenInactive(
                SequentialCommandGroup(
                    InstantCommand({ isLLCMDActive = false }),
                    turret.stopTurret()
                )
            )
    }

    private fun calculateTurretTarget() {
        // Get robot's location in a vector
        val robotLocation = otos.getPositionVector()
        // Get the robot's heading
        val robotHeading = otos.getHeading()
        // Get the vector difference from the goal's and robot position
        val newVector = targetGoalPosition - Vector2d(robotLocation.x, robotLocation.y)
        // The angle to the positive x axis of the vector difference
        val angleToGoal = Angle.fromRadians(newVector.angle())
        // Getting the turret angle by subtracting the robot's rotation to the field target angle
        turretTarget = Angle.fromDegrees(
            AngleUnit.normalizeDegrees(angleToGoal.degrees - robotHeading.degrees - 90.0)
        )
    }

    // DO NOT USE AS IT NOT WORKS
    private fun distanceToAprilTagOdometry() {
        // Get robot's location in a vector
        val robotLocationX = -otos.getPositionVector().x

        val robotLocationY = -otos.getPositionVector().y

        distanceToAprilTag = Distance.fromInches(
            sqrt(
                (targetAprilTagPosition.x - robotLocationX).pow(2.0) +
                        (targetAprilTagPosition.y - robotLocationY).pow(2.0)
            )
        )
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
            calculateTurretTarget()
            distanceToAprilTagOdometry()
            shooterSystem.periodic()
            controller.readButtons()

//            shooterSystem.shooter.log()
            telemetry.addData("Distance to Goal LL in", limelight.getDistanceToGoal(intArrayOf(
                when (alliance) {
                    Alliance.BLUE -> 20
                    Alliance.RED -> 24
                }
            )).inches)
            telemetry.update()
        }

        // Cancels all previous commands
        reset()
    }
}
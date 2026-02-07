package org.firstinspires.ftc.teamcode.auto


import Angle
import AngularVelocity
import Distance
import com.pedropathing.follower.Follower
import com.qualcomm.hardware.ams.AMSColorSensor.Wait
import com.qualcomm.hardware.sparkfun.SparkFunOTOS
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.seattlesolvers.solverslib.command.Command
import com.seattlesolvers.solverslib.command.CommandOpMode
import com.seattlesolvers.solverslib.command.CommandScheduler
import com.seattlesolvers.solverslib.command.InstantCommand
import com.seattlesolvers.solverslib.command.ParallelCommandGroup
import com.seattlesolvers.solverslib.command.RunCommand
import com.seattlesolvers.solverslib.command.SequentialCommandGroup
import com.seattlesolvers.solverslib.command.WaitCommand
import com.seattlesolvers.solverslib.command.WaitUntilCommand
import com.seattlesolvers.solverslib.kinematics.wpilibkinematics.ChassisSpeeds
import org.firstinspires.ftc.teamcode.OpModes.otosConfig
import org.firstinspires.ftc.teamcode.auto.Visualizer.Draw
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.SolversMecanum
import org.firstinspires.ftc.teamcode.subsystems.indexer.MotifPatterns
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake
import org.firstinspires.ftc.teamcode.subsystems.turret.AxonTurret
import org.firstinspires.ftc.teamcode.subsystems.turret.turretConfig
import org.firstinspires.ftc.teamcode.systems.shooterSystem.ShooterSystem
import org.firstinspires.ftc.teamcode.utils.gyroscopes.Otos
import org.firstinspires.ftc.teamcode.vision.Limelight

@Autonomous(name = "Shoot-far red", group = "Red")
class KotlinPrecharged: CommandOpMode() {
    //Variables
    var pathState = 0

    /* ! SET UP CODE ! */
    lateinit var otos: Otos

    lateinit var intake: Intake

    lateinit var limelight: Limelight

    lateinit var turret: AxonTurret

    lateinit var shooterSystem: ShooterSystem

    lateinit var mecanum: SolversMecanum

    // Here, declare code to be executed right after pressing the INIT button
    override fun initialize() {

        // Sets otos
        otos = Otos(hardwareMap, telemetry, otosConfig)
        otos.setPosition(SparkFunOTOS.Pose2D(0.0, 0.0, 0.0))

        // Starting Limelight
        limelight = Limelight(hardwareMap, telemetry, otos)
        limelight.start()

        // Initializing subsystems
        intake = Intake(hardwareMap, telemetry)

        turret = AxonTurret(hardwareMap, telemetry, turretConfig)
        turret.defaultCommand = turret.setTurretAngle { Angle.fromDegrees(0.0) }

        shooterSystem = ShooterSystem(
            hardwareMap, telemetry,
            { limelight.getDistanceToGoal(intArrayOf(20, 24)).inches },
            { limelight.llResult != null }
        )

        shooterSystem.shooter.setFlyWheelVelocity(AngularVelocity.fromRpm(1000.0)).schedule()

        mecanum = SolversMecanum(hardwareMap, telemetry, otos)
    }

    // Main code body
    override fun runOpMode() {
        // Code executed at the very beginning, right after hitting the INIT Button
        initialize()

        // Pauses OpMode until the START button is pressed on the Driver Hub
        waitForStart()

        setPathState(0)

        // Run the scheduler
        while (!isStopRequested && opModeIsActive()) {
            // Command for actually running the scheduler
            CommandScheduler.getInstance().run()

            // Actual autonomous sequence
            autonomousPathUpdates()

            // Updating the telemetry
            telemetry.update()
        }

        // Cancels all previous commands
        reset()
    }

    private fun autonomousPathUpdates() {
        when (pathState) {
            // turns to the right
            0 -> {
                RunCommand({
                    mecanum.setChassisSpeeds(ChassisSpeeds(0.0, 0.0, 0.2))
                })
                    .interruptOn { WaitCommand(150).isFinished }
                    //.andThen(WaitCommand(100))
                    //.withTimeout(100) //todo: check this number so that the chassis points toward the goal
//                    .andThen(
//                        RunCommand({
//                            mecanum.setChassisSpeeds(ChassisSpeeds(0.0, 0.0, 0.0))
//                        })
//                    )
                    .andThen(setPathState(1))
                    .schedule()
            }
            // shoots the precharged artifacts
            1 -> {
                RunCommand({
                    shooterSystem.shoot(MotifPatterns.NO_PATTERN_DETECTED)
                })
                    .interruptOn { !shooterSystem.indexer.isFull() }
                    .andThen(WaitCommand(1000))
                    .withTimeout(100)
                    .andThen(setPathState(2))
                    .schedule()
            }
            // turns to the left
            2 -> {
                RunCommand({
                    mecanum.setChassisSpeeds(ChassisSpeeds(0.0, 0.0, 0.2))
                })
                    .interruptOn { WaitCommand(150).isFinished }
                    //.withTimeout(150) //todo: must be the inverse of the timeout at 0
//                    .andThen(
//                        RunCommand({
//                            mecanum.setChassisSpeeds(ChassisSpeeds(0.0, 0.0, 0.0))
//                        })
//                    )
                    .andThen(setPathState(3))
                    .schedule()
            }

            // moves forward
            3 -> {
                SequentialCommandGroup(
                    WaitCommand(2000),
                    RunCommand({
                        mecanum.setChassisSpeeds(ChassisSpeeds(1.0, 0.0, 0.0))
                    })
                )
                    .andThen(WaitCommand(100))
                    .withTimeout(2000)
                    .andThen(setPathState(69))
                    .schedule()
            }

            // turns to the right
            4 -> {
                RunCommand({
                    mecanum.setChassisSpeeds(ChassisSpeeds(0.0, 0.0, 1.0))
                })
                    .withTimeout(100)
                    .withTimeout(500) //todo: check this number so that the chassis points 90 deg to the right
                    .andThen(setPathState(5))
                    .schedule()
            }

            // picks up the bottom row of the artifacts
            5 -> {
                ParallelCommandGroup(
                    intake.enableBothIntakes(),
                    RunCommand({
                        mecanum.setChassisSpeeds(ChassisSpeeds(0.0, 1.0, 0.0))
                    })
                )
                    .withTimeout(2000)
                    .andThen(intake.stopBothIntakes())
                    .andThen(setPathState(6))
                    .schedule()
            }

            // comes back again
            6 -> {

            }
        }
    }

    private fun setPathState(number: Int): Command {
        telemetry.addData("Current path state to be followed", number)
        return InstantCommand({ pathState = number })
    }
}
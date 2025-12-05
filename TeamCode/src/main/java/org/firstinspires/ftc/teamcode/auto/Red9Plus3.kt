package org.firstinspires.ftc.teamcode.auto

import com.pedropathing.follower.Follower
import com.pedropathing.util.Timer
import com.qualcomm.hardware.sparkfun.SparkFunOTOS
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.seattlesolvers.solverslib.command.CommandOpMode
import com.seattlesolvers.solverslib.command.CommandScheduler
import com.seattlesolvers.solverslib.command.InstantCommand
import com.seattlesolvers.solverslib.command.ParallelCommandGroup
import com.seattlesolvers.solverslib.command.SequentialCommandGroup
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake
import org.firstinspires.ftc.teamcode.subsystems.turret.Turret
import org.firstinspires.ftc.teamcode.systems.ShooterSystem
import org.firstinspires.ftc.teamcode.vision.Limelight

@Autonomous(name = "Red 9+3", group = "Red")
class Red9Plus3 : CommandOpMode() {
    /* ! SETUP CODE ! */ // Declares the PedroPathing Follower. This object is in charge of following all paths
    private lateinit var follower: Follower

    // Retrieves paths from the Path class
    private lateinit var paths: Paths

    // This following variable, pathState, will serve as the counter variable to determine
    // which is the next path to follow inside a switch statement
    private var pathState = 0

    // The following timer allows to set a time limit to each path
    private lateinit var pathTimer: Timer

    // Declaring subsystems
    private lateinit var intake: Intake
    private lateinit var shooter: ShooterSystem
    private lateinit var turret: Turret

    // Limelight
    private lateinit var limelight: Limelight
    private lateinit var otos: SparkFunOTOS


    override fun initialize() {
        // The follower is initialized & set to the starting pose
        follower = Constants.createFollower(hardwareMap)
        // Initializes the class where all paths are created
        paths = Paths(follower)
        follower.setStartingPose(paths.red9Plus3Poses.red9Plus3StartPose)
        // Initializes the timer that accounts for the timeout before a path is considered done
        pathTimer = Timer()

        // Initializing useful components
        otos = hardwareMap.get<SparkFunOTOS>(SparkFunOTOS::class.java, "otos")
        limelight = Limelight(hardwareMap, telemetry, otos)


        // Initializing subsystems
        intake = Intake(hardwareMap, telemetry)
        turret = Turret(hardwareMap, telemetry)
        shooter = ShooterSystem(hardwareMap, telemetry,
            { limelight.getDistanceToGoal(intArrayOf(24)) },
            { limelight.llResult != null && limelight.llResult!!.isValid }
        )

    }


    /* ! FUNCTIONAL CODE ! */ // Main code body
    override fun runOpMode() {
        // Code executed at the very beginning, right after hitting the INIT Button
        initialize()

        // Pauses OpMode until the START button is pressed on the Driver Hub
        waitForStart()

        // The following line sets the first path to be followed to be Path number 0
        setPathState(0)

        // Run the scheduler
        while (!isStopRequested && opModeIsActive()) {
            // Command for actually running the scheduler
            CommandScheduler.getInstance().run()

            // Turret following the AprilTags
            turret.alignToAprilTag(limelight.getAngleToGoal(intArrayOf(21)))

            // Actual path following
            follower.update()
            autonomousPathUpdates()

            // Updating the telemetry
            telemetry.update()
        }

        // Cancels all previous commands
        reset()
    }

    // The following class is in charge of changing the followed path. Logic should be added inside
    // here. It is the one called continuously during the autonomous
    fun autonomousPathUpdatesHola() {
        when (pathState) {
            0 -> {
                // Shoot precharged
                SequentialCommandGroup(
                    InstantCommand ({ follower.followPath(paths.red9Plus3ShootPrecharged) }),
                    InstantCommand ({ shooter.shoot(limelight.getMotifPattern()) })
                )
                setPathState(1)
            }

            1 -> if (!follower.isBusy()) {
                // Align itself & pick up the first row of Artifacts
                ParallelCommandGroup(
                    InstantCommand ({ follower.followPath(paths.red9Plus3PickFirstRow) }),
                    InstantCommand ({ intake.enableBothIntakes() })
                )
                setPathState(2)
            }

            2 -> if (!follower.isBusy()) {
                // Shoot that first row of artifacts
                SequentialCommandGroup(
                    InstantCommand ({ intake.stopBothIntakes() }),
                    InstantCommand ({ follower.followPath(paths.red9Plus3ShootFirstRow) }),
                    //InstantCommand({ turret.alignToAprilTag(limelight.getAngleToGoal(intArrayOf(21))) }),
                    InstantCommand ({ shooter.shoot(limelight.getMotifPattern()) })
                )
                setPathState(3)
            }

            3 -> if (!follower.isBusy()) {
                // Align itself and pick the second row of Artifacts
                ParallelCommandGroup(
                    InstantCommand ({ follower.followPath(paths.red9Plus3PickSecondRow) }),
                    InstantCommand ({ intake.enableBothIntakes() })
                )
                setPathState(4)
            }

            4 -> if (!follower.isBusy()) {
                // Shoot that second row of Artifacts
                SequentialCommandGroup(
                    InstantCommand ({ intake.stopBothIntakes() }),
                    InstantCommand ({ follower.followPath(paths.red9Plus3ShootSecondRow) }),
                    InstantCommand ({ shooter.shoot(limelight.getMotifPattern()) })
                )
                setPathState(5)
            }

            5 -> if (!follower.isBusy()) {
                // Align itself and pick up the third row of Artifacts
                ParallelCommandGroup(
                    InstantCommand ({ follower.followPath(paths.red9Plus3PickThirdRow) }),
                    InstantCommand ({ intake.enableBothIntakes() })
                )
                setPathState(6)
            }

            6 -> if (!follower.isBusy()) {
                // Go to its end position
                intake.stopBothIntakes()
                follower.followPath(paths.red9Plus3End)
                setPathState(7)
            }
        }
    }

    fun autonomousPathUpdates() {
        when (pathState) {
            0 -> {
                // Shoot precharged
                follower.followPath(paths.red9Plus3ShootPrecharged)
                setPathState(1)
            }
        }
    }


    // This method will change the number of the target path
    private fun setPathState(number: Int) {
        pathState = number
        pathTimer.resetTimer()
    }
}

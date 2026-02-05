package org.firstinspires.ftc.teamcode.OpModes

import Angle
import com.pedropathing.follower.Follower
import com.pedropathing.geometry.Pose
import com.qualcomm.hardware.sparkfun.SparkFunOTOS
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.seattlesolvers.solverslib.command.CommandOpMode
import com.seattlesolvers.solverslib.command.CommandScheduler
import com.seattlesolvers.solverslib.geometry.Vector2d
import org.firstinspires.ftc.teamcode.auto.Visualizer.Draw
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.utils.gyroscopes.Otos
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

@TeleOp(name = "LimelightCMD", group = "Op Mode")
class LimelightOpMode : CommandOpMode() {

    /* ! SET UP CODE ! */
    lateinit var limelight: Limelight

    lateinit var otos: Otos
    /* ! SETUP CODE ! */

    lateinit var follower: Follower

    var targetAprilTagLocation: Vector2d = Vector2d(0.0, 0.0)

    // Here, declare code to be executed right after pressing the INIT button
    override fun initialize() {
        /* Subsystem initialization */

        // The follower is initialized & set to the starting pose
        follower = Constants.createFollower(hardwareMap)
        follower.setStartingPose(Pose()) //set your starting pose

        // Initializing the OTOS
        otos = Otos(hardwareMap, telemetry, otosConfig)

        // Limelight initialization
        limelight = Limelight(hardwareMap, telemetry, otos)
        limelight.start()

        configureButtonBindings()
    }

    // All control bindings that involve command execution are declared here
    fun configureButtonBindings() {

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

            val limelightPose = limelight.getRobotPoseFromMegaTag2(Angle.fromDegrees(0.0))

            if (limelightPose != null) {
                updateOdometryPose(limelightPose)
            }

            follower.update()

            drawCurrent()

            telemetry.addData("Pattern", limelight.getMotifPattern())
            telemetry.update()
        }

        // Cancels all previous commands
        reset()
    }

    fun updateOdometryPose(pose: SparkFunOTOS.Pose2D) {
        otos.setPosition(pose)
    }


    private fun drawCurrent() {
        try {
            Draw.drawRobot(follower.getPose())
            Draw.sendPacket()
        } catch (e: Exception) {
            throw RuntimeException("Drawing failed " + e)
        }
    }
}
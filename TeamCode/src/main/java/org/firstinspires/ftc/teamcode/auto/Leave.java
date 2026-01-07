package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.util.Timer;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.kinematics.wpilibkinematics.ChassisSpeeds;

import org.firstinspires.ftc.teamcode.subsystems.drivetrain.SolversMecanum;

@Autonomous(name = "Leave", group = "All")
public class Leave extends CommandOpMode {

    /* ! SETUP CODE ! */
    // This following variable, pathState, will serve as the counter variable to determine
    // which is the next path to follow inside a switch statement
    private int pathState;

    // The following timer allows to set a time limit to each path
    private Timer pathTimer;
    private SolversMecanum mecanum;
    private SparkFunOTOS otos;

    @Override
    public void initialize() {
        // Initializes the timer that accounts for the timeout before a path is considered done
        pathTimer = new Timer();

        otos = hardwareMap.get(SparkFunOTOS.class, "otos");
        mecanum = new SolversMecanum(hardwareMap, telemetry, otos);
    }


    /* ! FUNCTIONAL CODE ! */

    // Main code body
    @Override
    public void runOpMode() {
        // Code executed at the very beginning, right after hitting the INIT Button
        initialize();

        // Pauses OpMode until the START button is pressed on the Driver Hub
        waitForStart();

        // The following line sets the first path to be followed to be Path number 0
        setPathState(0);

        // Run the scheduler
        while (!isStopRequested() && opModeIsActive()) {
            // Command for actually running the scheduler
            CommandScheduler.getInstance().run();

            autonomousPathUpdates();

            // Updating the telemetry
            telemetry.update();
        }

        // Cancels all previous commands
        reset();
    }

    public void autonomousPathUpdates() {
        switch (pathState) {
            // ! Shoot precharged artifacts ! //
            case 0: // Path from starting -> shooting position
                //follower.followPath(ball3Path, true);
                mecanum.setChassisSpeeds(new ChassisSpeeds(1.0, 0.0,0.0));
                sleep(800);
                mecanum.setChassisSpeeds(new ChassisSpeeds(0.0, 0.0, 0.0));
                setPathState(1);
                break;
            case 1: // Path from intake position --> shooting position + indexing + shooting
                setPathState(-1);
                break;
        }
    }


    // This method will change the number of the target path
    private void setPathState(int number) {
        pathState = number;
        pathTimer.resetTimer();
        telemetry.addData("Current path state to be followed", number);
    }
}
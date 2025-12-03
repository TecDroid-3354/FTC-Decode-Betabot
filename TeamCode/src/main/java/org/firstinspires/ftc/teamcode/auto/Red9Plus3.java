package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Red 9+3", group = "Red")
public class Red9Plus3 extends CommandOpMode {

    /* ! SETUP CODE ! */

    // Declares the PedroPathing Follower. This object is in charge of following all paths
    private Follower follower;

    // Retrieves paths from the Path class
    private Paths paths;

    // This following variable, pathState, will serve as the counter variable to determine
    // which is the next path to follow inside a switch statement
    private int pathState;

    // The following timer allows to set a time limit to each path
    private Timer pathTimer;


    @Override
    public void initialize() {
        // The follower is initialized & set to the starting pose
        follower = Constants.createFollower(hardwareMap);
        // Initializes the class where all paths are created
        paths = new Paths(follower);
        follower.setStartingPose(paths.red9Plus3Poses.red9Plus3StartPose);
        // Initializes the timer that accounts for the timeout before a path is considered done
        pathTimer = new Timer();
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

            // Actual path following
            follower.update();
            autonomousPathUpdates();

            // Updating the telemetry
            telemetry.update();
        }

        // Cancels all previous commands
        reset();
    }

    // The following class is in charge of changing the followed path. Logic should be added inside
    // here. It is the one called continuously during the autonomous
    public void autonomousPathUpdates() {
        switch (pathState) {
            // The follower is in charge of following a PathChain declared within the Paths object
            case 0:
                follower.followPath(paths.red9Plus3ShootPrecharged);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(paths.red9Plus3PickFirstRow);
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(paths.red9Plus3ShootFirstRow);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(paths.red9Plus3PickSecondRow);
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(paths.red9Plus3ShootSecondRow);
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(paths.red9Plus3PickThirdRow);
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(paths.red9Plus3End);
                    setPathState(7);
                }
                break;
        }
    }

    // This method will change the number of the target path
    private void setPathState(int number) {
        pathState = number;
        pathTimer.resetTimer();
    }
}

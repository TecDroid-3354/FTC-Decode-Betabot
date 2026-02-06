package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;

import org.firstinspires.ftc.teamcode.auto.Visualizer.Draw;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.systems.shooterSystem.ShooterSystem;
import org.firstinspires.ftc.teamcode.vision.Limelight;

@Autonomous(name="red 6 + 3", group = "Red")
public class blue6Plus3 extends CommandOpMode{
    //Pedro ´Pathing
    public static Follower follower;
    public static Paths paths;

    //Variables
    private int pathState;
    private Timer pathTimer;

    //Subsystem Instances

    private ShooterSystem shooterSystem;
    private Intake intake;
    private Limelight limelight;

    @Override
    public void initialize() {
        follower = Constants.createFollower(hardwareMap);
        paths = new Paths(follower);

        pathTimer = new Timer();

        follower.setStartingPose(paths.blue6Plus3Poses.blue6Plus3StartPose);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        waitForStart();

        setPathState(0);

        while (!isStopRequested() && opModeIsActive()) {
            // Command for actually running the scheduler
            CommandScheduler.getInstance().run();

            // Actual path following
            follower.update();
            autonomousPathUpdates();
            drawCurrent();

            // Updating the telemetry
            telemetry.addData("current pathState", pathState);
            telemetry.update();
        }
    }

    public void autonomousPathUpdates() {
        switch (pathState){
            //Go to the point to
            case 0:
                follower.followPath(paths.blue6Plus3ShootPrecharged,true);
                setPathState(1);
                break;

            case 1:
                follower.breakFollowing();
                shootOut();
                setPathState(2);
                break;

            case 2:
                intake.enableBothIntakes();
                follower.followPath(paths.blue6Plus3PickRowOne,true);
                setPathState(3);
                break;

            case 3:
                intake.stopBothIntakes();
                follower.followPath(paths.blue6Plus3ShootRowOne,true);
                setPathState(4);
                break;

            case 4:
                follower.breakFollowing();
                shootOut();
                setPathState(5);
                break;

            case 5:
                intake.enableBothIntakes();
                follower.followPath(paths.blue6Plus3PickRowTwo,true);
                setPathState(6);
                break;

            case 6:
                intake.stopBothIntakes();
                follower.followPath(paths.blue6Plus3ShootRowTwo, true);
                setPathState(7);
                break;

            case 7:
                follower.breakFollowing();
                shootOut();
        }

    }

    private void setPathState(int number) {
        pathState = number;
        pathTimer.resetTimer();
        telemetry.addData("Current path state to be followed", number);
    }

    private static void drawCurrent() {
        try {
            Draw.drawRobot(follower.getPose());
            Draw.sendPacket();
        } catch (Exception e) {
            throw new RuntimeException("Drawing failed " + e);
        }
    }

    private void shootOut(){
        //Activates rollers and feed the shooter with the correct Motif Pattern.
        shooterSystem.shoot(limelight.getMotifPattern()).schedule();
    }
}

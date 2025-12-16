package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;


// We will declare all paths in this class
public class Paths {

    // The flow this follows goes:
    // Pose object --> Path object --> PathChain object

    /* A PathChain object basically represents a transition between points */
    public PathChain testPath1;


    /* ! Actual poses + paths ! */
    // The structure goes as following: we'll declare the key poses individually in another file.
    // Then, we will import those poses to this file and combine them into PathChains.

    // Red side //

    // Red 9 + 3
    Red9Plus3Poses red9Plus3Poses; // Imported poses
    public PathChain red9Plus3ShootPrecharged,
            red9Plus3PickFirstRow, red9Plus3ShootFirstRow,
            red9Plus3PickSecondRow, red9Plus3ShootSecondRow,
            red9Plus3PickThirdRow, red9Plus3ShootThirdRow,
            red9Plus3Test;



    // Blue side //

    // BLue 9 + 3
    Blue9Plus3Poses blue9Plus3Poses; // Imported poses
    public PathChain blue9Plus3ShootPrecharged,
            blue9Plus3PickFirstRow, blue9Plus3ShootFirstRow,
            blue9Plus3PickSecondRow, blue9Plus3ShootSecondRow,
            blue9Plus3PickThirdRow, blue9Plus3ShootThirdRow;


    // In this case, the constructor will only take the Follower, which will allow us to build the paths
    public Paths(Follower follower) {
        // A PathChain is set. We can add multiple paths with the .addPath() method, but we will declare
        // each path individually through one variable to avoid confusion later on
        testPath1 = follower.pathBuilder()
                // .addPath() adds a path or curve to the chain
                .addPath(

                /* This "new BezierCurve" is a Path object, simply declared within the .addPath method
                   There are two types of Path objects:

                   1. BezierCurves (take 3 or more points). The first and last ones are the defined
                      points. The ones in the middle are the control points, which define the curve
                      between the start & end points
                   2. BezierLines (take 2 points). The first and last ones are the defined points,
                      basically a straight line
                 */
                        new BezierCurve(
                                // This "new Pose" is a Pose object, simply declared within the BezierCurve
                                new Pose(14.014, 114.144),
                                new Pose(78.804, 84.694),
                                new Pose(49.200, 36.000)
                        )
                )
                // .setLinearHeadingInterpolation(start, end) basically sets the heading
                // of the robot during the path. @start is the direction it'll start the path facing to
                // & @end is the direction it'll end the path with

                /* Interpolation
                   Interpolation in PathPlanner represents the heading/rotation of the robot during/after a path

                   We have 3 main types of interpolation:
                   1. Linear Heading Interpolation: Basically makes the robot turn from a given startHeading
                      to a given endHeading, for a duration given through endTime.
                   2. Constant Heading Interpolation: The robot's heading stays the same throughout the whole path
                   3. Tangent Heading Interpolation:

                   All measurements should be given in radians!
                */
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                .build(); // Converts from pathBuilder to pathChain


        /* ! Actual paths ! */

        // Red side //

        // Red 9 + 3
        red9Plus3Poses = new Red9Plus3Poses();
        Red9Plus3Paths(follower);


        // Blue side //

        // Blue 9 + 3
        blue9Plus3Poses = new Blue9Plus3Poses();
        Blue9Plus3Paths(follower);
    }

    // Red 9 + 3
    private void Red9Plus3Paths(Follower follower) {
        red9Plus3Test = follower.pathBuilder()
            .addPath(
                new BezierLine(
                    red9Plus3Poses.red9Plus3ShootingPose,
                    red9Plus3Poses.red9Plus3StartPose
                )
            )
            .setLinearHeadingInterpolation(
                red9Plus3Poses.red9Plus3ShootingPose.getHeading(),
                red9Plus3Poses.red9Plus3StartPose.getHeading()
            )
            .build();

        red9Plus3ShootPrecharged = follower.pathBuilder()
            .addPath(
                new BezierLine(
                    red9Plus3Poses.red9Plus3StartPose,
                    red9Plus3Poses.red9Plus3ShootingPose
                )
            )
            .setLinearHeadingInterpolation(
                    red9Plus3Poses.red9Plus3StartPose.getHeading(),
                    red9Plus3Poses.red9Plus3ShootingPose.getHeading())
            .build();

        red9Plus3PickFirstRow = follower.pathBuilder()
            .addPath(
                new BezierLine(
                    red9Plus3Poses.red9Plus3ShootingPose,
                    new Pose(129, 84)
                )
            )
            .setLinearHeadingInterpolation(
                    red9Plus3Poses.red9Plus3ShootingPose.getHeading(),
                    Math.toRadians(0)
            )
            .build();

        red9Plus3ShootFirstRow = follower.pathBuilder()
            .addPath(
                new BezierLine(
                    new Pose(129, 84),
                    red9Plus3Poses.red9Plus3ShootingPose
                )
            )
            .setLinearHeadingInterpolation(
                Math.toRadians(0),
                red9Plus3Poses.red9Plus3ShootingPose.getHeading()
            )
            .build();

        red9Plus3PickSecondRow = follower.pathBuilder()
            .addPath(
                new BezierLine(
                    red9Plus3Poses.red9Plus3ShootingPose,
                    new Pose(103, 59)
                )
            )
            .setLinearHeadingInterpolation(
                    red9Plus3Poses.red9Plus3ShootingPose.getHeading(),
                    Math.toRadians(0)
            )
            .addPath(
                new BezierLine(
                    new Pose(103, 59),
                    new Pose(129, 59)
                )
            )
            .build();

        red9Plus3ShootSecondRow = follower.pathBuilder()
            .addPath(
                new BezierLine(
                    new Pose(129, 59),
                    red9Plus3Poses.red9Plus3ShootingPose
                )
            )
            .setLinearHeadingInterpolation(
                    Math.toRadians(0),
                    red9Plus3Poses.red9Plus3ShootingPose.getHeading()
            )
            .build();

        red9Plus3PickThirdRow = follower.pathBuilder()
            .addPath(
                new BezierLine(
                    red9Plus3Poses.red9Plus3ShootingPose,
                    new Pose(103, 35)
                )
            )
            .setLinearHeadingInterpolation(
                    red9Plus3Poses.red9Plus3ShootingPose.getHeading(),
                    Math.toRadians(0)
            )
            .addPath(
                new BezierLine(
                    new Pose(103, 35),
                    new Pose(129, 35)
                )
            )
            .build();

        red9Plus3ShootThirdRow = follower.pathBuilder()
            .addPath(
                new BezierLine(
                    new Pose(129, 35),
                    red9Plus3Poses.red9Plus3ShootingPose
                )
            )
            .setLinearHeadingInterpolation(
                    Math.toRadians(0),
                    red9Plus3Poses.red9Plus3ShootingPose.getHeading()
            )
            .build();
    }

    // Blue 9 + 3
    private void Blue9Plus3Paths(Follower follower) {
        blue9Plus3ShootPrecharged = follower.pathBuilder()
            .addPath(
                new BezierLine(
                    blue9Plus3Poses.blue9Plus3StartPose,
                    blue9Plus3Poses.blue9Plus3ShootingPose
                )
            )
            .setLinearHeadingInterpolation(
                    blue9Plus3Poses.blue9Plus3StartPose.getHeading(),
                    blue9Plus3Poses.blue9Plus3ShootingPose.getHeading())
            .build();

        blue9Plus3PickFirstRow = follower.pathBuilder()
            .addPath(
                new BezierLine(
                    blue9Plus3Poses.blue9Plus3ShootingPose,
                    new Pose(15, 84)
                )
            )
            .setLinearHeadingInterpolation(
                    blue9Plus3Poses.blue9Plus3ShootingPose.getHeading(),
                    Math.toRadians(180)
            )
            .build();

        blue9Plus3ShootFirstRow = follower.pathBuilder()
            .addPath(
                new BezierLine(
                    new Pose(15, 84),
                    blue9Plus3Poses.blue9Plus3ShootingPose
                )
            )
            .setLinearHeadingInterpolation(
                    Math.toRadians(180),
                    blue9Plus3Poses.blue9Plus3ShootingPose.getHeading()
            )
            .build();

        blue9Plus3PickSecondRow = follower.pathBuilder()
            .addPath(
                new BezierLine(
                    blue9Plus3Poses.blue9Plus3ShootingPose,
                    new Pose(41, 59)
                )
            )
            .setLinearHeadingInterpolation(
                    blue9Plus3Poses.blue9Plus3ShootingPose.getHeading(),
                    Math.toRadians(180)
            )
            .addPath(
                new BezierLine(
                    new Pose(41, 59),
                    new Pose(15, 59)
                )
            )
            .build();

        blue9Plus3ShootSecondRow = follower.pathBuilder()
            .addPath(
                new BezierLine(
                    new Pose(15, 59),
                    blue9Plus3Poses.blue9Plus3ShootingPose
                )
            )
            .setLinearHeadingInterpolation(
                    Math.toRadians(180),
                    blue9Plus3Poses.blue9Plus3ShootingPose.getHeading()
            )
            .build();

        blue9Plus3PickThirdRow = follower.pathBuilder()
            .addPath(
                new BezierLine(
                    blue9Plus3Poses.blue9Plus3ShootingPose,
                    new Pose(41, 35)
                )
            )
            .setLinearHeadingInterpolation(
                    blue9Plus3Poses.blue9Plus3ShootingPose.getHeading(),
                    Math.toRadians(180)
            )
            .addPath(
                new BezierLine(
                    new Pose(41, 35),
                    new Pose(15, 35)
                )
            )
            .build();

        blue9Plus3ShootThirdRow = follower.pathBuilder()
            .addPath(
                new BezierLine(
                    new Pose(15, 35),
                    blue9Plus3Poses.blue9Plus3ShootingPose
                )
            )
            .setLinearHeadingInterpolation(
                    Math.toRadians(180),
                    blue9Plus3Poses.blue9Plus3ShootingPose.getHeading()
            )
            .build();
    }
}

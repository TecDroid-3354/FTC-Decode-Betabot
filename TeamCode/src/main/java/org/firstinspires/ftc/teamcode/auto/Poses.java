package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.geometry.Pose;


// This file does NOT contain any logic, only poses

/* A pose object represents a point related to the center of the robot, in the field.
   From here, we join several Poses together to form a Path object
   Then, after joining several Paths together, we form the PathChain object */

class Red9Plus3Poses {
    final Pose red9Plus3StartPose = new Pose(129, 112, Math.toRadians(0));
    final Pose red9Plus3ShootingPose = new Pose(84, 84, Math.toRadians(50));
}

class Red6Plus3Poses {
    final Pose red6Plus3StartPose = new Pose(88,9,0);
    final Pose red6Plus3ShootingPose = new Pose(85,21,55);
}

class Blue6Plus3Poses{
    final Pose blue6Plus3StartPose = new Pose(55,8,180);
    final Pose blue6Plus3ShootingPose = new Pose(69,27,130);
}

class Blue9Plus3Poses {
    final Pose blue9Plus3StartPose = new Pose(15, 112, Math.toRadians(180   ));
    final Pose blue9Plus3ShootingPose = new Pose(60, 84, Math.toRadians(130));
}
package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;

public class GameTrajectorySequenceBuilder extends TrajectorySequenceBuilder {
    public GameTrajectorySequenceBuilder(Pose2d startPose, Double startTangent, TrajectoryVelocityConstraint baseVelConstraint, TrajectoryAccelerationConstraint baseAccelConstraint, double baseTurnConstraintMaxAngVel, double baseTurnConstraintMaxAngAccel) {
        super(startPose, startTangent, baseVelConstraint, baseAccelConstraint, baseTurnConstraintMaxAngVel, baseTurnConstraintMaxAngAccel);
    }

    public GameTrajectorySequenceBuilder(Pose2d startPose, TrajectoryVelocityConstraint baseVelConstraint, TrajectoryAccelerationConstraint baseAccelConstraint, double baseTurnConstraintMaxAngVel, double baseTurnConstraintMaxAngAccel) {
        super(startPose, baseVelConstraint, baseAccelConstraint, baseTurnConstraintMaxAngVel, baseTurnConstraintMaxAngAccel);
    }
}
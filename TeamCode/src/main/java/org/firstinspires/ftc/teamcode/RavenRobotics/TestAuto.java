package org.firstinspires.ftc.teamcode.RavenRobotics;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.pedro.Constants;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;

import static com.pedropathing.api.Paths.*;
import com.pedropathing.paths.Path;

import com.pedropathing.ivy.Scheduler;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;

@Autonomous
public class TestAuto extends OpMode {
    private Follower follower;
    private final PoseFactory p = PoseFactory.degrees();

    private final Pose startPose = p.of(72,72,90);
    private final Pose scorePose = p.of(120,24,90);
    private final Pose midPose = p.of(24,120,90);
    private final Pose endPose = p.of(24,24,90);

    private Path score() {
        return line(startPose, scorePose).linear(startPose, scorePose);
    }
    private Path score2() {
        return line(scorePose, midPose).linear(scorePose, midPose);
    }
    private Path park() {
        return line(midPose, endPose).linear(midPose, endPose);
    }
    private Command autoRoutine() {
        return sequential(
                follow(follower, score()),
                follow(follower, score2()),
                follow(follower, park())
        );
    }

    @Override
    public void init() {
        Scheduler.reset();

        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);
    }

    @Override
    public void start() {
        schedule(autoRoutine());
    }

    @Override
    public void loop() {
        follower.update();
        Scheduler.execute();

        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData("Heading", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Follower Mode", follower.mode());
        telemetry.update();
    }
}

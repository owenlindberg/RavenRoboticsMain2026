package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }

    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("FLmotor");
        c.frontRightName.set("FRmotor");
        c.backLeftName.set("BLmotor");
        c.backRightName.set("BRmotor");

        c.frontLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);

        c.manualBrakeMode.set(false);
    });

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(2.4476473350224532);
        c.yPodOffset.set(1.4274840467558134);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.27766037058934556);
                Controller secondaryTranslationalForward = Controller.proportional(0.1025880877110193);
                Controller primaryTranslationalLateral = Controller.proportional(0.39335770273189735);
                Controller secondaryTranslationalLateral = Controller.proportional(0.1453351604480405);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.019023002819191698));
                c.brake.set(Controller.proportionalFeedforward(0.01616955239631294));

                c.headingFeedback.set(Controller.proportional(3.590169520427031));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.04135824698771137, 0.014442589166534054));

                c.linearBrakeCoefficients.set(Matrix.diag(0.06852077996595524, 0.05033697129338126));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0032100200798364487, 0.0034743801940016186));

                c.maxAchievableForwardVelocity.set(55.496542477478144);
                c.maxAchievableStrafeVelocity.set(44.76221344466352);
                c.naturalForwardDeceleration.set(51.03262680462412);
                c.naturalStrafeDeceleration.set(57.51906117695031);
            }
    );

}

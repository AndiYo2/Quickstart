package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
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
        c.frontLeftName.set("fl");
        c.frontRightName.set("fr");
        c.backLeftName.set("bl");
        c.backRightName.set("br");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(-1.560289878544845);
        c.yPodOffset.set(-4.27580825925812);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.5547209413331926);
                Controller secondaryTranslationalForward = Controller.proportional(0.20495456540607387);
                Controller primaryTranslationalLateral = Controller.proportional(0.8807899768888864);
                Controller secondaryTranslationalLateral = Controller.proportional(0.3254283613188081);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.018118190158429463));
                c.brake.set(Controller.proportionalFeedforward(0.015400461634665043));

                c.headingFeedback.set(Controller.proportional(6.104266040888877));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.057080808078835414, 0.0061633933877967536));

                c.linearBrakeCoefficients.set(Matrix.diag(0.07713267395708434, 0.04849119102934069));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.002041858401694166, 0.002036405989025004));

                c.maxAchievableForwardVelocity.set(66.41124693259972);
                c.maxAchievableStrafeVelocity.set(45.193138329807);
                c.naturalForwardDeceleration.set(50.95242558189715);
                c.naturalStrafeDeceleration.set(84.8858340891413);

                c.translationalConstraint.set(1.0);          // inches from the end point
                c.headingConstraint.set(Math.toRadians(5));  // radians from the end heading
                c.velocityConstraint.set(3.0);               // in/s, robot must be slower than this
                c.parametricTConstraint.set(0.1);            // how far from t = 1 counts as the end
                c.timeoutConstraint.set(100.0);
            }
    );
}
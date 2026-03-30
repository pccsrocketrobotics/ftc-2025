package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.paths.HeadingInterpolator;

public class PoseUtil {
    public static Pose mirror(Pose p) {
        return new Pose(p.getX(), -p.getY(), -p.getHeading());
    }

    public static HeadingInterpolator facingPoint(double x, double y) {
        return closestPoint -> MathFunctions.normalizeAngle(Math.atan2(
            y - closestPoint.pose.getY(),
            x - closestPoint.pose.getX()
        ));
    }
}

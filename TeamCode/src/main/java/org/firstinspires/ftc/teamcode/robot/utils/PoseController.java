package org.firstinspires.ftc.teamcode.robot.utils;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;

@Configurable
public class PoseController {

    public static double rVM = 0.7; // Значение из твоего кода

    // 🔥 Возвращаем толеранс (буфер расширения зон в дюймах)
    public static double zoneTolerance = 12.0;

    // ==========================================
    // 1. УПРЕЖДЕНИЕ ДВИЖЕНИЯ
    // ==========================================
    public static Pose getFuturePose(Follower follower) {
        return getFuturePose(follower, rVM);
    }

    public static Pose getFuturePose(Follower follower, double timePrediction) {
        Pose currentPose = follower.getPose();
        Vector velocity = follower.getVelocity();

        return new Pose(
                currentPose.getX() + (velocity.getXComponent() * timePrediction),
                currentPose.getY() + (velocity.getYComponent() * timePrediction),
                currentPose.getHeading()
        );
    }

    // ==========================================
    // 2. БОЛЬШАЯ ЗОНА (closeLaunchZone)
    // ==========================================
    public static boolean isInZone(Pose pose) {
        // Базовые точки: (144, 144), (72, 72), (0, 144) + расширение zoneTolerance
        double[] expandedLaunchZone = {
                144.0 + zoneTolerance, 144.0 + zoneTolerance,
                72.0, 72.0 - zoneTolerance,
                -zoneTolerance, 144.0 + zoneTolerance
        };
        return isPointInTriangle(pose.getX(), pose.getY(), expandedLaunchZone);
    }

    // ==========================================
    // 3. МАЛЕНЬКАЯ ЗОНА (farLaunchZone)
    // ==========================================
    public static boolean isInSmallZone(Pose pose) {
        // Базовые точки: (48, 0), (72, 24), (96, 0) + расширение zoneTolerance
        double[] expandedSmallZone = {
                48.0 - zoneTolerance, -zoneTolerance,
                72.0, 24.0 + zoneTolerance,
                96.0 + zoneTolerance, -zoneTolerance
        };
        return isPointInTriangle(pose.getX(), pose.getY(), expandedSmallZone);
    }

    // ==========================================
    // 4. ПРИТЯГИВАНИЕ К ЗОНЕ (SNAPPING - ТОЛЬКО БОЛЬШАЯ)
    // ==========================================
    public static Pose getNearestPose(Pose pose) {
        double px = pose.getX();
        double py = pose.getY();

        // 🔥 ПРОЕКЦИЯ ТОЛЬКО НА БОЛЬШУЮ ЗОНУ (closeLaunchZone)

        // 1. Левая грань большой зоны
        double x1 = 72.0 + (px - py) / 2.0;
        x1 = Math.max(0.0, Math.min(72.0, x1));
        double y1 = 144.0 - x1;
        double dist1 = Math.hypot(px - x1, py - y1);

        // 2. Правая грань большой зоны
        double x2 = (px + py) / 2.0;
        x2 = Math.max(72.0, Math.min(144.0, x2));
        double y2 = x2;
        double dist2 = Math.hypot(px - x2, py - y2);

        // Возвращаем ближайшую точку только на гранях большой зоны
        if (dist1 < dist2) {
            return new Pose(x1, y1, pose.getHeading());
        } else {
            return new Pose(x2, y2, pose.getHeading());
        }
    }

    // ==========================================
    // ВНУТРЕННЯЯ МАТЕМАТИКА ТРЕУГОЛЬНИКОВ
    // ==========================================
    private static boolean isPointInTriangle(double px, double py, double[] t) {
        double d1 = sign(px, py, t[0], t[1], t[2], t[3]);
        double d2 = sign(px, py, t[2], t[3], t[4], t[5]);
        double d3 = sign(px, py, t[4], t[5], t[0], t[1]);

        boolean has_neg = (d1 < 0) || (d2 < 0) || (d3 < 0);
        boolean has_pos = (d1 > 0) || (d2 > 0) || (d3 > 0);

        return !(has_neg && has_pos);
    }

    private static double sign(double p1x, double p1y, double p2x, double p2y, double p3x, double p3y) {
        return (p1x - p3x) * (p2y - p3y) - (p2x - p3x) * (p1y - p3y);
    }
}
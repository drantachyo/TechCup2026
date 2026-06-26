package org.firstinspires.ftc.teamcode.robot.utils;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;

@Configurable
public class PoseController {

    // Множитель времени предсказания (в секундах).
    // 0.7 означает "где робот будет через 0.7 секунд при текущей скорости"
    public static double rVM = 0.65;

    // ==========================================
    // 1. УПРЕЖДЕНИЕ ДВИЖЕНИЯ (KINEMATIC PREDICTION)
    // ==========================================

    /**
     * Возвращает будущую позицию робота на основе его текущей скорости.
     * Именно ЭТУ позицию нужно использовать для авто-прицеливания башни на ходу!
     */
    public static Pose getFuturePose(Follower follower) {
        return getFuturePose(follower, rVM);
    }

    public static Pose getFuturePose(Follower follower, double timePrediction) {
        Pose currentPose = follower.getPose();
        // В Pedro Pathing getVelocity() возвращает Pose, где X и Y - это скорости.
        Vector velocity = follower.getVelocity();

        return new Pose(
                currentPose.getX() + (velocity.getXComponent() * timePrediction),
                currentPose.getY() + (velocity.getYComponent() * timePrediction),
                currentPose.getHeading() // Направление оставляем текущим
        );
    }

    // ==========================================
    // 2. ГЕОЗОНИРОВАНИЕ (GEOFENCING)
    // ==========================================

    // Координаты зон стрельбы (треугольники на поле)
    // Формат: x1, y1, x2, y2, x3, y3
    private static final double[] closeLaunchZone = {144, 144, 72, 72, 0, 144};
    private static final double[] farLaunchZone = {48, 0, 72, 24, 96, 0};

    public static boolean isInZone(Pose pose) {
        return isPointInTriangle(pose.getX(), pose.getY(), closeLaunchZone) ||
                isPointInTriangle(pose.getX(), pose.getY(), farLaunchZone);
    }

    // ==========================================
    // 3. ПРИТЯГИВАНИЕ К ЗОНЕ (SNAPPING)
    // ==========================================

    public static Pose nearBigZonePose(Pose pose) {
        if (isInZone(pose)) return pose;
        double x = 72 + (pose.getX() - pose.getY()) / 2;
        return new Pose(x, 144 - x, pose.getHeading());
    }

    public static Pose nearSmallZonePose(Pose pose) {
        if (isInZone(pose)) return pose;
        double x = 24 + (pose.getX() + pose.getY()) / 2;
        return new Pose(x, x - 48, pose.getHeading());
    }

    public static Pose getNearestPose(Pose pose) {
        // Выбираем ближайшую из двух зон
        if (pose.distanceFrom(nearBigZonePose(pose)) > pose.distanceFrom(nearSmallZonePose(pose))) {
            return nearSmallZonePose(pose);
        }
        return nearBigZonePose(pose);
    }

    // ==========================================
    // ВНУТРЕННЯЯ МАТЕМАТИКА (Заменяет skeletonarmy.marrow)
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
    }}

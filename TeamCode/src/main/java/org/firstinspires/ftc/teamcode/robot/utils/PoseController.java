package org.firstinspires.ftc.teamcode.robot.utils;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;

@Configurable
public class PoseController {

    // Множитель времени предсказания (в секундах).
    // 0.65 означает "где робот будет через 0.65 секунд при текущей скорости"
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

    // Координаты главной зоны стрельбы (большой треугольник на поле)
    // Формат: x1, y1, x2, y2, x3, y3
    private static final double[] closeLaunchZone = {144, 144, 72, 72, 0, 144};

    public static boolean isInZone(Pose pose) {
        // Проверяем нахождение только в большой зоне
        return isPointInTriangle(pose.getX(), pose.getY(), closeLaunchZone);
    }

    // ==========================================
    // 3. ПРИТЯГИВАНИЕ К ЗОНЕ (SNAPPING)
    // ==========================================

    // ==========================================
    // 3. ПРИТЯГИВАНИЕ К ЗОНЕ (SNAPPING)
    // ==========================================

    public static Pose getNearestPose(Pose pose) {
        // Если уже в зоне — никуда ехать не надо
        if (isInZone(pose)) {
            return pose;
        }

        double px = pose.getX();
        double py = pose.getY();

        // 1. Считаем проекцию на ЛЕВУЮ грань (от 0 до 72 по оси X)
        double x1 = 72.0 + (px - py) / 2.0;
        x1 = Math.max(0.0, Math.min(72.0, x1)); // Запрещаем выходить за пределы отрезка
        double y1 = 144.0 - x1;
        double dist1 = Math.hypot(px - x1, py - y1);

        // 2. Считаем проекцию на ПРАВУЮ грань (от 72 до 144 по оси X)
        double x2 = (px + py) / 2.0;
        x2 = Math.max(72.0, Math.min(144.0, x2)); // Запрещаем выходить за пределы отрезка
        double y2 = x2;
        double dist2 = Math.hypot(px - x2, py - y2);

        // 3. Выбираем ту грань, к которой физически ближе ехать
        if (dist1 < dist2) {
            return new Pose(x1, y1, pose.getHeading());
        } else {
            return new Pose(x2, y2, pose.getHeading());
        }
    }

    // ==========================================
    // ВНУТРЕННЯЯ МАТЕМАТИКА
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
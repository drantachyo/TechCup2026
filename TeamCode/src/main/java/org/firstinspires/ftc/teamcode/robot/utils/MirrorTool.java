package org.firstinspires.ftc.teamcode.robot.utils;

import com.pedropathing.geometry.Pose;

public class MirrorTool {

    // 🔥 Смещение для синей стороны (в дюймах).
    // Если робот едет правее, чем нужно (ближе к красной стороне), используй отрицательные значения (например, -2.0)
    public static double BLUE_X_OFFSET = -1.5;
    public static double BLUE_Y_OFFSET = 0.0;

    /**
     * Основной метод с проверкой на стартовую позицию.
     * @param redPose Исходная (красная) позиция.
     * @param isStartPose Если true — оффсет НЕ применяется (чтобы не сдвигать локализацию).
     */
    public static Pose toBlue(Pose redPose, boolean isStartPose) {
        // Отражаем по оси X (Лево-Право)
        double blueX = 144.0 - redPose.getX();

        // Ось Y (вперед-назад)
        double blueY = redPose.getY() + 2.0;

        // 🔥 Если это точка пути (а не физический старт робота), применяем оффсет
        if (!isStartPose) {
            blueX += BLUE_X_OFFSET;
            blueY += BLUE_Y_OFFSET;
        }

        // Отражаем угол
        double blueHeading = Math.PI - redPose.getHeading();

        // Нормализуем угол, чтобы он оставался в правильных пределах от -PI до PI
        while (blueHeading > Math.PI) blueHeading -= 2 * Math.PI;
        while (blueHeading <= -Math.PI) blueHeading += 2 * Math.PI;

        return new Pose(blueX, blueY, blueHeading);
    }

    /**
     * Оставим этот метод для обратной совместимости (по умолчанию считает, что это точка пути)
     */
    public static Pose toBlue(Pose redPose) {
        return toBlue(redPose, false);
    }
}
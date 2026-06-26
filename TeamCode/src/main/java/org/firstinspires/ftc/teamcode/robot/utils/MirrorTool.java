package org.firstinspires.ftc.teamcode.robot.utils;

import com.pedropathing.geometry.Pose;

public class MirrorTool {

    /**
     * Превращает "красную" позицию в "синюю" (Осевая симметрия для INTO THE DEEP).
     * Красная корзина: (144, 144). Синяя корзина: (0, 144).
     */
    public static Pose toBlue(Pose redPose) {
        // Отражаем только по оси X (Лево-Право)
        double blueX = 144.0 - redPose.getX();

        // Ось Y (вперед-назад) остается абсолютно такой же!
        double blueY = redPose.getY();

        // Отражаем угол (0 превращается в 180, 90 остается 90, 45 становится 135)
        double blueHeading = Math.PI - redPose.getHeading();

        // Нормализуем угол, чтобы он оставался в правильных пределах от -PI до PI
        while (blueHeading > Math.PI) blueHeading -= 2 * Math.PI;
        while (blueHeading <= -Math.PI) blueHeading += 2 * Math.PI;

        return new Pose(blueX, blueY, blueHeading);
    }
}
package org.firstinspires.ftc.teamcode.robot.utils;

import com.pedropathing.geometry.Pose;

/**
 * Хранилище глобальных переменных.
 * Статика здесь АБСОЛЮТНО БЕЗОПАСНА, так как не ссылается на моторы или датчики.
 */
public class GlobalState {
    // Координаты по умолчанию (когда робот только включился)
    public static Pose currentPose = new Pose(0, 0, 0);

    // Флаг: был ли запущен Автоном перед ТелеОпом
    public static boolean isAutoBeen = false;

    // 🔥 Флаг альянса (false = Красный, true = Синий)
    public static boolean isBlueAlliance = false;
}
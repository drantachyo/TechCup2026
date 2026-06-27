package org.firstinspires.ftc.teamcode.robot.utils;

import com.pedropathing.geometry.Pose;

/**
 * Хранилище глобальных переменных.
 */
public class GlobalState {
    // Координаты по умолчанию
    public static Pose currentPose = new Pose(0, 0, 0);

    // Флаг: был ли запущен Автоном перед ТелеОпом
    public static boolean isAutoBeen = false;

    // Флаг альянса (false = Красный, true = Синий)
    public static boolean isBlueAlliance = false;

    // 🔥 Новая переменная: Сюда автоном запишет свою последнюю координату перед завершением
    public static Pose lastAutoPose = null;
}
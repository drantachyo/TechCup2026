package org.firstinspires.ftc.teamcode.robot.utils;

import java.util.Map;
import java.util.TreeMap;

/**
 * Класс линейной интерполяции (Look-Up Table).
 * Берет известные точки измерений и плавно вычисляет значения между ними.
 */
public class LUT {
    // TreeMap автоматически сортирует ключи (дистанции) по возрастанию
    private final TreeMap<Double, Double> table = new TreeMap<>();

    /**
     * Добавить контрольную точку.
     * @param input  Входное значение (например, дистанция)
     * @param output Выходное значение (например, скорость или угол)
     */
    public void add(double input, double output) {
        table.put(input, output);
    }

    /**
     * Получить вычисленное значение для любой дистанции.
     */
    public double get(double input) {
        if (table.isEmpty()) return 0.0;

        // Находим ближайшую точку СНИЗУ (floor) и СВЕРХУ (ceiling)
        Map.Entry<Double, Double> floor = table.floorEntry(input);
        Map.Entry<Double, Double> ceiling = table.ceilingEntry(input);

        // Если мы спросили дистанцию меньше самой минимальной в таблице
        if (floor == null) return ceiling.getValue();

        // Если мы спросили дистанцию больше самой максимальной в таблице
        if (ceiling == null) return floor.getValue();

        // Если попали прямо в точку (input точно совпадает с ключом)
        if (floor.getKey().equals(ceiling.getKey())) {
            return floor.getValue();
        }

        // --- ЛИНЕЙНАЯ ИНТЕРПОЛЯЦИЯ (LERP) ---
        // Считаем пропорцию: насколько input сдвинут между floor и ceiling
        double t = (input - floor.getKey()) / (ceiling.getKey() - floor.getKey());

        // Вычисляем итоговое значение
        return floor.getValue() + t * (ceiling.getValue() - floor.getValue());
    }
}
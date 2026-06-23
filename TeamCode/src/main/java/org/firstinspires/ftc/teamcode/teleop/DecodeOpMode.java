package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

// Импортируем твой будущий класс Robot.
// ВНИМАНИЕ: Пока ты не создашь Robot.java на Шаге 3,
// эти строчки будут подчеркнуты красным. Это абсолютно нормально!
import org.firstinspires.ftc.teamcode.robot.Robot;

import java.util.List;

/**
 * Базовый класс для всех TeleOp команды Decode.
 * Берет на себя всю рутину: кэширование, таймеры, обновление геймпадов.
 */
public abstract class DecodeOpMode extends OpMode {

    // Геймпады от FTCLib (забудь про обычные gamepad1/2, используй эти)
    protected GamepadEx base;
    protected GamepadEx helper;

    // Список хабов для быстрого кэширования (оптимизация Loop Time)
    protected List<LynxModule> allHubs;

    // Таймеры для замера времени цикла
    protected ElapsedTime deltaTimer;
    protected double deltaTime;

    // Наш глобальный менеджер робота
    protected Robot robot;

    // === Пустые методы для переопределения в наследниках ===

    // Вызывается один раз при нажатии INIT
    public void onInit() {}

    // Вызывается один раз при нажатии PLAY
    public void onStart() {}

    // ОБЯЗАТЕЛЬНЫЙ метод. Вызывается постоянно во время работы (вместо loop)
    public abstract void onUpdate();

    @Override
    public void init() {
        // Инициализируем умные геймпады FTCLib
        base = new GamepadEx(gamepad1);
        helper = new GamepadEx(gamepad2);

        // МАГИЯ ОПТИМИЗАЦИИ: Включаем ручное кэширование
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        // Получаем доступ к нашему железу (паттерн Singleton)
        robot = Robot.getInstance();
        robot.init(hardwareMap);

        deltaTimer = new ElapsedTime();

        // Вызываем пользовательскую инициализацию из дочернего класса
        onInit();
    }

    @Override
    public void start() {
        deltaTimer.reset();
        onStart();
    }

    @Override
    public void loop() {
        // 1. Считаем время цикла в секундах
        deltaTime = deltaTimer.milliseconds() / 1000.0;
        deltaTimer.reset();

        // 2. Очищаем кэш хабов - СУПЕР ВАЖНО для MANUAL режима!
        // Без этого данные с датчиков просто "зависнут"
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }

        // 3. Обновляем историю нажатий кнопок (чтобы работало wasJustPressed)
        base.readButtons();
        helper.readButtons();

        // 4. Выполняем твою логику управления (ту, что ты напишешь в TeleOp)
        onUpdate();

        // 5. Рассылаем сигнал обновления всем сабсистемам
        robot.periodic();
    }
}
package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.utils.GlobalState;

import java.util.List;

public abstract class DecodeOpMode extends OpMode {

    protected GamepadEx base;
    protected GamepadEx helper;
    protected List<LynxModule> allHubs;
    protected ElapsedTime deltaTimer;
    protected double deltaTime;

    protected Robot robot;

    public void onInit() {}
    public void onStart() {}
    public abstract void onUpdate();

    @Override
    public void init() {
        base = new GamepadEx(gamepad1);
        helper = new GamepadEx(gamepad2);

        // Ручное кэширование хабов
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        // 1. СОЗДАЕМ АБСОЛЮТНО НОВОГО РОБОТА
        robot = new Robot();

        // 2. РЕШАЕМ, КАКИЕ КООРДИНАТЫ ЕМУ ДАТЬ-*
        if (GlobalState.isAutoBeen) {
            // Берем координаты, которые оставил Автоном
            robot.init(hardwareMap, GlobalState.currentPose);
        } else {
            // Автонома не было: ставим жесткие координаты старта ТелеОпа
            Pose teleopStartPose = new Pose(10.5, 8.5, 0);
            robot.init(hardwareMap, teleopStartPose);
        }

        deltaTimer = new ElapsedTime();
        onInit();
    }

    @Override
    public void start() {
        deltaTimer.reset();
        robot.drive.startTeleop();
        onStart();
    }

    @Override
    public void loop() {
        deltaTime = deltaTimer.milliseconds() / 1000.0;
        deltaTimer.reset();

        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }

        base.readButtons();
        helper.readButtons();

        onUpdate();
        robot.periodic();
    }

    @Override
    public void stop() {
        // Сбрасываем флаг при выключении ТелеОпа
        GlobalState.isAutoBeen = false;
    }
}
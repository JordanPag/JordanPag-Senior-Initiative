package org.firstinspires.ftc.teamcode.mechanism;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public interface Mechanism {
    void init(HardwareMap hardwareMap);
    void run(Gamepad gamepad);
    void run(Gamepad gamepad, boolean fieldCentric, boolean wallAvoider, Telemetry telemetry);
}

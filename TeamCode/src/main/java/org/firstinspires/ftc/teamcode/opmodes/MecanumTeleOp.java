package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.mechanism.chassis.MecanumChassis;

@TeleOp(name = "Mecanum OpMode", group = "Remote")
public class MecanumTeleOp extends OpMode {
    ElapsedTime elapsedTime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    MecanumChassis chassis = new MecanumChassis();
    boolean aIsPressed = false;
    boolean bIsPressed = false;
    boolean fieldCentric = true;
    boolean wallAvoider = true;

    @Override
    public void init() {
        // Initialize each mechanism
        chassis.init(hardwareMap);
    }

    @Override
    public void start(){
        // Reset time when start button pressed
        elapsedTime.reset();
    }

    @Override
    public void loop() {
        // Run each mechanism

        // Toggling Field Centric Driving Mode
        if (gamepad2.a && !aIsPressed) {
            aIsPressed = true;
            fieldCentric = !fieldCentric;
        } else if (!gamepad2.a && aIsPressed) {
            aIsPressed = false;
        }

        // Toggling Wall Avoider Driving Mode
        if (gamepad2.b && !bIsPressed) {
            bIsPressed = true;
            wallAvoider = !wallAvoider;
        } else if (!gamepad2.b && bIsPressed) {
            bIsPressed = false;
        }

        chassis.run(gamepad1, fieldCentric, wallAvoider);

        double[] pos = chassis.getPos();
        telemetry.addData("x", pos[0]);
        telemetry.addData("y", pos[1]);
    }
}

package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.Constants.FIELD_LENGTH;
import static org.firstinspires.ftc.teamcode.Constants.FIELD_WIDTH;
import static org.firstinspires.ftc.teamcode.Constants.ROBOT_LENGTH;
import static org.firstinspires.ftc.teamcode.Constants.ROBOT_WIDTH;

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
        chassis.setPos(
                FIELD_WIDTH - ROBOT_WIDTH / 2,
                -FIELD_LENGTH + ROBOT_LENGTH / 2,
                Math.PI / 2
        ); // Starting in the bottom right corner, facing upwards - the center of the field is (0, 0)
        // 0 heading is to the right along the x axis, angles increase counter-clockwise
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

        telemetry.addData("field centric", fieldCentric ? "ON" : "OFF");
        telemetry.addData("wall avoider", wallAvoider ? "ON" : "OFF");
        double[] pos = chassis.getPos();
        telemetry.addData("x", pos[0]);
        telemetry.addData("y", pos[1]);
        telemetry.addData("heading", pos[2]);
    }
}

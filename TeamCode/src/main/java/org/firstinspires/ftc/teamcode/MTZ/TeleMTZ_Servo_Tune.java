package org.firstinspires.ftc.teamcode.MTZ;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="TeleMTZ_Servo_Tune", group ="Test")
//@Disabled
/****
 * Use up and down arrows to tune servo values
 */

public class TeleMTZ_Servo_Tune extends LinearOpMode {
    double pos = 0.5;
    private Servo leftClaw;
    mtzButtonBehavior levelUpStatus = new mtzButtonBehavior();         //Move Hand to Next Level Higher
    mtzButtonBehavior levelDownStatus = new mtzButtonBehavior();         //Move Hand to Next Level Lower
    @Override
    public void runOpMode() throws InterruptedException {
        leftClaw = hardwareMap.servo.get("leftClaw");
        waitForStart();
        while (opModeIsActive()) {
            levelUpStatus.update(gamepad2.dpad_up);             //Move Hand to Next Level Higher
            levelDownStatus.update(gamepad2.dpad_down);             //Move Hand to Next Level Lower
            displayTelemetry();
            if (levelUpStatus.clickedDown) {
                pos = pos + 0.05;
            }
            if (levelDownStatus.clickedDown) {
                pos = pos - 0.05;
            }
            leftClaw.setPosition(pos);
        }
    }
//Telemetry Methods
    public void displayTelemetry() {
        telemetry.clearAll();

            telemetry.addLine()
                    .addData("leftClaw: ", leftClaw.getPosition());

        telemetry.update();
    }

    //End of Telemetry Methods
    //End of Class
}

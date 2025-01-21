package org.firstinspires.ftc.teamcode.MTZ;

import static org.firstinspires.ftc.teamcode.MTZ.mtzConstants_ItD.defaultDriveSpeed;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="pushbot2", group ="A_Top")
//@Disabled
public class pushbot2 extends TeleMTZ_Drive_Controls_ItD {
    public void runOpMode() throws InterruptedException {

        super.controlRobot("Red", "Center Stage R1", defaultDriveSpeed, true, false, false, true);
    }

}

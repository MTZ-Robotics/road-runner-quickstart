package org.firstinspires.ftc.teamcode.MTZ;
import static org.firstinspires.ftc.teamcode.MTZ.mtzConstants_ItD.defaultDriveSpeed;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@TeleOp(name="Blue Right Strafe", group ="A_Top")
//@Disabled
public class Tele_Blue_Right_Strafe extends TeleMTZ_Drive_Controls_ItD {
    public void runOpMode() throws InterruptedException {

        super.controlRobot("Blue", "Center Stage R1", defaultDriveSpeed, true, true, true, true);
    }

}

package org.firstinspires.ftc.teamcode.MTZ;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.io.IOException;

@Autonomous(name="auto2024v2", group ="A_Top")
//@Disabled
public class Auto2024v2 extends AutoControlsMTZ {
    public void runOpMode() throws InterruptedException {
        try {
            super.autoPaths("Red","auto2024v2",true);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

}

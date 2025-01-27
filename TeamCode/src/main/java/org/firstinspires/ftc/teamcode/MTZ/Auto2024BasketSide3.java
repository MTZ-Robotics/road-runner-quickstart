package org.firstinspires.ftc.teamcode.MTZ;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.io.IOException;

@Autonomous(name="basketsidev3", group ="A_Top")
//@Disabled
public class Auto2024BasketSide3 extends AutoControlsMTZ {
    public void runOpMode() throws InterruptedException {
        try {
            super.autoPaths("Red","basketv3",true);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

}

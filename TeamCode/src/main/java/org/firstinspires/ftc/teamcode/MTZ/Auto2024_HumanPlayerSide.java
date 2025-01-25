package org.firstinspires.ftc.teamcode.MTZ;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import java.io.IOException;

@Autonomous(name="Observation Zone 2025", group ="A_Top")
@Disabled
public class Auto2024_HumanPlayerSide extends AutoControlsMTZ {
    public void runOpMode() throws InterruptedException {
        try {
            //Blue will reverse the turns and strafes
            super.autoPaths("Blue","Auto2024RrTest",true);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

}

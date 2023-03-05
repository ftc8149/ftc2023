package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.lang.annotation.Annotation;

public class TeleOpTest implements TeleOp {
    @Override
    public String name() {
        return "TeleOp 1";
    }

    @Override
    public String group() {
        return null;
    }

    @Override
    public Class<? extends Annotation> annotationType() {
        return null;
    }
}

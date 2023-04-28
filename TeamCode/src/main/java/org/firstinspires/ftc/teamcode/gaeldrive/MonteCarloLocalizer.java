package org.firstinspires.ftc.teamcode.gaeldrive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.gaeldrive.geometry.Particle;
import org.firstinspires.ftc.teamcode.gaeldrive.sensors.SensorBuffer;

public class MonteCarloLocalizer {

    Particle[] particles;
    Pose2d  bestPose;
    SensorBuffer sensors;

    public MonteCarloLocalizer(HardwareMap hardwareMap){
        this.sensors = new SensorBuffer(hardwareMap);
    }


    public void initializeParticle (int ParticleCount) {
        Particle[] initParticles = {new Particle(new Pose2d(0,0,0), 0)};
        this.particles = initParticles;
    }

    public void translateParticles () {

    }

    public void resampleParticles () {

    }

    public void weighParticles () {

        if (this.sensors.hasTrackingWheels) {

        }
    }

    public Pose2d getBestPose (){
        return this.bestPose;
    }
}

package org.firstinspires.ftc.teamcode.gaeldrive;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.gaeldrive.geometry.Particle;
import org.firstinspires.ftc.teamcode.gaeldrive.sensors.SensorBuffer;


/**
 * Localizer that uses Monte Carlo Localization (MCL) to get the position of the robot,
 * given a set of sensor values
 */
public class MonteCarloLocalizer implements Localizer {

    Particle[] particles;
    int particleCount = 50;
    
    Pose2d  poseEstimate;
    SensorBuffer sensors;


    public MonteCarloLocalizer(HardwareMap hardwareMap){
        this.sensors = new SensorBuffer(hardwareMap);
    }

    /**
     * Getting and Setting the PoseEstimate
     */
    @Override
    public Pose2d getPoseEstimate() {
        return this.poseEstimate;
    }

    @Override
    public void setPoseEstimate(@NonNull Pose2d pose2d) {
        this.poseEstimate = pose2d;
    }

    @Override
    public Pose2d getPoseVelocity() {
        return null;
    }

    /**
     * Run one localization cycle
     * This function is inherited from the Localizer class and is used by the drive object.
     */
    @Override
    public void update() {
        this.sensors.update();
        poseEstimate = this.sensors.trackingWheelPose;
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
        for (int currentParticle = 0; currentParticle < particleCount; currentParticle++){
            double confidence = 0.0;
            int numberOfSensors = 0;
            if (this.sensors.hasTrackingWheels) {
                //particles[particleNumber]
                numberOfSensors++;
            }
        }
    }
}

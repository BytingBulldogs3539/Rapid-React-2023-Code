// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;

/** Add your docs here. */
public class PIDConstants {
    private double p = 0;
    private double i = 0;
    private double d = 0;
    private double f = 0;

    public PIDConstants(double p) {
        this.p = p;
    }

    public PIDConstants(double p, double i) {
        this.p = p;
        this.i = i;
    }

    public PIDConstants(double p, double i, double d) {
        this.p = p;
        this.i = i;
        this.d = d;
    }

    public PIDConstants(double p, double i, double d, double f) {
        this.p = p;
        this.i = i;
        this.d = d;
        this.f = f;
    }

    // Getter Methods
    /**
     * Returns p value for PID
     * 
     * @return p value for PID
     */
    public double getP() {
        return p;
    }

    /**
     * Returns i value for PID
     * 
     * @return i value for PID
     */
    public double getI() {
        return i;
    }

    /**
     * Returns d value for PID
     * 
     * @return d value for PID
     */
    public double getD() {
        return d;
    }

    /**
     * Returns f value for PID
     * 
     * @return f value for PID
     */
    public double getF() {
        return f;
    }

    // Setter Methods
    /**
     * Sets the value of p for PID
     * 
     * @param p (new value of p)
     */
    public void setP(double p) {
        this.p = p;
    }

    /**
     * Sets the value of i for PID
     * 
     * @param i (new value of i)
     */
    public void setI(double i) {
        this.i = i;
    }

    /**
     * Sets the value of d for PID
     * 
     * @param d (new value of d)
     */
    public void setD(double d) {
        this.d = d;
    }

    /**
     * Sets the value of f for PID
     * 
     * @param f (new value of f)
     */
    public void setF(double f) {
        this.f = f;
    }
}
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.math;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.util.swerve.Util;

/** Add your docs here. */
public class vector {

    public double[] vectorArray;
    public vector(double[] vectorArray){
        this.vectorArray = vectorArray;
    }
    public vector(Translation2d translation){
        this.vectorArray = new double[]{
            translation.getX(),
            translation.getY()
        };
    }

    public vector add(vector b){
        double[] a = vectorArray.clone();
        for(int i = 0; i < vectorArray.length; i++){
            a[i] += b.vectorArray[i];
        }
        return new vector(a);
    }

    public double dot(vector b){
        double[] a = vectorArray.clone();
        double dotProduct = 0;
        for(int i = 0; i < vectorArray.length; i++){
            dotProduct +=a[i] * b.vectorArray[i];
        }
        return dotProduct;
    }
    public vector times(double scale){
        double[] a = vectorArray.clone();
        for(int i = 0; i < vectorArray.length; i++){
            a[i] *= scale;
        }
        return new vector(a);
    }
    public double norm(){
        double total = 0;
        for(int i = 0; i < vectorArray.length; i++){
            total += Math.pow(vectorArray[i], 2);
        }
        return Math.sqrt(total);
    }

    public vector unit(){
        double norm = norm();
        double[] a = vectorArray.clone();
        for(int i = 0; i < vectorArray.length; i++){
            a[i] /= norm + Util.kEpsilon;
        }
        return new vector(a);
    }
}

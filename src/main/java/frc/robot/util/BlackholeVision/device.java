// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.BlackholeVision;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringArrayPublisher;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;

/** Add your docs here. */
public class device {

    NetworkTable deviceTable;
    StringPublisher tagLayout_publisher;
    DoublePublisher tagSize_publisher;
    StringArrayPublisher camerasName_publisher;

    String[] camerasName;

    public device(String deviceName, camera[] cameras, String tagLayoutFile, double tagSize){
        this.deviceTable = NetworkTableInstance.getDefault().getTable(deviceName);

        String tagLayoutString = getTagLayoutString(tagLayoutFile);
        tagLayout_publisher = deviceTable.getStringTopic("tagLayout").publish();
        tagLayout_publisher.set(tagLayoutString);

        tagSize_publisher = deviceTable.getDoubleTopic("tagSize").publish();
        tagSize_publisher.set(tagSize);

        camerasName_publisher = deviceTable.getStringArrayTopic("camerasName").publish();

        this.camerasName = new String[cameras.length];
        for (int i = 0; i < cameras.length; i++ ){
            this.camerasName[i] = cameras[i].getPort();
            cameras[i].setup(deviceTable);
        }

        start();
    }

    private String getTagLayoutString(String fileName) {
        try {
            // Get the path to the deploy directory
            Path deployDirectory = Filesystem.getDeployDirectory().toPath();
            
            // Resolve the JSON file path
            Path jsonPath = deployDirectory.resolve("BlackholeVision/tagLayout/" + fileName + ".json");
            
            // Read JSON file content
            String jsonContent = Files.readString(jsonPath, StandardCharsets.UTF_8);
            return jsonContent;
            

        } catch (IOException e) {
            DriverStation.reportError("Error reading tag layout JSON file: " + e.getMessage(), false);
            return ""; //TODO handle it
        }
    }

    public NetworkTable getDeviceTable(){
        return deviceTable;
    }

    public void start(){
        camerasName_publisher.set(this.camerasName);
    }

}

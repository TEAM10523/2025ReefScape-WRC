package frc.robot.util.BlackholePlanner;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import java.util.Map;
import java.util.NavigableMap;
import java.util.TreeMap;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;

import com.fasterxml.jackson.core.type.TypeReference;
import com.fasterxml.jackson.databind.ObjectMapper;

public class Trajectory2d {
    public NavigableMap<Double, double[]> trajectory = new TreeMap<>();
    public Boolean ended = false;
    public double speed = 1;
    public Trajectory2d(String fileName, double speed){
        trajectory = getTrajectory(fileName);
        this.speed = speed;
    }
    private NavigableMap<Double, double[]> getTrajectory(String fileName) {
        try {
            // Get the path to the deploy directory
            Path deployDirectory = Filesystem.getDeployDirectory().toPath();
            
            // Resolve the JSON file path
            Path jsonPath = deployDirectory.resolve("BlackholePlanner/traj/" + fileName + ".json");
            
            // Read JSON file content
            String jsonContent = Files.readString(jsonPath, StandardCharsets.UTF_8);

            // Jackson ObjectMapper to parse JSON
            ObjectMapper objectMapper = new ObjectMapper();

            // Deserialize JSON into a List of Maps
            List<Map<String, Double>> dataList = objectMapper.readValue(jsonContent, new TypeReference<>() {});

            // Convert List of Maps into a NavigableMap
            NavigableMap<Double, double[]> navigableMap = new TreeMap<>();

            for (Map<String, Double> entry : dataList) {
                double time = entry.remove("t"); // Extract 't' as the key
                double[] values = entry.values().stream().mapToDouble(Double::doubleValue).toArray(); // Convert remaining values to array
                navigableMap.put(time, values);
            }
            return navigableMap;
            

        } catch (IOException e) {
            DriverStation.reportError("Error reading trajectory JSON file: " + e.getMessage(), false);
            return null; //TODO handle it
        }
    }
    // Function to interpolate values for a given query time
    public static double[] getInterpolatedValues(NavigableMap<Double, double[]> map, double queryTime) {
        // Find the closest lower and higher times
        Map.Entry<Double, double[]> lowerEntry = map.floorEntry(queryTime);
        Map.Entry<Double, double[]> higherEntry = map.ceilingEntry(queryTime);

        // If queryTime is exactly at a key, return it directly
        if (lowerEntry != null && lowerEntry.getKey() == queryTime) {
            return lowerEntry.getValue();
        }

        // If there's no lower or higher entry, return the closest available data
        if (lowerEntry == null) return higherEntry.getValue();
        if (higherEntry == null) return lowerEntry.getValue();

        // Extract values for interpolation
        double t1 = lowerEntry.getKey();
        double[] state1 = lowerEntry.getValue();
        double t2 = higherEntry.getKey();
        double[] state2 = higherEntry.getValue();

        // Linear interpolation
        double[] interpolatedValues = new double[state1.length];
        double ratio = (queryTime - t1) / (t2 - t1);

        for (int i = 0; i < state1.length; i++) {
            interpolatedValues[i] = state1[i] + ratio * (state2[i] - state1[i]);
        }

        return interpolatedValues;
    }

    public Setpoint2d getSetpoint(double t){
        t *= speed;
        if(t > trajectory.lastKey()){
            ended = true;
        }else{
            ended = false;
        }
        double[] state = getInterpolatedValues(trajectory, t);
        return new Setpoint2d(state[0], state[1], state[2]*speed, state[3]*speed, state[4]*speed*speed, state[5]*speed*speed);
    }
    public Setpoint2d getStartlpoint(){
        return getSetpoint(0);
    }
    public Setpoint2d getEndpoint(){
        return getSetpoint(1e+100);
    }

    public boolean ended(){
        return ended;
    }
}

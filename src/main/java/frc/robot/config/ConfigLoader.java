package frc.robot.config;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import java.io.File;
import java.io.IOException;
import java.util.Iterator;
import java.util.Map;

public class ConfigLoader {
    private JsonNode root;

    public ConfigLoader(String filePath) {
        try {
            var deployPath = edu.wpi.first.wpilibj.Filesystem.getDeployDirectory().toPath().resolve(filePath);
            ObjectMapper mapper = new ObjectMapper();
            root = mapper.readTree(deployPath.toFile());
        } catch (IOException e) {
            throw new RuntimeException("Failed to load config " + filePath, e);
        }
    }

    public double getDouble(String... path){
        JsonNode node = root;
        for(String p: path){
            node = node.path(p);
        }

        return node.asDouble();
    }

    public int getInt(String... path){
        JsonNode node = root;
        for(String p: path){
            node = node.path(p);
        }

        return node.asInt();
    }

    public boolean getBoolean(String... path){
        JsonNode node = root;
        for(String p: path){
            node = node.path(p);
        }

        return node.asBoolean();
    }

    public void printNestedMap(String... path){
        JsonNode node = root;
        for (String p :path){
            node = node.path(p);
        }

        Iterator<Map.Entry<String, JsonNode>> fields = node.fields();
        while(fields.hasNext()){
            Map.Entry<String, JsonNode> entry = fields.next();
        }
    }
}

import org.junit.Test;

import com.fasterxml.jackson.annotation.JsonAutoDetect.Visibility;
import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.annotation.PropertyAccessor;
import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;

// make a test that does nothing so we just specify this in build.gradle
// (if we specify nothing, we get everything).
public class ObjectMapperTest {
    @Test
    public void doNothing() throws JsonProcessingException {
        ObjectMapper objectMapper = new ObjectMapper().setVisibility(PropertyAccessor.FIELD, Visibility.PUBLIC_ONLY);
        System.out.println (objectMapper.writeValueAsString(new Pose2d()));
        System.out.println (objectMapper.writeValueAsString(new Pose3d()));
    }
}

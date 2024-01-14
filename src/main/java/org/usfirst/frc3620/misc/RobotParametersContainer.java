package org.usfirst.frc3620.misc;

import com.fasterxml.jackson.databind.JavaType;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.type.TypeFactory;

import frc.robot.Robot;
import org.slf4j.Logger;
import org.usfirst.frc3620.logger.EventLogging;

import edu.wpi.first.wpilibj.Filesystem;

import java.io.File;
import java.io.IOException;
import java.lang.reflect.Constructor;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Modifier;
import java.net.NetworkInterface;
import java.net.SocketException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.*;

public class RobotParametersContainer {
    public final static Logger logger = EventLogging.getLogger(RobotParametersBase.class, EventLogging.Level.INFO);

    static ObjectMapper objectMapper = new ObjectMapper();

    public static <T extends RobotParametersBase> Map<String, T> makeParameterMap(List<T> l) {
        Map<String, T> rv = new HashMap<>();
        for (T c : l) {
            rv.put(c.macAddress.toLowerCase(), c);
        }
        return rv;
    }

    static <T extends RobotParametersBase> Map<String, T> readConfiguration(Class<T> parametersClass, Path path) throws IOException {
        String json = Files.readString(path);
        json = Minifier.minify(json);
        // https://stackoverflow.com/a/61154659/17887564
        JavaType javaType = TypeFactory.defaultInstance().constructCollectionType(List.class, parametersClass);
        List<T> list = objectMapper.readValue(json, javaType);

        return makeParameterMap(list);
    }

    public static <T extends RobotParametersBase> T getRobotParameters (Class<T> parametersClass, String filename) {
        Path path = Paths.get(filename);
        return getRobotParameters(parametersClass, path, identifyRoboRIO());
    }

    public static <T extends RobotParametersBase> T getRobotParameters (Class<T> parametersClass) {
        File file = new File(Filesystem.getDeployDirectory(), "robot_parameters.json");
        return getRobotParameters(parametersClass, file.toPath(), identifyRoboRIO());
    }

    @SuppressWarnings("unchecked")
    public static <T extends RobotParametersBase> T getRobotParameters (Class<T> parametersClass, Path path, String mac) {
        if (! RobotParametersBase.class.isAssignableFrom(parametersClass)) {
            logger.error("getRobotParameters needs a subclass of RobotParameters, returning null");
            return null;
        }
        RobotParametersBase rv = null;

        Map<String, T> parameterMap = null;
        logger.info("reading {} from {}", parametersClass.getName(), path);
        try {
            parameterMap = readConfiguration(parametersClass, path);
        } catch (IOException e) {
            logger.error ("can't read configuration at {}", path);
            logger.error ("caused by", e);
        }

        if (parameterMap != null) {
            rv = parameterMap.get(mac.toLowerCase());
            if (rv == null) {
                logger.info ("no entry in {} for \"{}\"", path, mac);
            }
        }
        if (rv == null) {
            if (parametersClass.isMemberClass() && ((parametersClass.getModifiers() & Modifier.STATIC) == 0)) {
                logger.error("Cannot make a default value; this is a non-static inner class");
            } else {
                try {
                    logger.info("making default {}", parametersClass.getName());
                    Constructor<? extends RobotParametersBase> c = parametersClass.getConstructor();
                    rv = c.newInstance();
                } catch (InvocationTargetException | InstantiationException | IllegalAccessException | NoSuchMethodException e) {
                    e.printStackTrace(System.err);
                    logger.error("got exception {}, returning null RobotParameters", e);
                }
            }
        }
        if (rv != null) {
            logger.info("robot parameters {}: {}", rv.getClass(), rv);
            /*
            try {
                // looks like Jackson is not serializing all fields?
                logger.info("robot parameters {}: {}", rv.getClass(), objectMapper.writeValueAsString(rv));
            } catch (JsonProcessingException ex) {
                logger.error("Unable to format JSON of RobotParameters for logging: {}", ex);
            }
            */
        }
        return (T) rv;
    }

    static String roboRIOMacAddress = null;

    public static String identifyRoboRIO() {
        if (roboRIOMacAddress == null) {
            String rv = "(unknown)";
            if (Robot.isSimulation()) {
                rv = "(simulation)";
            } else {
                try {
                    for (Enumeration<NetworkInterface> e = NetworkInterface
                            .getNetworkInterfaces(); e.hasMoreElements(); ) {
                        NetworkInterface network = e.nextElement();
                        byte[] mac = network.getHardwareAddress();
                        if (mac == null) {
                            logger.info("found network {}, no MAC", network.getName());
                        } else {
                            StringBuilder sb = new StringBuilder();
                            for (int i = 0; i < mac.length; i++) {
                                sb.append(String.format("%02X%s", mac[i],
                                        (i < mac.length - 1) ? "-" : ""));
                            }
                            String macString = sb.toString();
                            logger.info("found network {}, MAC address {}", network.getName(), macString);
                            if (network.getName().equals("eth0")) {
                                rv = macString;
                                break;
                            }
                        }
                    }
                } catch (SocketException e) {
                    e.printStackTrace();
                }
            }
            roboRIOMacAddress = rv;
        }
        return roboRIOMacAddress;
    }

}

package org.usfirst.frc3620.misc;

import java.io.File;
import java.io.FileInputStream;
import java.io.InputStream;
import java.util.Properties;

import org.slf4j.Logger;
import org.usfirst.frc3620.logger.EventLogging;
import org.usfirst.frc3620.logger.EventLogging.Level;

import edu.wpi.first.wpilibj.Filesystem;

public class GitNess {
    static Logger logger = EventLogging.getLogger(GitNess.class, Level.INFO);
    private static Properties instance;
    public static Properties gitProperties() {
        if (instance == null) {
            instance = new Properties();
            File propFile = new File(Filesystem.getDeployDirectory(), "git.properties");

            // .close of resourceStream automagically happens
            try (InputStream resourceStream = new FileInputStream(propFile)) {
                instance.load(resourceStream);
            } catch (Exception ex) {
                instance.setProperty("oopsie", ex.toString());
            }
        }
        return instance;
    }

    public static String gitDescription() {
        Properties p = gitProperties();
        StringBuilder sb = new StringBuilder();
        String d;
        
        d = p.getProperty("project.dir");
        if (d == null) {
            d = "unknown-project";
        }
        sb.append(d);
        sb.append(" ");

        d = p.getProperty("git.commit.id.describe");
        if (d == null) {
            d = "unknown";
        }
        sb.append(d);

        d = p.getProperty("git.dirty");
        if (d == null) {
            d = "-unknownDirtyness";
        } else {
            d = Boolean.parseBoolean(d) ? "-dirty" : "";
        }
        sb.append(d);

        d = p.getProperty("git.branch");
        if (d == null) {
            d = " Branch=(null)";
        } else {
            d = " Branch=" + d;
        }
        sb.append(d);

        d = p.getProperty("build.time");
        if (d != null) {
            d = " Build-time=\"" + d + "\"";
        }
        sb.append(d);

        d = p.getProperty("git.build.host");
        if (d != null) {
            d = " Build-host=" + d;
        }
        sb.append(d);

        return sb.toString();
    }

    public static String gitString() {
        return gitProperties().toString();
    }

}

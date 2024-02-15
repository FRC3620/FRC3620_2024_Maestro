package org.usfirst.frc3620.logger;

import java.io.File;
import java.util.Date;

public interface IDataLogger {
    public void setLoggingDirectory(File loggingDirectory);

    public void setFilename(String filename);

    public void setFilenameTimestamp(Date ts);

    public void setInterval(double seconds);

    public double getInterval();

    public void addPrelude(DataLoggerPrelude prelude);

    public void addPostlude(DataLoggerPostlude postlude);

    public void addDataProvider(String name, 
            IDataLoggerDataProvider iDataLoggerDataProvider);

    public void addMetadata(String s, double d);

    public void addMetadata(String s, String v);

    public String start();

}

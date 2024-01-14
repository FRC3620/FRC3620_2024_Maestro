package org.usfirst.frc3620.logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.slf4j.Logger;

import java.io.BufferedOutputStream;
import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.concurrent.ArrayBlockingQueue;

public class AsyncDataLogger<T extends AsyncDataLoggerDatum> extends SubsystemBase {
  ArrayBlockingQueue<AsyncDataLoggerMessage> queue;
  int droppedMessageCount = 0;
  int sentMessageCount = 0;
  Logger logger = EventLogging.getLogger(getClass(), EventLogging.Level.INFO);

  public AsyncDataLogger(String filename, int capacity) {
    queue = new ArrayBlockingQueue<>(capacity);

    File file = new File(LoggingMaster.getLoggingDirectory(), filename);
    
    Thread t = new Thread(() -> { writerThread(file); });
    t.setName("AsyncDataLogger:" + filename);
    t.setDaemon(true);
    t.start();
  }

  boolean do_queue(AsyncDataLoggerMessage m) {
    if (queue.remainingCapacity() > 0) {
      queue.add(m);
      sentMessageCount++;
      return true;
    } else {
      droppedMessageCount++;
      return false;
    }
  }

  public void send(T o) {
    do_queue(new AsyncDataLoggerMessage(MessageType.DATA, o));
  }

  public void flush() {
    do_queue(new AsyncDataLoggerMessage(MessageType.FLUSH, null));
  }

  public void close() {
    boolean sent = do_queue(new AsyncDataLoggerMessage(MessageType.CLOSE, null));
    if (!sent) {
      logger.error ("Unable to send CLOSE message");
    }
    logger.info ("{} messages sent, {} dropped", sentMessageCount, droppedMessageCount);
  }

  enum MessageType { DATA, FLUSH, CLOSE }

  class AsyncDataLoggerMessage {
    MessageType messageType;
    T payload;

    AsyncDataLoggerMessage(MessageType messageType, T payload) {
      this.messageType = messageType;
      this.payload = payload;
    }
  }

  public int getDroppedMessageCount() {
    return droppedMessageCount;
  }

  public int getSentMessageCount() {
    return sentMessageCount;
  }

  void writerThread(File file) {
    int receivedMessageCount = 0;
    try {
      BufferedOutputStream bos = new BufferedOutputStream(new FileOutputStream(file));
      boolean running = true;
      while (running) {
        AsyncDataLoggerMessage m = queue.take();
        receivedMessageCount++;
        MessageType mt = m.messageType;
        switch (mt) {
          case FLUSH:
            bos.flush();
            break;
          case CLOSE:
            bos.close();
            running = false;
            break;
          case DATA:
            T payload = m.payload;
            if (payload != null) {
              bos.write(payload.getAsyncDataLoggerBytes());
            }
            break;
        }
      }
    } catch (IOException | InterruptedException ex) {
      throw new RuntimeException("writer thread choked", ex);
    }
    logger.info ("{} messages received", receivedMessageCount);
  }
}

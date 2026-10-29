package btools.util;

import java.io.File;

/** Browser replacement: thread dumps are not available in a TeaVM build. Shadows the brouter-util class. */
public class StackSampler {
  public StackSampler(File logfile, int interval) {
  }

  public void start() {
  }

  public void close() {
  }
}

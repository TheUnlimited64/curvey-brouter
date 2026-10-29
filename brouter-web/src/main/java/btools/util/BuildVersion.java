package btools.util;

/** Browser replacement: no jar manifest in a TeaVM build (the JVM test run has none either). Shadows the brouter-util class. */
public final class BuildVersion {
  private BuildVersion() {
  }

  public static String get() {
    return null;
  }
}

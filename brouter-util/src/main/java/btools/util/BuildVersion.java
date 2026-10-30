package btools.util;

/** The BRouter version from the jar manifest. A seam so builds without manifests (browser) can replace it. */
public final class BuildVersion {
  private BuildVersion() {
  }

  public static String get() {
    return BuildVersion.class.getPackage().getImplementationVersion();
  }
}

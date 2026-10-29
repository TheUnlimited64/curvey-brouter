package btools.web;

import org.teavm.jso.JSExport;
import org.teavm.runtime.fs.VirtualFileSystemProvider;

/** Browser entry point for spike #259: exports {@code route(...)}; files come from {@code globalThis.brouterFs}. */
public final class TeaVmMain {
  private TeaVmMain() {
  }

  public static void main(String[] args) {
    VirtualFileSystemProvider.setInstance(new JsFileSystem());
  }

  @JSExport
  public static String route(String lonlats, String profilePath, String segmentDir, String extraParams) {
    return WebRouter.route(lonlats, profilePath, segmentDir, extraParams);
  }
}

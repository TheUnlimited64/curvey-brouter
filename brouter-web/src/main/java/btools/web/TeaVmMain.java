package btools.web;

import org.teavm.jso.JSExport;
import org.teavm.runtime.fs.VirtualFileSystemProvider;

/** Browser entry point: exports {@link #route}; every file read goes through {@link JsFileSystem}. */
public final class TeaVmMain {
  private TeaVmMain() {
  }

  public static void main(String[] args) {
    VirtualFileSystemProvider.setInstance(new JsFileSystem());
  }

  /** See {@link WebRouter#route}. Blocking; run it in a Web Worker. */
  @JSExport
  public static String route(String lonlats, String profilePath, String segmentDir, String requestParams,
                             String profileParams) {
    return WebRouter.route(lonlats, profilePath, segmentDir, requestParams, profileParams);
  }
}

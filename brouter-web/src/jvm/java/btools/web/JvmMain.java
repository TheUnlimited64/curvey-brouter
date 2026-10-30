package btools.web;

import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Paths;

/**
 * Reference CLI: runs {@link WebRouter#route} on the JVM and writes the GeoJSON, so the browser build
 * can be compared byte for byte. Args: lonlats profile segmentDir out.json [requestParams [profileParams]].
 */
public final class JvmMain {
  private JvmMain() {
  }

  public static void main(String[] args) throws Exception {
    String requestParams = args.length > 4 ? args[4] : "";
    String profileParams = args.length > 5 ? args[5] : "";
    String json = WebRouter.route(args[0], args[1], args[2], requestParams, profileParams);
    Files.write(Paths.get(args[3]), json.getBytes(StandardCharsets.UTF_8));
  }
}

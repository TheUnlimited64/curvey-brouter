package btools.web;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.io.File;
import java.net.URL;

import org.junit.Test;

/** {@link WebRouter} on the Dreieich test segments the map-creator tests generate. */
public class WebRouterTest {
  private static File workingDir() {
    URL resulturl = WebRouterTest.class.getResource("/");
    return new File(resulturl.getFile());
  }

  private static String segments() {
    return new File(workingDir(), "../../../../../brouter-map-creator/build/resources/test/tmp/segments").getPath();
  }

  private static String profile(String name) {
    return new File(workingDir(), "../../../../../misc/profiles2/" + name + ".brf").getPath();
  }

  @Test
  public void routesToGeoJson() {
    String json = WebRouter.route("8.720897,50.002515|8.723658,49.997510", profile("trekking"), segments(), "", "");

    assertTrue(json, json.contains("\"type\": \"FeatureCollection\""));
    assertTrue(json, json.contains("\"track-length\""));
  }

  @Test
  public void appliesRequestAndProfileParameters() {
    String plain = WebRouter.route("8.723037,50.000491|8.712737,50.002899", profile("trekking"), segments(), "", "");
    String tuned = WebRouter.route("8.723037,50.000491|8.712737,50.002899", profile("trekking"), segments(),
      "alternativeidx=0", "avoid_unsafe=1");

    assertTrue(plain, plain.contains("\"features\""));
    assertTrue(tuned, tuned.contains("\"features\""));
  }

  @Test
  public void reportsAMissingSegmentAsErrorObject() {
    String json = WebRouter.route("8.720897,50.002515|16.723658,49.997510", profile("trekking"), segments(), "", "");

    assertTrue(json, json.startsWith("{\"error\":\""));
    assertTrue(json, json.contains("\"unreachableWaypointIndex\":"));
    assertTrue(json, json.endsWith(",\"uTurnBlockedWaypointIndex\":-1}"));
  }

  @Test
  public void escapesErrorMessages() {
    assertEquals("{\"error\":\"a \\\"b\\\" \\\\ \\u000a\",\"unreachableWaypointIndex\":2,\"uTurnBlockedWaypointIndex\":-1}",
      WebRouter.error("a \"b\" \\ \n", 2, -1));
  }
}

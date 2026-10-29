package btools.web;

import java.io.File;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import btools.router.FormatJson;
import btools.router.OsmNodeNamed;
import btools.router.RoutingContext;
import btools.router.RoutingEngine;
import btools.router.RoutingParamCollector;

/**
 * One engine run, the same steps as Curvey's Android {@code BRouterEngineRunner}: waypoints, a profile file
 * and AIDL-style parameters in, BRouter GeoJSON out. Shared by the JVM baseline and the browser build.
 */
public final class WebRouter {
  /** Engine wall-clock limit, as on Android. */
  public static final long MAX_RUNNING_TIME_MS = 30_000L;

  private WebRouter() {
  }

  /**
   * @param lonlats     {@code lon,lat|lon,lat|...}
   * @param profilePath profile file; {@code lookups.dat} must sit next to it
   * @param segmentDir  directory with the {@code .rd5} files
   * @param extraParams {@code avoid_motorways=1&...} profile variable overrides, may be empty
   * @return BRouter GeoJSON, or {@code {"error": "..."}}
   */
  public static String route(String lonlats, String profilePath, String segmentDir, String extraParams) {
    RoutingParamCollector collector = new RoutingParamCollector();
    List<OsmNodeNamed> waypoints = collector.getWayPointList(lonlats);
    RoutingContext context = new RoutingContext();
    context.localFunction = profilePath;
    Map<String, String> params = new HashMap<>();
    params.put("timode", "3");
    collector.setParams(context, waypoints, params);
    if (extraParams != null && !extraParams.isEmpty()) {
      try {
        collector.setProfileParams(context, collector.getUrlParams(extraParams));
      } catch (java.io.UnsupportedEncodingException e) {
        return "{\"error\":\"bad extra params\"}";
      }
    }
    RoutingEngine engine = new RoutingEngine(null, null, new File(segmentDir), waypoints, context);
    engine.quite = true;
    engine.doRun(MAX_RUNNING_TIME_MS);
    if (engine.getErrorMessage() != null) {
      return "{\"error\":\"" + engine.getErrorMessage().replace("\"", "'") + "\"}";
    }
    return new FormatJson(context).format(engine.getFoundTrack());
  }
}

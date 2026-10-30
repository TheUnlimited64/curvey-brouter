package btools.web;

import java.io.File;
import java.io.UnsupportedEncodingException;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import btools.router.FormatJson;
import btools.router.OsmNodeNamed;
import btools.router.RoutingContext;
import btools.router.RoutingEngine;
import btools.router.RoutingParamCollector;

/**
 * One engine run, the same steps as Curvey's Android {@code BRouterEngineRunner}: waypoints, a profile
 * file and request parameters in, BRouter GeoJSON out. Shared by the browser build and the JVM.
 */
public final class WebRouter {
  /** Engine wall-clock limit; matches Curvey's Android embedded engine. */
  public static final long MAX_RUNNING_TIME_MS = 30_000L;

  private WebRouter() {
  }

  /**
   * @param lonlats       {@code lon,lat|lon,lat|...}
   * @param profilePath   profile file; {@code lookups.dat} must sit next to it
   * @param segmentDir    directory with the {@code .rd5} files
   * @param requestParams URL-encoded {@code key=value&...} request parameters the engine's
   *                      {@link RoutingParamCollector} understands ({@code timode}, {@code heading},
   *                      {@code nogos}, {@code alternativeidx}, ...); may be empty
   * @param profileParams URL-encoded {@code key=value&...} profile variable overrides
   *                      ({@code avoid_motorways=1}); may be empty
   * @return BRouter GeoJSON, or an error object (see {@link #error})
   */
  public static String route(String lonlats, String profilePath, String segmentDir, String requestParams,
                             String profileParams) {
    RoutingParamCollector collector = new RoutingParamCollector();
    List<OsmNodeNamed> waypoints = collector.getWayPointList(lonlats);
    RoutingContext context = new RoutingContext();
    context.localFunction = profilePath;
    try {
      Map<String, String> params = new HashMap<>();
      params.put("timode", "3");
      if (!isEmpty(requestParams)) {
        params.putAll(collector.getUrlParams(requestParams));
      }
      collector.setParams(context, waypoints, params);
      if (!isEmpty(profileParams)) {
        collector.setProfileParams(context, collector.getUrlParams(profileParams));
      }
    } catch (UnsupportedEncodingException e) {
      return error("bad parameters: " + e.getMessage(), -1, -1);
    }
    RoutingEngine engine = new RoutingEngine(null, null, new File(segmentDir), waypoints, context);
    engine.quite = true;
    engine.doRun(MAX_RUNNING_TIME_MS);
    if (engine.getErrorMessage() != null) {
      return error(engine.getErrorMessage(), engine.getUnreachableWaypointIndex(), engine.getReverseGuardBlockedWaypointIndex());
    }
    if (engine.getFoundTrack() == null) {
      return error("no track found", -1, -1);
    }
    return new FormatJson(context).format(engine.getFoundTrack());
  }

  /**
   * {@code {"error": message, "unreachableWaypointIndex": i, "uTurnBlockedWaypointIndex": j}}; an index is
   * -1 when it does not apply. The indexes are the engine's typed errors (Curvey #201, #228).
   */
  static String error(String message, int unreachableWaypointIndex, int uTurnBlockedWaypointIndex) {
    return "{\"error\":\"" + escape(message) + "\",\"unreachableWaypointIndex\":" + unreachableWaypointIndex
      + ",\"uTurnBlockedWaypointIndex\":" + uTurnBlockedWaypointIndex + "}";
  }

  private static boolean isEmpty(String value) {
    return value == null || value.isEmpty();
  }

  private static String escape(String value) {
    StringBuilder out = new StringBuilder(value.length());
    for (int i = 0; i < value.length(); i++) {
      char c = value.charAt(i);
      if (c == '"' || c == '\\') {
        out.append('\\').append(c);
      } else if (c < 0x20) {
        out.append(String.format("\\u%04x", (int) c));
      } else {
        out.append(c);
      }
    }
    return out.toString();
  }
}

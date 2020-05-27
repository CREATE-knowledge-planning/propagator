import cesiumlanguagewriter.*;
import com.google.gson.*;
import handler.SaveStateStepHandler;
import org.hipparchus.geometry.euclidean.threed.Rotation;
import org.hipparchus.geometry.euclidean.threed.Vector3D;
import org.hipparchus.util.FastMath;
import org.orekit.attitudes.Attitude;
import org.orekit.attitudes.AttitudeProvider;
import org.orekit.attitudes.NadirPointing;
import org.orekit.attitudes.YawCompensation;
import org.orekit.bodies.BodyShape;
import org.orekit.bodies.GeodeticPoint;
import org.orekit.bodies.OneAxisEllipsoid;
import org.orekit.data.DataContext;
import org.orekit.data.DataProvidersManager;
import org.orekit.data.DirectoryCrawler;
import org.orekit.frames.Frame;
import org.orekit.frames.FramesFactory;
import org.orekit.frames.TopocentricFrame;
import org.orekit.geometry.fov.CircularFieldOfView;
import org.orekit.geometry.fov.DoubleDihedraFieldOfView;
import org.orekit.geometry.fov.FieldOfView;
import org.orekit.propagation.Propagator;
import org.orekit.propagation.analytical.tle.TLE;
import org.orekit.time.AbsoluteDate;
import org.orekit.time.TimeScale;
import org.orekit.time.TimeScalesFactory;
import org.orekit.utils.Constants;
import org.orekit.utils.IERSConventions;
import seakers.orekit.coverage.access.RiseSetTime;
import seakers.orekit.coverage.access.TimeIntervalArray;
import seakers.orekit.coverage.access.TimeIntervalMerger;
import seakers.orekit.event.EventAnalysis;
import seakers.orekit.event.EventAnalysisEnum;
import seakers.orekit.event.EventAnalysisFactory;
import seakers.orekit.event.FieldOfViewEventAnalysis;
import seakers.orekit.examples.CoverageExample;
import seakers.orekit.object.*;
import seakers.orekit.propagation.PropagatorFactory;
import seakers.orekit.propagation.PropagatorType;
import seakers.orekit.scenario.Scenario;
import seakers.orekit.util.OrekitConfig;

import java.awt.*;
import java.io.*;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.*;
import java.util.logging.Level;
import java.util.logging.Logger;

public class CZMLGenerator {
    public static void configureOrekit() {
        // configure Orekit
        File home       = new File(System.getProperty("user.home"));
        File orekitData = new File(home, "orekit-data");
        if (!orekitData.exists()) {
            System.err.format(Locale.US, "Failed to find %s folder%n",
                    orekitData.getAbsolutePath());
            System.err.format(Locale.US, "You need to download %s from %s, unzip it in %s and rename it 'orekit-data' for this code to work%n",
                    "orekit-data-master.zip", "https://gitlab.orekit.org/orekit/orekit-data/-/archive/master/orekit-data-master.zip",
                    home.getAbsolutePath());
            System.exit(1);
        }
        DataProvidersManager manager = DataContext.getDefault().getDataProvidersManager();
        manager.addProvider(new DirectoryCrawler(orekitData));
        //if running on a non-US machine, need the line below
        Locale.setDefault(new Locale("en", "US"));
    }

    public static void main(String[] args) {
        configureOrekit();

        // 1. Load JSON files with propagation information
        JsonParser parser = new JsonParser();
        Path missionPath = Paths.get(System.getProperty("user.dir"),"int_files", "mission.json");
        Path satellitesPath = Paths.get(System.getProperty("user.dir"),"int_files", "satellites.json");
        JsonObject missionJson = null;
        JsonArray satellitesJson = null;
        try {
            BufferedReader missionFile = Files.newBufferedReader(missionPath);
            BufferedReader satellitesFile = Files.newBufferedReader(satellitesPath);
            missionJson = parser.parse(missionFile).getAsJsonObject();
            satellitesJson = parser.parse(satellitesFile).getAsJsonArray();
        }
        catch (IOException e) {
            e.printStackTrace();
            System.exit(1);
        }

        int processors = Runtime.getRuntime().availableProcessors();
        OrekitConfig.init(processors-3);

        // 2. Get the start and end dates from an observation in the mission
        JsonArray observations = missionJson.getAsJsonArray("observations");
        String startDateString = observations.get(0).getAsJsonObject().get("startDate").getAsString();
        String endDateString = observations.get(0).getAsJsonObject().get("endDate").getAsString();

        // 3. Define the start and end date of the simulation
        TimeScale utc = TimeScalesFactory.getUTC();
        AbsoluteDate startDate = new AbsoluteDate(startDateString, utc);
        AbsoluteDate endDate = new AbsoluteDate(endDateString, utc);

        // Define the scenario parameters
        Frame earthFrame = FramesFactory.getITRF(IERSConventions.IERS_2003, true);
        Frame inertialFrame = FramesFactory.getEME2000();
        BodyShape earthShape = new OneAxisEllipsoid(Constants.WGS84_EARTH_EQUATORIAL_RADIUS,
                Constants.WGS84_EARTH_FLATTENING, earthFrame);

        // Cesium stuff
        StringWriter stringWriter = new StringWriter();
        CesiumOutputStream output = new CesiumOutputStream(stringWriter);
        output.setPrettyFormatting(true);

        CesiumStreamWriter stream = new CesiumStreamWriter();

        // Write document first
        try (PacketCesiumWriter packet = stream.openPacket(output)) {
            packet.writeId("document");
            packet.writeVersion("1.0");
            try (ClockCesiumWriter clock = packet.openClockProperty()) {
                double startSeconds = startDate.durationFrom(AbsoluteDate.JULIAN_EPOCH);
                int startDays = (int)startSeconds % (24*3600);
                double startDaySeconds = startSeconds - startDays*24*3600;
                double endSeconds = endDate.durationFrom(AbsoluteDate.JULIAN_EPOCH);
                int endDays = (int)endSeconds % (24*3600);
                double endDaySeconds = endSeconds - endDays*24*3600;
                clock.writeInterval(new JulianDate(startDays, startDaySeconds), new JulianDate(endDays, endDaySeconds));
                clock.writeCurrentTime(new JulianDate(startDays, startDaySeconds));
                clock.writeMultiplier(1);
                clock.writeRange(ClockRange.LOOP_STOP);
                clock.writeStep(ClockStep.SYSTEM_CLOCK_MULTIPLIER);
            }
        }

        // Load the locations from the JSON and create a coverage definition for them
        JsonArray locations = missionJson.getAsJsonArray("locations");
        ArrayList<CoveragePoint> targetLocations = new ArrayList<>();
        for (JsonElement location: locations) {
            JsonObject locObject = location.getAsJsonObject();
            double latitude = locObject.get("latitude").getAsDouble();
            double longitude = locObject.get("longitude").getAsDouble();
            String locName = locObject.get("name").getAsString();
            GeodeticPoint pt = new GeodeticPoint(FastMath.toRadians(latitude), FastMath.toRadians(longitude), 0.);
            CoveragePoint cp = new CoveragePoint(earthShape, pt, locName);
            targetLocations.add(cp);

            // Add Ground Station to Cesium file
            try (PacketCesiumWriter packet = stream.openPacket(output)) {
                packet.writeId(locName);

                // Save to Cesium
                try (PositionCesiumWriter position = packet.openPositionProperty()) {
                    position.writeCartographicDegrees(new Cartographic(longitude, latitude, 0.));
                }
                try (PointCesiumWriter point = packet.openPointProperty()) {
                    point.writePixelSizeProperty(15.);
                    point.writeColorProperty(255,0,0,255);
                }
                try (LabelCesiumWriter label = packet.openLabelProperty()) {
                    label.writeTextProperty(locName);
                    label.writePixelOffsetProperty(12, 0);
                    try (FontCesiumWriter font = label.openFontProperty()) {
                        font.writeFont("11pt Lucida Console");
                    }
                    label.writeHorizontalOriginProperty(CesiumHorizontalOrigin.LEFT);
                    label.writeVerticalOriginProperty(CesiumVerticalOrigin.CENTER);
                }
            }
        }

        CoverageDefinition targetPoints = new CoverageDefinition("target_points", targetLocations);

        // set the type of propagation
        PropagatorFactory pf = new PropagatorFactory(PropagatorType.TLE, new Properties());

        // Iterate over satellites, build scenarios, run scenarios in parallel
        for (JsonElement satellite: satellitesJson) {
            JsonObject satObject = satellite.getAsJsonObject();
            String tleLine1 = satObject.get("line1").getAsString();
            String tleLine2 = satObject.get("line2").getAsString();

            TLE satTLE = new TLE(tleLine1, tleLine2, utc);
            String satName = satObject.get("name").getAsString();

            // define instruments based on JSON
            ArrayList<Instrument> payload = new ArrayList<>();
            for (JsonElement instrument: satObject.get("sensors").getAsJsonArray()) {
                JsonObject instrObject = instrument.getAsJsonObject();
                String fovType = instrObject.get("geometry_type").getAsString();
                FieldOfView fovDef = null;
                if (fovType.equals("rectangular")) {
                    double acrossTrackFov = instrObject.get("across_fov").getAsDouble();
                    double alongTrackFov = instrObject.get("along_fov").getAsDouble();
                    fovDef = new DoubleDihedraFieldOfView(Vector3D.PLUS_K, Vector3D.PLUS_I, FastMath.toRadians(acrossTrackFov), Vector3D.PLUS_J, FastMath.toRadians(alongTrackFov), 0.);
                }
                else if (fovType.equals("conical")) {
                    double conicalFov = instrObject.get("conical_fov").getAsDouble();
                    fovDef = new CircularFieldOfView(Vector3D.PLUS_K, FastMath.toRadians(conicalFov), 0.);
                }
                else {
                    throw new IllegalArgumentException("Unexpected FOV type!");
                }
                String instrName = instrObject.get("name").getAsString();
                Instrument view = new Instrument(instrName, fovDef, 100, 100);
                payload.add(view);
            }

            ArrayList<Satellite> satellites = new ArrayList<>();
            AttitudeProvider nadirAttitude = new YawCompensation(inertialFrame, new NadirPointing(inertialFrame, earthShape));
            Satellite sat = new Satellite(satName, satTLE, nadirAttitude, payload);
            satellites.add(sat);
            Constellation constellation = new Constellation(satName, satellites);
            targetPoints.assignConstellation(constellation);
            HashSet<CoverageDefinition> covDefs = new HashSet<>();
            covDefs.add(targetPoints);

            // can set the properties of the analyses
            Properties propertiesEventAnalysis = new Properties();
            propertiesEventAnalysis.setProperty("fov.saveAccess", "true");

            // set the coverage event analyses
            EventAnalysisFactory eaf = new EventAnalysisFactory(startDate, endDate, inertialFrame, pf);
            ArrayList<EventAnalysis> eventanalyses = new ArrayList<>();
            FieldOfViewEventAnalysis fovEventAnalysis = (FieldOfViewEventAnalysis) eaf.createGroundPointAnalysis(EventAnalysisEnum.FOV, covDefs, propertiesEventAnalysis);
            eventanalyses.add(fovEventAnalysis);

            // build the scenario
            long start = System.nanoTime();

            Scenario scen = new Scenario.Builder(startDate, endDate, utc).
                    eventAnalysis(eventanalyses).covDefs(covDefs).
                    name(satName).properties(propertiesEventAnalysis).
                    propagatorFactory(pf).build();
            try {
                System.out.println(String.format("Running Scenario %s", scen));
                System.out.println(String.format("Number of points:     %d", targetPoints.getNumberOfPoints()));
                System.out.println(String.format("Number of satellites: %d", constellation.getSatellites().size()));
                // run the scenario
                scen.call();
            } catch (Exception ex) {
                Logger.getLogger(CoverageExample.class.getName()).log(Level.SEVERE, null, ex);
                throw new IllegalStateException("scenario failed to complete.");
            }

            HashMap<Instrument, HashMap<TopocentricFrame, TimeIntervalArray>> accesses =
                    fovEventAnalysis.getAllInstrumentAccesses().get(targetPoints).get(sat);

            // Obtain satellite state at steps
            Propagator prop = pf.createPropagator(satTLE, nadirAttitude, 1000.);
            SaveStateStepHandler stepHandler = new SaveStateStepHandler();
            prop.setMasterMode(stepHandler);
            prop.propagate(startDate, endDate);

            // Save everything to CZML
            try (PacketCesiumWriter packet = stream.openPacket(output)) {
                packet.writeId(sat.getName());

                ArrayList<JulianDate> julianDates = new ArrayList<>();
                for (AbsoluteDate date: stepHandler.dates) {
                    double seconds = date.durationFrom(AbsoluteDate.JULIAN_EPOCH);
                    int days = (int)seconds % (24*3600);
                    double day_seconds = seconds - days*24*3600;
                    julianDates.add(new JulianDate(days, day_seconds));
                }
                // Satellite position
                try (PositionCesiumWriter position = packet.openPositionProperty()) {
                    position.writeInterpolationAlgorithm(CesiumInterpolationAlgorithm.LAGRANGE);
                    position.writeInterpolationDegree(5);
                    position.writeReferenceFrame("INERTIAL");

                    ArrayList<Cartesian> cartesians = new ArrayList<>();
                    for (Vector3D sat_position: stepHandler.positions) {
                        cartesians.add(new Cartesian(sat_position.getX(), sat_position.getY(), sat_position.getZ()));
                    }
                    position.writeCartesian(julianDates, cartesians);
                }
                // Satellite path
                try (PathCesiumWriter path = packet.openPathProperty()) {
                    path.writeLeadTimeProperty(3000.);
                    path.writeTrailTimeProperty(3000.);
                    try (PolylineMaterialCesiumWriter material = path.openMaterialProperty()) {
                        try (SolidColorMaterialCesiumWriter solidColor = material.openSolidColorProperty()){
                            try (ColorCesiumWriter color = solidColor.openColorProperty()) {
                                color.writeRgba(255, 255, 255, 255);
                            }
                        }
                    }
                    path.writeResolutionProperty(300);
                }
                // Satellite orientation
                try (OrientationCesiumWriter orientation = packet.openOrientationProperty()) {
                    orientation.writeInterpolationAlgorithm(CesiumInterpolationAlgorithm.LINEAR);
                    orientation.writeInterpolationDegree(1);

                    ArrayList<UnitQuaternion> quaternions = new ArrayList<>();
                    for (Attitude satOrientation: stepHandler.attitudes) {
                        Rotation rotation = satOrientation.getRotation();
                        Rotation earthRotation = earthFrame.getTransformTo(satOrientation.getReferenceFrame(), satOrientation.getDate()).getRotation();
                        Rotation finalRotation = rotation.applyTo(earthRotation);
                        quaternions.add(new UnitQuaternion(finalRotation.getQ0(), finalRotation.getQ1(), finalRotation.getQ2(), finalRotation.getQ3()));
                    }
                    orientation.writeUnitQuaternion(julianDates, quaternions);
                }
                // Satellite point graphic
                try (PointCesiumWriter point = packet.openPointProperty()) {
                    point.writePixelSizeProperty(15.);
                }
                // Satellite name
                try (LabelCesiumWriter label = packet.openLabelProperty()) {
                    label.writeTextProperty(sat.getName());
                    label.writePixelOffsetProperty(12, 0);
                    try (FontCesiumWriter font = label.openFontProperty()) {
                        font.writeFont("11pt Lucida Console");
                    }
                    label.writeHorizontalOriginProperty(CesiumHorizontalOrigin.LEFT);
                    label.writeVerticalOriginProperty(CesiumVerticalOrigin.CENTER);
                }
            }
            // Sensors FOV
            accesses.forEach(((instrument, instrAccesses) -> {
                // Merge all instrument accesses into single array
                TimeIntervalMerger merger = new TimeIntervalMerger(instrAccesses.values());
                TimeIntervalArray mergedAccesses = merger.orCombine();

                ArrayList<Color> colors = new ArrayList<>();
                ArrayList<JulianDate> julianDates = new ArrayList<>();
                boolean isInside = true;
                double startSeconds = startDate.durationFrom(AbsoluteDate.JULIAN_EPOCH);
                int startDays = (int)startSeconds % (24*3600);
                double startDayseconds = startSeconds - startDays*24*3600;
                julianDates.add(new JulianDate(startDays, startDayseconds));
                if (!mergedAccesses.isEmpty()) {
                    if (mergedAccesses.getRiseSetTimes().get(0).isRise()) {
                        colors.add(new Color(255, 255, 255, 255));
                        isInside = false;
                    }
                    else {
                        colors.add(new Color(255, 0, 0, 255));
                        isInside = true;
                    }
                }
                else {
                    colors.add(new Color(255, 255, 255, 255));
                }
                for (RiseSetTime time: mergedAccesses) {
                    double seconds = mergedAccesses.getHead().durationFrom(AbsoluteDate.JULIAN_EPOCH) + time.getTime();
                    int days = (int)seconds % (24*3600);
                    double day_seconds = seconds - days*24*3600;
                    julianDates.add(new JulianDate(days, day_seconds));
                    if (isInside) {
                        colors.add(new Color(255, 255, 255, 255));
                        isInside = false;
                    }
                    else {
                        colors.add(new Color(255, 0, 0, 255));
                        isInside = true;
                    }
                }

                try (PacketCesiumWriter packet = stream.openPacket(output)) {
                    packet.writeId(instrument.getName());
                    packet.writeParent(sat.getName());
                    try (PositionCesiumWriter position = packet.openPositionProperty()) {
                        position.writeReference(new Reference(sat.getName(), "position"));
                    }
                    try (OrientationCesiumWriter orientation = packet.openOrientationProperty()) {
                        orientation.writeReference(new Reference(sat.getName(), "orientation"));
                    }

                    if (instrument.getFOV() instanceof DoubleDihedraFieldOfView) {
                        try (RectangularSensorCesiumWriter rectangularSensor = packet.openRectangularSensorProperty()) {
                            try (DoubleCesiumWriter xHalfAngle = rectangularSensor.openXHalfAngleProperty()) {
                                xHalfAngle.writeNumber(FastMath.toRadians(15.));
                            }
                            try (DoubleCesiumWriter yHalfAngle = rectangularSensor.openYHalfAngleProperty()) {
                                yHalfAngle.writeNumber(FastMath.toRadians(15.));
                            }
                            try (ColorCesiumWriter intersectionColor = rectangularSensor.openIntersectionColorProperty()) {
                                intersectionColor.writeRgba(julianDates, colors);
                            }
                        }
                    }
                    else if (instrument.getFOV() instanceof CircularFieldOfView) {
                        try (ConicSensorCesiumWriter conicSensor = packet.openConicSensorProperty()) {
                            try (DoubleCesiumWriter outerHalfAngle = conicSensor.openOuterHalfAngleProperty()) {
                                outerHalfAngle.writeNumber(((CircularFieldOfView) instrument.getFOV()).getHalfAperture());
                            }
                            try (ColorCesiumWriter intersectionColor = conicSensor.openIntersectionColorProperty()) {
                                intersectionColor.writeRgba(julianDates, colors);
                            }
                        }
                    }

                }
            }));
        }

        OrekitConfig.end();

        // Save to CZML
        try {
            stringWriter.close();
        }
        catch (IOException e) {
            e.printStackTrace();
        }

        String czml = "[" + stringWriter.toString() + "]";

        Path outputPath = Paths.get(System.getProperty("user.dir"),"int_files", "demo.czml");

        try {
            Files.writeString(outputPath, czml);
        } catch (IOException e) {
            e.printStackTrace();
        }

    }
}

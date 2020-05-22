import com.google.gson.*;
import org.hipparchus.geometry.euclidean.threed.Vector3D;
import org.hipparchus.stat.descriptive.DescriptiveStatistics;
import org.hipparchus.util.FastMath;
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
import org.orekit.propagation.analytical.tle.TLE;
import org.orekit.time.AbsoluteDate;
import org.orekit.time.TimeScale;
import org.orekit.time.TimeScalesFactory;
import org.orekit.utils.Constants;
import org.orekit.utils.IERSConventions;
import seakers.orekit.coverage.access.TimeIntervalArray;
import seakers.orekit.coverage.analysis.AnalysisMetric;
import seakers.orekit.coverage.analysis.GroundEventAnalyzer;
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

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.*;
import java.util.logging.ConsoleHandler;
import java.util.logging.Level;
import java.util.logging.Logger;

public class Propagator {
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

    private final static Logger LOGGER = Logger.getLogger(Logger.GLOBAL_LOGGER_NAME);

    public static void main(String[] args) {
        configureOrekit();

        //setup logger
        Level level = Level.INFO;
        LOGGER.setLevel(level);
        ConsoleHandler handler = new ConsoleHandler();
        handler.setLevel(level);
        LOGGER.addHandler(handler);

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

        // Load the locations from the JSON and create a coverage definition for them
        JsonArray locations = missionJson.getAsJsonArray("locations");
        ArrayList<CoveragePoint> targetLocations = new ArrayList<>();
        for (JsonElement location: locations) {
            JsonObject locObject = location.getAsJsonObject();
            double latitude = locObject.get("latitude").getAsDouble();
            double longitude = locObject.get("longitude").getAsDouble();
            GeodeticPoint pt = new GeodeticPoint(FastMath.toRadians(latitude), FastMath.toRadians(longitude), 0.);
            CoveragePoint cp = new CoveragePoint(earthShape, pt, locObject.get("name").getAsString());
            targetLocations.add(cp);
        }
        CoverageDefinition targetPoints = new CoverageDefinition("target_points", targetLocations);

        // Iterate over satellites, build scenarios, run scenarios in parallel
        AccessesOutput mainOutput = new AccessesOutput();
        for (JsonElement satellite: satellitesJson) {
            JsonObject satObject = satellite.getAsJsonObject();
            String tleLine1 = satObject.get("line1").getAsString();
            String tleLine2 = satObject.get("line2").getAsString();

            TLE satTLE = new TLE(tleLine1, tleLine2, utc);
            String satName = satObject.get("name").getAsString();
            mainOutput.output.put(satName, new HashMap<>());

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
                mainOutput.output.get(satName).put(instrName, new HashMap<>());
            }

            ArrayList<Satellite> satellites = new ArrayList<>();
            AttitudeProvider nadirAttitude = new YawCompensation(inertialFrame, new NadirPointing(inertialFrame, earthShape));
            Satellite sat = new Satellite(satName, satTLE, nadirAttitude, payload);
            satellites.add(sat);
            Constellation constellation = new Constellation(satName, satellites);
            targetPoints.assignConstellation(constellation);
            HashSet<CoverageDefinition> covDefs = new HashSet<>();
            covDefs.add(targetPoints);

            // set the type of propagation
            PropagatorFactory pf = new PropagatorFactory(PropagatorType.TLE, new Properties());

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

            // Extract the coverage and access metrics
            GroundEventAnalyzer ea = new GroundEventAnalyzer(fovEventAnalysis.getEvents(targetPoints));
            DescriptiveStatistics accessStats = ea.getStatistics(AnalysisMetric.DURATION, true, new Properties());
            DescriptiveStatistics gapStats = ea.getStatistics(AnalysisMetric.DURATION, false, new Properties());

            System.out.println(String.format("Max access time %s", accessStats.getMax()));
            System.out.println(String.format("Mean access time %s", accessStats.getMean()));
            System.out.println(String.format("Min access time %s", accessStats.getMin()));
            System.out.println(String.format("50th access time %s", accessStats.getPercentile(50)));
            System.out.println(String.format("80th acceses time %s", accessStats.getPercentile(80)));
            System.out.println(String.format("90th access time %s", accessStats.getPercentile(90)));

            System.out.println(String.format("Max gap time %s", gapStats.getMax()));
            System.out.println(String.format("Mean gap time %s", gapStats.getMean()));
            System.out.println(String.format("Min gap time %s", gapStats.getMin()));
            System.out.println(String.format("50th gap time %s", gapStats.getPercentile(50)));
            System.out.println(String.format("80th gap time %s", gapStats.getPercentile(80)));
            System.out.println(String.format("90th gap time %s", gapStats.getPercentile(90)));

            // Save accesses to JSON file
            HashMap<Instrument, HashMap<TopocentricFrame, TimeIntervalArray>> accesses =
                    fovEventAnalysis.getAllInstrumentAccesses().get(targetPoints).get(sat);
            accesses.forEach((instrument, instrAccesses) -> {
                instrAccesses.forEach((point, localAccesses) -> {
                    String pointName = point.getName();
                    mainOutput.output.get(sat.getName()).get(instrument.getName()).put(pointName, localAccesses);
                });
            });

            long end = System.nanoTime();
            LOGGER.finest(String.format("Took %.4f sec", (end - start) / Math.pow(10, 9)));
        }
        Gson gson = new Gson();
        String jsonOutput = gson.toJson(mainOutput);

        Path outputPath = Paths.get(System.getProperty("user.dir"),"int_files", "accesses.json");
        try (BufferedWriter writer = Files.newBufferedWriter(outputPath)) {
            writer.write(jsonOutput);
        } catch (IOException x) {
            System.err.format("IOException: %s%n", x);
        }

        OrekitConfig.end();
    }
}

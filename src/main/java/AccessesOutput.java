import seakers.orekit.coverage.access.TimeIntervalArray;

import java.util.HashMap;

public class AccessesOutput {
    // satellite, instrument, targetLocation, accesses
    public HashMap<String, HashMap<String, HashMap<String, TimeIntervalArray>>> output;

    public AccessesOutput() {
        output = new HashMap<>();
    }
}

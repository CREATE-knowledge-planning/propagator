package handler;

import org.hipparchus.geometry.euclidean.threed.Vector3D;
import org.orekit.attitudes.Attitude;
import org.orekit.propagation.sampling.OrekitStepHandler;
import org.orekit.propagation.sampling.OrekitStepInterpolator;
import org.orekit.time.AbsoluteDate;
import org.orekit.utils.TimeStampedPVCoordinates;

import java.util.ArrayList;

public class SaveStateStepHandler implements OrekitStepHandler {
    public ArrayList<Vector3D> positions;
    public ArrayList<AbsoluteDate> dates;
    public ArrayList<Attitude> attitudes;

    public SaveStateStepHandler() {
        positions = new ArrayList<>();
        dates = new ArrayList<>();
        attitudes = new ArrayList<>();

    }

    @Override
    public void handleStep(OrekitStepInterpolator interpolator, boolean isLast) {
        TimeStampedPVCoordinates coords = interpolator.getCurrentState().getPVCoordinates();
        positions.add(coords.getPosition());
        dates.add(coords.getDate());
        attitudes.add(interpolator.getCurrentState().getAttitude());
    }
}

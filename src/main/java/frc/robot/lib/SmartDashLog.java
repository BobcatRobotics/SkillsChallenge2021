package frc.robot.lib;

import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.List;

import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;

public class SmartDashLog implements Sendable {

	private static final int MAX_LINES = 10;
	private static List<String> lines = new ArrayList<String>();
	private int currentLine = 0;

	public SmartDashLog() {
		initializeLog();
	}

	private void initializeLog() {
		lines.add("Roborio Real Time Log");
		lines.add("===========================================================");
		lines.add(new SimpleDateFormat("MM/dd/yy h:mm a").format(new Date()));
		lines.add("");
		currentLine = 0;
	}

	public void log(String line) {
		lines.add(line);
	}

	@Override
	public void initSendable(SendableBuilder builder) {
		builder.setSmartDashboardType("Log Display");

		// Add most recent MAX_LINES to display
		currentLine = (lines.size() < MAX_LINES ?  0 : lines.size() - MAX_LINES);
		int linesToDisplay = (lines.size() < MAX_LINES ? lines.size() : currentLine + MAX_LINES);
		
		List<String> lineView = lines.subList(currentLine, linesToDisplay);
		builder.addStringArrayProperty
		  ("LOG", 
		    () -> { return lineView.toArray(new String[0]);}, 
			null
		  );
	}
}

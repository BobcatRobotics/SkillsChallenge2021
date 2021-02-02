package frc.robot.lib;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.List;

import edu.wpi.first.wpilibj.Timer;

public class RioLoggerThread {
	public static RioLoggerThread instance = null;
	private static Thread loggerThread;
	private static String path =  "";
	private static String logName = "thread.txt";
	private static final String dateFmt = "yyyy-MM-dd_hh.mm.ss'.thread.txt'";

	private static List<String> logs = new ArrayList<String>();
	private static long totalLogTime = 3600L; // Default is 1 hour (seconds)
	private static long logFrequency = 150L;  // Default is 2 minutes 30 seconds (seconds)
	private static double endTime = 0.0;
	private static boolean isLogging = false;

	
	static {
		path =  File.separator + "home" + File.separator + "lvuser" + File.separator + "logs" + File.separator;
		logName = path + new SimpleDateFormat(dateFmt).format(new Date());
		instance = new RioLoggerThread();
		createLogDirectory();
		loggerThread = new Thread(new LogThread());
		//logThread.setPriority((Thread.NORM_PRIORITY + Thread.MAX_PRIORITY) / 2);
		loggerThread.start();
		RioLogger.log("RioLoggerThread static {} finished");
	}
	
	private static class LogThread implements Runnable {
		@Override
		public void run() {
			isLogging = true;
			endTime = Timer.getFPGATimestamp() + totalLogTime;
			RioLogger.log(String.format("RioLoggerThread.LogThread run() configuration current time, end time %18f, %18.0f ",Timer.getFPGATimestamp(), endTime));
			//endTime = System.currentTimeMillis() + totalLogTime;
			//RioLogger.log(String.format("RioLoggerThread.LogThread run() configuration current time, end time %18d, %18.0f ",System.currentTimeMillis(), endTime));
			startLogging();
			RioLogger.log(String.format("RioLoggerThread.LogThread run() ending current time, end time %18d, %18.0f ",Timer.getFPGATimestamp(), endTime));
			//RioLogger.log(String.format("RioLoggerThread.LogThread run() ending current time, end time %18d, %18.0f ",System.currentTimeMillis(), endTime));
		}
	}

	
	/**
	 * Returns the one and only one instance of RioLoggerThread
	 */
	public static RioLoggerThread getInstance() {
		return instance;
	}

	private RioLoggerThread() {
	}

	// Set Logging Time (in seconds)
	public static void setLoggingParameters(long totLogTime, long totFreq) {
		RioLogger.log("RioLoggerThread.LogThread setLoggingParameters() totLogTime, totFreq " +totLogTime + ", " + totFreq );
		stopLogging();
		logName = path + new SimpleDateFormat(dateFmt).format(new Date());

		totalLogTime = totLogTime;
		logFrequency = totFreq;
		
		loggerThread = new Thread(new LogThread());
		//logThread.setPriority((Thread.NORM_PRIORITY + Thread.MAX_PRIORITY) / 2);
		loggerThread.start();
	}

	private static void startLogging() {
		double cTime = 0.0;
		isLogging = true;
		do {
			try {
					Thread.sleep(logFrequency * 1000L); // sleep() in milliseconds
			} catch (InterruptedException e) {
				/* The thread can be interrupted by a request to write the logs */
				RioLogger.log("RioLoggerThread startLogging() interrupted. Processing logs.");
			}
			if (logs.size() > 0) {
				List<String> tempLog = new ArrayList<String>(logs);
				logs.clear();
				writeLog(tempLog);
			} 
			cTime = Timer.getFPGATimestamp();
			//cTime = System.currentTimeMillis();
		} while (isLogging && (cTime < endTime));
		//cTime = System.currentTimeMillis();
		cTime = Timer.getFPGATimestamp();
		writeLog(logs);
		logs.clear();
		isLogging = false;
	}
	
	public static void log(String line) {
		logs.add(line);
	}

	public static void stopLogging() {
		isLogging = false;
	}

	public static boolean isLogging() {
		return isLogging;
	}

	/* This method will interrupt the current thread */
	/* write the log and then resume                 */
	public static void writeLog() {
		loggerThread.interrupt();
	}
	
	private static void createLogDirectory() {
		File newDir = new File(path);
		if (!newDir.exists()) {
			try {
				newDir.mkdir();
			} catch (SecurityException e) {
				RioLogger.errorLog("RioLoggerThread Security exception " + e);
			}
		}
	}

	private static void writeLog(List<String> log) {
		BufferedWriter outputStream;
		try {
			// Open the file
			outputStream = new BufferedWriter(new FileWriter(logName, true));
			for (String line : log) {
				outputStream.write(line);
				outputStream.newLine();
			}
			// Close the file
			outputStream.close();
		} catch (IOException e) {
			RioLogger.errorLog("RioLoggerThread Security exception " + e);
		}
	}
}

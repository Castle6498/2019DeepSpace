
import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.io.PrintWriter;
import java.io.UnsupportedEncodingException;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Scanner;

import de.siegmar.fastcsv.reader.CsvParser;
import de.siegmar.fastcsv.reader.CsvReader;
import de.siegmar.fastcsv.reader.CsvRow;

public class Main {

	public static void main(String[] args) {
		
		 
		 String st, inputLocation = null,outputLocation = null;
		
	
		try(BufferedReader br = new BufferedReader(new FileReader("Settings.txt"))) {
		    
		    int r = 0;
			  while ((st = br.readLine()) != null) {
				  if(r==1) inputLocation =st;
				  else if(r==3) outputLocation=st;
			  	r++;
			  } 
		
		} catch (FileNotFoundException e1) {
			// TODO Auto-generated catch block
			e1.printStackTrace();
		} catch (IOException e1) {
			// TODO Auto-generated catch block
			e1.printStackTrace();
		}
	
	
		System.out.println("Input: "+inputLocation);
		System.out.println("Output: "+outputLocation);
		
		
		Scanner reader = new Scanner(System.in);  // Reading from System.in
		System.out.println("Enter the path name: ");
		String name = reader.next(); // Scans the next token of the input as an int.
		//once finished
		reader.close();
		
		System.out.println("Name set to:" + name);
		
		ArrayList<Double> posLeft = new ArrayList<Double>();
		ArrayList<Double> velLeft = new ArrayList<Double>();
		ArrayList<Double> timeLeft = new ArrayList<Double>();
		
		ArrayList<Double> posRight = new ArrayList<Double>();
		ArrayList<Double> velRight = new ArrayList<Double>();
		ArrayList<Double> timeRight = new ArrayList<Double>();
		
		double conversion = 4096/(3.14159*6);
		
		File leftFile = new File(inputLocation+name+".right.pf1.csv");
		File rightFile = new File(inputLocation+name+".left.pf1.csv");
		
		CsvReader csvReaderLeft = new CsvReader();
		CsvReader csvReaderRight = new CsvReader();
		
		try (CsvParser csvParser = csvReaderLeft.parse(leftFile, StandardCharsets.UTF_8)) {
		    CsvRow row;
		    while ((row = csvParser.nextRow()) != null) {
		    	if(row.getOriginalLineNumber()!=1) {
		      //  System.out.println("Read line: " + row);
	    		posLeft.add(Double.parseDouble(row.getField(3))*conversion);
	    		velLeft.add(Double.parseDouble(row.getField(4))*conversion);
	    		timeLeft.add(Double.parseDouble(row.getField(0))*1000);
		        //System.out.println("First column of line: " + row.getField(0));
		    	}
		    }
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
		try (CsvParser csvParser = csvReaderRight.parse(rightFile, StandardCharsets.UTF_8)) {
		    CsvRow row;
		    while ((row = csvParser.nextRow()) != null) {
		    	if(row.getOriginalLineNumber()!=1) {
		      //  System.out.println("Read line: " + row);
	    		posRight.add(Double.parseDouble(row.getField(3))*conversion);
	    		velRight.add(Double.parseDouble(row.getField(4))*conversion);
	    		timeRight.add(Double.parseDouble(row.getField(0))*1000);
		        //System.out.println("First column of line: " + row.getField(0));
		    	}
		    }
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
		System.out.println("Read "+velLeft.size()+" lines.");
		System.out.println("Converting to output format.");
		
		ArrayList<String> outputLeft = new ArrayList<String>(Arrays.asList(
				"package frc.robot.motion_profile;",
				"public class GeneratedMotionProfileLeft{" 
				));
		
		outputLeft.add("public static final int kNumPoints = "+velLeft.size()+";");
		
		outputLeft.add("public static double [][]Points = new double[][]{");
		
		for(int i=0;i<velLeft.size();i++) {
			outputLeft.add("{"+posLeft.get(i)+" , "+velLeft.get(i)+" , "+timeLeft.get(i)+"},");
		}
		
		outputLeft.add("};}");
		
		ArrayList<String> outputRight = new ArrayList<String>(Arrays.asList(
				"package frc.robot.motion_profile;",
				"public class GeneratedMotionProfileRight{" 
				));
		
		outputRight.add("public static final int kNumPoints = "+velRight.size()+";");
		
		outputRight.add("public static double [][]Points = new double[][]{");
		
		for(int i=0;i<velRight.size();i++) {
			outputRight.add("{"+posRight.get(i)+" , "+velRight.get(i)+" , "+timeRight.get(i)+"},");
		}
		
		outputRight.add("};}");
		
		System.out.println("Writing java files.");
		
		PrintWriter writerLeft;
		PrintWriter writerRight;
		try {
			writerLeft = new PrintWriter(outputLocation+"GeneratedMotionProfileLeft.java", "UTF-8");
			
			for(String j:outputLeft) {
				writerLeft.println(j);
			}
			writerLeft.close();
			
			writerRight = new PrintWriter(outputLocation+"GeneratedMotionProfileRight.java", "UTF-8");
			
			for(String j:outputRight) {
				writerRight.println(j);
			}
			writerRight.close();
			
		} catch (FileNotFoundException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (UnsupportedEncodingException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
		System.out.println("Done!");

}
}

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot.subsystems;

import java.io.File;
import java.sql.Date;
import java.text.MessageFormat;
import java.util.stream.Stream;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc4388.utility.CSV;

/**
 * An example subsystem that uses CSV data.
 */
// Single line comments are to explain code and should be removed.
public class ExampleCSVSubsystem extends SubsystemBase {

  // The class to store CSV data in. Fields correspond to columns in the CSV file.
  // Modifiers and accessors don't matter for the CSV reader, so use what works
  // best with the rest of the implementation.
  public static class ExampleCSVEntry {
    public Double distance;
    public Integer duration;
    public Date date;
    public Boolean wind;

    // Convenience getters for use with method references in lerp functions.
    public Double getDistance() {
      return distance;
    }

    public Integer getDuration() {
      return duration;
    }
  }

  public final CSV<ExampleCSVEntry> m_distanceCSV;

  /**
   * Creates a new ExampleCSVSubsystem.
   */
  public ExampleCSVSubsystem() {
    // Initialize a File pointing to the CSV file in the deploy directory.
    var csvFile = new File(Filesystem.getDeployDirectory(), "example_data.csv");
    // Initialize the CSV reader using the path to the file.
    m_distanceCSV = new CSV<>(csvFile.toPath(), ExampleCSVEntry.class);
    // Sets units and parentheses from the CSV header to be removed by passing a
    // lambda expression that removes parentheses and their contents from the passed
    // string with a regular expression.
    m_distanceCSV.setHeaderPreprocessor(s -> s.replaceAll("\\([^\\)]*+\\)", ""));
    // Reads and stores the CSV data (getData() is lazy, first run will read and
    // return the data, consequent runs will return data without reading). Run this
    // once in the constructor to read the CSV file.
    m_distanceCSV.getData();
    // Print the CSV as a table.
    printDataTable(10);
  }

  // Getters for data based on distance and duration, with linear interpolation.
  public Integer getDuration(Double distance) {
    return m_distanceCSV.linearInterpolate(distance, ExampleCSVEntry::getDistance, ExampleCSVEntry::getDuration)
        .intValue();
  }

  public Double getDistance(Integer duration) {
    return m_distanceCSV.linearInterpolate(duration, ExampleCSVEntry::getDuration, ExampleCSVEntry::getDistance)
        .doubleValue();
  }

  public Date getDate(Double distance) {
    return Stream.of(m_distanceCSV.getData()).filter(entry -> entry.distance.equals(distance)).findFirst()
        .orElseThrow().date;
  }

  public Boolean getWind(Double distance) {
    return Stream.of(m_distanceCSV.getData()).filter(entry -> entry.distance.equals(distance)).findFirst()
        .orElseThrow().wind;
  }

  /**
   * Prints the CSV data as a table.
   * 
   * @param columnWidth the minimum number of characters in each column, used for
   *                    padding.
   */
  public void printDataTable(int columnWidth) {
    Stream.of(m_distanceCSV.getData()).forEachOrdered(
        row -> System.out.println(String.format(MessageFormat.format("| %{0}s | %{0}s | %{0}s | %{0}s |", columnWidth),
            row.distance, row.duration, row.date, row.wind)));
  }

}
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot.subsystems;

import java.io.File;
import java.io.IOException;
import java.sql.Date;
import java.util.Comparator;
import java.util.Map;
import java.util.Optional;
import java.util.function.Function;
import java.util.regex.Pattern;
import java.util.stream.IntStream;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc4388.utility.CSV;

/**
 * An example subsystem that uses CSV data.
 */
// Single line comments are to explain example code and can be removed.
public class ExampleCSVSubsystem extends SubsystemBase {

  // The class to store a single row of CSV data in. Each field corresponds with
  // a cell in any given row.
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

    public Date getDate() {
      return date;
    }

    public Boolean getWind() {
      return wind;
    }
  }

  public final ExampleCSVEntry[] m_exampleCSVData;

  /**
   * Creates a new ExampleCSVSubsystem.
   */
  public ExampleCSVSubsystem() {
    // Initialize a File pointing to the CSV file in the deploy directory.
    var csvFilePath = new File(Filesystem.getDeployDirectory(), "example_data.csv").toPath();
    // Initialize the CSV reader using the path to the file.
    CSV<ExampleCSVEntry> exampleCSV = new CSV<>(ExampleCSVEntry::new) {
      // A regular expression for parentheses and their contents.
      private final Pattern parentheses = Pattern.compile("\\([^\\)]*+\\)");

      // Removes text matching the parentheses regex from the header, then calls the
      // super header sanitizer which removes remaining invalid identifier characters.
      // For example: "Run Distance (miles) " -> "Run Distance " -> "RunDistance"
      @Override
      protected String headerSanitizer(final String header) {
        return super.headerSanitizer(parentheses.matcher(header).replaceAll(""));
      }
    };
    try {
      // Read and store the CSV data.
      m_exampleCSVData = exampleCSV.read(csvFilePath);
    } catch (IOException e) {
      // Throw a RuntimeException if an IOException occurs while reading.
      throw new RuntimeException(e);
    }
    // Print the CSV data as a colored table.
    System.out.println(CSV.ReflectionTable.create(m_exampleCSVData));
  }

  // Getters for data based on distance and duration, with linear interpolation.
  // For use in other classes.
  public Integer getDuration(Double distance) {
    return linearInterpolate(distance, ExampleCSVEntry::getDistance, ExampleCSVEntry::getDuration).intValue();
  }

  public Double getDistance(Integer duration) {
    return linearInterpolate(duration, ExampleCSVEntry::getDuration, ExampleCSVEntry::getDistance).doubleValue();
  }

  // Getters for data based on distance, or null if an entry with the passed
  // distance isn't found (or the CSV is empty). For use in other classes.
  public Date getDate(Double distance) {
    return lookup(distance, ExampleCSVEntry::getDistance, true).map(Map.Entry::getValue).map(ExampleCSVEntry::getDate)
        .orElse(null);
  }

  public Boolean getWind(Double distance) {
    return lookup(distance, ExampleCSVEntry::getDistance, true).map(Map.Entry::getValue).map(ExampleCSVEntry::getWind)
        .orElse(null);
  }

  /**
   * Retrieves a linearly interpolated value by finding the closest entries in the
   * CSV data using the given {@code value} and {@code valueGetter}. The return
   * value is interpolated between the values retrieved with {@code targetGetter}.
   * 
   * @param lookupValue  the data point to predict a value for
   * @param lookupGetter a getter function for the given lookup value
   * @param targetGetter a getter for the desired value
   * @return a linearly interpolated value
   */
  private Number linearInterpolate(final Number lookupValue, final Function<ExampleCSVEntry, Number> lookupGetter,
      final Function<ExampleCSVEntry, Number> targetGetter) {
    final Map.Entry<Integer, ExampleCSVEntry> closestEntry = lookup(lookupValue.doubleValue(), lookupGetter, false)
        .orElse(Map.entry(m_exampleCSVData.length - 1, m_exampleCSVData[m_exampleCSVData.length - 1]));
    final ExampleCSVEntry closestRecord = closestEntry.getValue();
    final int closestRecordIndex = closestEntry.getKey();
    final ExampleCSVEntry neighborRecord = m_exampleCSVData[lookupValue.doubleValue() <= lookupGetter
        .apply(closestRecord).doubleValue() ? Math.max(closestRecordIndex == 0 ? 1 : 0, closestRecordIndex - 1)
            : Math.min(closestRecordIndex + 1,
                m_exampleCSVData.length - (closestRecordIndex == m_exampleCSVData.length - 1 ? 2 : 1))];
    return lerp2(lookupValue, lookupGetter.apply(closestRecord), targetGetter.apply(closestRecord),
        lookupGetter.apply(neighborRecord), targetGetter.apply(neighborRecord));
  }

  /**
   * Retrieves an entry from the CSV data that is closest to the given value.
   * 
   * @param value       the value to search for
   * @param valueGetter a getter for the value of a record to compare with
   * @param exactMatch  if the result's value must exactly match the given value
   * @return an optional containing an entry of the record's index in the CSV data
   *         and the record itself, or an empty optional if an exact match was
   *         requested but not found (or the CSV is empty).
   */
  private Optional<Map.Entry<Integer, ExampleCSVEntry>> lookup(final Number value,
      final Function<ExampleCSVEntry, Number> valueGetter, boolean exactMatch) {
    Optional<Map.Entry<Integer, ExampleCSVEntry>> match = IntStream.range(0, m_exampleCSVData.length)
        .mapToObj(i -> Map.entry(i, m_exampleCSVData[i])).min(Comparator
            .comparingDouble(e -> Math.abs(valueGetter.apply(e.getValue()).doubleValue() - value.doubleValue())));
    return !exactMatch || match.map(e -> valueGetter.apply(e.getValue()).equals(value)).orElse(false) ? match
        : Optional.empty();

  }

  private Number lerp2(final Number x, final Number x0, final Number y0, final Number x1, final Number y1) {
    final Number f = (x.doubleValue() - x0.doubleValue()) / (x1.doubleValue() - x0.doubleValue());
    return (1.0 - f.doubleValue()) * y0.doubleValue() + f.doubleValue() * y1.doubleValue();
  }
}
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.utility;

import java.io.IOException;
import java.lang.reflect.Array;
import java.lang.reflect.Constructor;
import java.lang.reflect.Field;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.nio.file.Files;
import java.nio.file.Path;
import java.text.MessageFormat;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.function.Function;
import java.util.function.Predicate;
import java.util.function.UnaryOperator;
import java.util.regex.Pattern;
import java.util.stream.IntStream;
import java.util.stream.Stream;

public class CSV<E> {

    private static Predicate<String> quoteMatcher = Pattern.compile("\"*").asMatchPredicate();

    private Path path;
    private Class<E> clazz;
    private UnaryOperator<String> headerPreprocessor = (String value) -> value;
    private E[] data;

    /**
     * Creates a new {@code CSV} instance for a CSV file.
     * 
     * @param path  the path to a CSV file
     * @param clazz a class representing an object with fields for the matching
     *              columns in the CSV file. The first character of the column names
     *              from the header in the CSV file will be made lowecase and
     *              invalid characters will be removed when getting data.
     * @see #getData()
     */
    public CSV(Path path, Class<E> clazz) {
        this.path = path;
        this.clazz = clazz;
    }

    /**
     * Sets a function to be applied to the string value of every column name in the
     * header of this CSV.
     *
     * @param headerPreprocessor a function that returns a modified string
     */
    public void setHeaderPreprocessor(UnaryOperator<String> headerPreprocessor) {
        this.headerPreprocessor = headerPreprocessor;
    }

    /**
     * Shorthand for {@link #getData(Boolean) getData(true)}
     * 
     * @see #getData(Boolean)
     */
    public E[] getData() {
        return getData(false);
    }

    /**
     * Reads and parses the contents of the CSV file, and returns an array of the
     * previously given class. Cells are parsed using the field's
     * {@code valueOf(String)} function.
     * 
     * @param flush if the CSV file should be read from the path again
     * @return the parsed data from the CSV file
     * @throws RuntimeException (Caused by IOException) if an I/O error occurs
     *                          opening the file
     * @throws RuntimeException (Caused by NoSuchMethodException) if a constructor
     *                          with no parameters is not found.
     */
    @SuppressWarnings("unchecked")
    public E[] getData(boolean flush) {
        if (flush || data == null) {
            try {
                String[] fieldNames = Stream
                        .of(unescapedSplit(headerPreprocessor.apply(Files.lines(path).findFirst().orElseThrow())
                                .replaceAll("[^$\\w,]", "")))
                        .map(header -> (Character.toLowerCase(header.charAt(0)) + header.substring(1)).trim())
                        .toArray(String[]::new);
                Field[] fields = Stream.of(fieldNames).map(fieldName -> getField(clazz, fieldName))
                        .toArray(Field[]::new);
                Method[] parsers = Stream.of(fields).map(CSV::getMethod).toArray(Method[]::new);
                Constructor<E> constructor = clazz.getConstructor();
                data = Files.lines(path).skip(1).filter(Predicate.not(String::isBlank)).map(line -> {
                    try {
                        var element = constructor.newInstance();
                        String[] cells = unescapedSplit(line);
                        IntStream.range(0, Math.min(fieldNames.length, cells.length)).forEach(
                                i -> setParsedField(element, fields[i], fieldNames[i], parsers[i], cells[i].trim()));
                        return element;
                    } catch (InstantiationException | IllegalAccessException | InvocationTargetException e) {
                        throw new RuntimeException(e);
                    }
                }).toArray(size -> (E[]) Array.newInstance(clazz, size));
            } catch (IOException | NoSuchMethodException e) {
                throw new RuntimeException(e);
            }
        }
        return data;
    }

    /**
     * Retrieves a linearly interpolated value by finding the closest entries in the
     * CSV data using the given {@code value} and {@code valueGetter}. The return
     * value is interpolated between the values retrieved with {@code targetGetter}.
     * 
     * @param value        the value to linear interpolate to
     * @param valueGetter  a getter for the given value, used to find the closest
     *                     entries in the CSV for linear interpolation
     * @param targetGetter a getter for the desired value, used to get adjacent
     *                     values for linear interpolation
     * @return a linearly interpolated value
     */
    public Number linearInterpolate(Number value, Function<E, Number> valueGetter, Function<E, Number> targetGetter) {
        int i = getClosestRowIndex(value.doubleValue(), valueGetter);
        int j = value.doubleValue() <= valueGetter.apply(data[i]).doubleValue() ? Math.max(i != 0 ? 0 : 1, i - 1)
                : Math.min(i + 1, data.length - (i != data.length - 1 ? 1 : 2));
        return lerp2(value, valueGetter.apply(data[i]), targetGetter.apply(data[i]), valueGetter.apply(data[j]),
                targetGetter.apply(data[j]));
    }

    private int getClosestRowIndex(Number value, Function<E, Number> valueGetter) {
        return IntStream.range(0, data.length).boxed()
                .min((a, b) -> Double.compare(Math.abs(valueGetter.apply(data[a]).doubleValue() - value.doubleValue()),
                        Math.abs(valueGetter.apply(data[b]).doubleValue() - value.doubleValue())))
                .orElse(data.length - 1);
    }

    private Number lerp2(Number x, Number x0, Number y0, Number x1, Number y1) {
        Number f = (x.doubleValue() - x0.doubleValue()) / (x1.doubleValue() - x0.doubleValue());
        return (1.0 - f.doubleValue()) * y0.doubleValue() + f.doubleValue() * y1.doubleValue();
    }

    private static Field getField(Class<?> clazz, String fieldName) {
        try {
            return clazz.getDeclaredField(fieldName);
        } catch (NoSuchFieldException | SecurityException e) {
            System.err.println(MessageFormat.format(
                    "Warning in thread \"{0}\" {1}: {2}.{3}, values from CSV column \"{3}\" will be dropped.",
                    Thread.currentThread().getName(), e.getClass().getName(), clazz.getSimpleName(), fieldName));
            return null;
        }
    }

    private static Method getMethod(Field field) {
        try {
            return field.getType().getMethod("valueOf", String.class);
        } catch (NoSuchMethodException | NullPointerException e) {
            return null;
        } catch (SecurityException e) {
            throw new RuntimeException(e);
        }
    }

    private static String[] unescapedSplit(CharSequence input) {
        var openedQuotes = new AtomicInteger(input.charAt(0) == '"' ? 1 : 0);
        List<String> strings = new ArrayList<>();
        strings.add(input.chars().boxed().map(value -> String.valueOf((char) value.intValue()))
                .reduce((result, element) -> {
                    if (result.equals(",")) {
                        strings.add("");
                        return "";
                    }
                    switch (element) {
                        case "\"":
                            if (result.isBlank() || quoteMatcher.test(result)) {
                                openedQuotes.incrementAndGet();
                            } else {
                                openedQuotes.decrementAndGet();
                            }
                            return result + element;
                        case ",":
                            if (openedQuotes.get() <= 0) {
                                openedQuotes.set(0);
                                strings.add(result);
                                return "";
                            }
                        default:
                            return result + element;
                    }
                }).orElseThrow());
        return strings.toArray(String[]::new);
    }

    private void setParsedField(E element, Field field, String fieldName, Method parser, String value) {
        try {
            if (field == null)
                throw new NoSuchFieldException(clazz.getSimpleName() + "." + fieldName);
            if (parser == null && !field.getType().isInstance(value)) {
                throw new NoSuchMethodException(field.getType().getName() + ".valueOf(String)");
            }
            field.trySetAccessible();
            field.set(element, parser != null ? (value.isBlank() ? null : parser.invoke(element, value)) : value);
        } catch (IllegalAccessException | NoSuchFieldException | NoSuchMethodException e) {
            System.err.println(MessageFormat.format(
                    "Warning in thread \"{0}\" {1}: {2}, value \"{3}\" from CSV column \"{4}\" dropped.",
                    Thread.currentThread().getName(), e.getClass().getName(), e.getMessage(), value, fieldName));
        } catch (InvocationTargetException e) {
            throw new RuntimeException(e);
        }
    }

}

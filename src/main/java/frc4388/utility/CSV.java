/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.utility;

import java.awt.Color;

import java.io.BufferedReader;
import java.io.IOException;
import java.lang.invoke.MethodHandleProxies;
import java.lang.invoke.MethodHandles;
import java.lang.invoke.MethodType;
import java.lang.reflect.Array;
import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.nio.file.Files;
import java.nio.file.Path;
import java.text.MessageFormat;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.function.BiConsumer;
import java.util.function.Function;
import java.util.function.IntFunction;
import java.util.function.Predicate;
import java.util.function.Supplier;
import java.util.regex.Pattern;
import java.util.stream.Collectors;
import java.util.stream.IntStream;
import java.util.stream.Stream;

public class CSV<R> {
    private static final Pattern SANITIZER = Pattern.compile("[^$\\w,]");

    private final Supplier<R> generator;
    private final IntFunction<R[]> arrayGenerator;
    private final Map<String, BiConsumer<R, String>> setters;

    /**
     * A binary string operator to be applied to the entire header of the CSV.
     */
    protected String headerSanitizer(final String header) {
        return SANITIZER.matcher(header).replaceAll("");
    }

    /**
     * A binary string operator to be applied to each name in the header of the CSV.
     */
    protected String nameProcessor(final String name) {
        return Character.toLowerCase(name.charAt(0)) + name.substring(1);
    }

    /**
     * Creates a new {@code CSV} instance and prepares for populating the fields of
     * objects created by the given generator. Fields of primitive types is not
     * supported.
     * 
     * @param generator a parameterless supplier which produces a new object with
     *                  any number of fields corresponding to header names from a
     *                  CSV file. The first character of the names from the header
     *                  in the CSV file will be made lowercase and invalid
     *                  characters will be removed to match Java naming conventions.
     * @see #read(Path)
     */
    @SuppressWarnings("unchecked")
    public CSV(final Supplier<R> generator) {
        final Class<?> clazz = generator.get().getClass();
        final Map<Class<?>, Function<String, ?>> fieldParsers = new HashMap<>();
        this.arrayGenerator = size -> (R[]) Array.newInstance(clazz, size);
        this.generator = generator;
        this.setters = new HashMap<>();
        for (final Field field : clazz.getFields()) {
            final Function<String, ?> parser = Modifier.isStatic(field.getModifiers()) ? null
                    : fieldParsers.computeIfAbsent(field.getType(), CSV::getTypeParser);
            if (parser != null)
                this.setters.put(field.getName(), (final R obj, final String rawValue) -> {
                    try {
                        field.set(obj, rawValue.isEmpty() ? null : parser.apply(rawValue));
                    } catch (final IllegalAccessException e) {
                        throw new RuntimeException(e);
                    }
                });
        }
    }

    /**
     * Reads and parses the contents of the given CSV file, and returns an array
     * filled with populated objects created with the previously given generator.
     * Cells are parsed using their corresponding field's {@code valueOf(String)}
     * function.
     *
     * @param path the path to a CSV file
     * @return the parsed data from the CSV file
     * @throws IOException if an I/O error occurs opening the file
     */
    @SuppressWarnings("unchecked")
    public R[] read(final Path path) throws IOException {
        try (final BufferedReader reader = Files.newBufferedReader(path)) {
            final BiConsumer<R, String>[] fieldSetters = Stream.of(headerSanitizer(reader.readLine()).split(","))
                    .map(this::nameProcessor).map(setters::get).toArray(BiConsumer[]::new);
            final Stream<String> lines = reader.lines();
            return lines.filter(Predicate.not(String::isBlank))
                    .map(line -> deserializeRecordString(line, fieldSetters, generator.get()))
                    .toArray(this.arrayGenerator);
        }
    }

    @SuppressWarnings("unchecked")
    private static Function<String, ?> getTypeParser(final Class<?> type) {
        try {
            return type.isAssignableFrom(String.class) ? Function.identity()
                    : MethodHandleProxies.asInterfaceInstance(Function.class, MethodHandles.publicLookup()
                            .findStatic(type, "valueOf", MethodType.methodType(type, String.class)));
        } catch (final NoSuchMethodException | IllegalAccessException e) {
            return null;
        }
    }

    private static <R> R deserializeRecordString(final String recordString,
            final BiConsumer<R, String>[] fieldParseSetters, final R object) {
        final int recordStringLength = recordString.length();
        int fieldBeginIndex = 0, tryFieldEndFromIndex = 0, i = 0;
        while (tryFieldEndFromIndex < recordStringLength && i < fieldParseSetters.length) {
            final int tryFieldEndIndex = recordString.indexOf(',', tryFieldEndFromIndex);
            String field = recordString
                    .substring(fieldBeginIndex, tryFieldEndIndex == -1 ? recordStringLength : tryFieldEndIndex).strip();
            if (!field.isEmpty() && (tryFieldEndFromIndex != fieldBeginIndex || field.charAt(0) == '"')) {
                final int fieldLength = field.length();
                if (countTrailing(field, '"') % 2 == 0) {
                    tryFieldEndFromIndex = tryFieldEndIndex + 1;
                    continue;
                } else
                    field = field.substring(1, fieldLength - 1).replace("\"\"", "\"");
            }
            final BiConsumer<R, String> setter = fieldParseSetters[i];
            if (setter != null)
                setter.accept(object, field);
            tryFieldEndFromIndex = fieldBeginIndex = tryFieldEndIndex + 1;
            i++;
        }
        return object;
    }

    private static int countTrailing(final String str, final char c) {
        final int l = str.length();
        int count = 0;
        while (str.charAt(l - count - 1) == c && count < l)
            count++;
        return count;
    }

    public static class ReflectionTable {
        public static <T> String create(final T[] objects) {
            final Field[] fields = Stream.of(objects).flatMap(object -> Stream.of(object.getClass().getFields()))
                    .distinct().toArray(Field[]::new);
            final List<List<ReflectionTable>> rows = new ArrayList<>();
            rows.add(Stream.of(fields).map(ReflectionTable::new).collect(Collectors.toList()));
            rows.addAll(Stream.of(objects).map(
                    obj -> Stream.of(fields).map(field -> new ReflectionTable(obj, field)).collect(Collectors.toList()))
                    .collect(Collectors.toList()));
            final int[] columnWidths = rows.stream()
                    .map(row -> row.stream().map(cell -> cell.string).mapToInt(String::length).toArray())
                    .reduce(new int[fields.length], (result, row) -> IntStream.range(0, row.length)
                            .map(i -> Math.max(result[i], row[i])).toArray());
            IntStream.range(0, fields.length).forEach(i -> {
                final var columnSummaryStatistics = rows.stream().skip(1)
                        .mapToDouble(row -> row.get(i).getValue().doubleValue()).summaryStatistics();
                rows.stream().skip(1).forEach(row -> row.get(i).colorByValue(columnSummaryStatistics.getMin(),
                        columnSummaryStatistics.getMax()));
            });
            return rows.stream()
                    .map(row -> IntStream.range(0, row.size())
                            .mapToObj(i -> String.format(
                                    MessageFormat.format("{0} %{1}{2}s {3}", row.get(i).escape,
                                            row.get(i).padRight ? "-" : "", columnWidths[i], RESET_STYLE),
                                    row.get(i).string))
                            .collect(Collectors.joining("|")))
                    .collect(Collectors.joining(LF));
        }

        private static final Color GRADIENT_MIN = new Color(0, 51, 0);
        private static final Color GRADIENT_MAX = new Color(0, 255, 0);
        private static final String CONTROL = "\033";
        private static final String CSI = "[";
        private static final String LF = "\n";
        private static final String RESET = "0";
        private static final String BOLD = "1";
        private static final String ITALIC = "3";
        private static final String UNDERLINE = "4";
        private static final String BACKGROUND_RED = "41";
        private static final String FOREGROUND = "38";
        private static final String BACKGROUND = "48";
        private static final String TRUECOLOR = "2";
        private static final String SEPARATOR = ";";
        private static final String SGR = "m";
        private static final String HEADER_STYLE = CONTROL + CSI + BOLD + SEPARATOR + UNDERLINE + SGR;
        private static final String NULL_STYLE = CONTROL + CSI + ITALIC + SGR;
        private static final String ERROR_STYLE = CONTROL + CSI + ITALIC + SGR + CONTROL + CSI + BACKGROUND_RED + SGR;
        private static final String RESET_STYLE = CONTROL + CSI + RESET + SGR;
        private Object value;
        private String string;
        private boolean padRight;
        private String escape;

        private ReflectionTable(final Object obj, final Field field) {
            try {
                value = field.get(obj);
                string = Objects.toString(value);
                padRight = !Number.class.isAssignableFrom(field.getType());
                escape = Objects.isNull(value) ? NULL_STYLE : "";
            } catch (final IllegalAccessException | IllegalArgumentException e) {
                value = null;
                string = e.getClass().getSimpleName();
                padRight = false;
                escape = ERROR_STYLE;
            }
        }

        private ReflectionTable(final Field field) {
            value = null;
            string = field.getName();
            padRight = true;
            escape = HEADER_STYLE;
        }

        private Number getValue() {
            return padRight ? Objects.hashCode(string) : Objects.requireNonNullElse((Number) value, 0);
        }

        private void colorByValue(final Number min, final Number max) {
            if (Objects.nonNull(value)) {
                final double normal = (getValue().doubleValue() - min.doubleValue())
                        / (max.doubleValue() - min.doubleValue());
                final Color color = new Color(range(normal, GRADIENT_MIN.getRed(), GRADIENT_MAX.getRed()),
                        range(normal, GRADIENT_MIN.getGreen(), GRADIENT_MAX.getGreen()),
                        range(normal, GRADIENT_MIN.getBlue(), GRADIENT_MAX.getBlue()));
                escape += (contrastRatio(color, Color.BLACK) > contrastRatio(Color.WHITE, color)
                        ? colorTo24BitSGR(Color.BLACK, false)
                        : colorTo24BitSGR(Color.WHITE, false)) + colorTo24BitSGR(color, true);
            }
        }

        private static String colorTo24BitSGR(final Color color, final boolean background) {
            return CONTROL + CSI + (background ? BACKGROUND : FOREGROUND) + SEPARATOR + TRUECOLOR + SEPARATOR
                    + color.getRed() + SEPARATOR + color.getGreen() + SEPARATOR + color.getBlue() + SGR;
        }

        private static int range(final double normal, final int min, final int max) {
            return (int) (normal * (max - min) + min);
        }

        /* https://www.w3.org/TR/WCAG20/#contrast-ratiodef */
        private static float contrastRatio(final Color lighter, final Color darker) {
            return (relativeLuminance(lighter) + 0.05f) / (relativeLuminance(darker) + 0.05f);
        }

        /* https://www.w3.org/TR/2008/REC-WCAG20-20081211/#relativeluminancedef */
        private static float relativeLuminance(final Color color) {
            final float[] components = color.getRGBComponents(null);
            final float r = components[0] <= 0.03928f ? components[0] / 12.92f
                    : (float) Math.pow((components[0] + 0.055f) / 1.055f, 2.4f);
            final float g = components[1] <= 0.03928f ? components[1] / 12.92f
                    : (float) Math.pow((components[1] + 0.055f) / 1.055f, 2.4f);
            final float b = components[2] <= 0.03928f ? components[2] / 12.92f
                    : (float) Math.pow((components[2] + 0.055f) / 1.055f, 2.4f);
            return 0.2126f * r + 0.7152f * g + 0.0722f * b;
        }
    }
}

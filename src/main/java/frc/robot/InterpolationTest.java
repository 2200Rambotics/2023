package frc.robot;

public class InterpolationTest {

    public static void main(String[] args) {
        InterpolationTest t = new InterpolationTest();
        t.tableTest();
    }

    public void tableTest() {
        Point[] points = {
                new Point(0, 0),
                new Point(1, 5),
                new Point(2, 8),
                new Point(3, 10)
        };
        Interpolation l = new Interpolation(points);

        double[][] test_cases = {
                { 0.5, 2.5 },
                { 1, 5 },
                { 0, 0 },
                { -1, -5 },
                { 4, 12 },
                { 0.75, 3.75 }
        };

        /*
         * double[][] test_cases2 = {
         * {1, 3.768},
         * {2, 8.321},
         * {3, 13.659},
         * {4, 19.782},
         * {5, 26.69}
         * };
         */

        for (int i = 0; i < test_cases.length; i++) {
            double input = test_cases[i][0];
            double expected = test_cases[i][1];
            double actual = l.interpolate(input);
            if (expected != actual) {
                System.out.printf("ERROR interpolate(%f) should be %f, was %f\n", input, expected, actual);
            } else {
                System.out.println("PASS");
            }
        }
    }
}

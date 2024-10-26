package Random;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.opencv.videoio.VideoCapture;

public class AprilTagDetection {
    
    static { System.loadLibrary(Core.NATIVE_LIBRARY_NAME); }

    public static void main(String[] args) {
        VideoCapture camera = new VideoCapture(0); // 0 is the ID of the default camera
        Mat frame = new Mat();

        if (!camera.isOpened()) {
            System.out.println("Camera not available.");
            return;
        }

        while (true) {
            camera.read(frame);
            if (!frame.empty()) {
                // Convert to grayscale
                Mat gray = new Mat();
                Imgproc.cvtColor(frame, gray, Imgproc.COLOR_BGR2GRAY);

                // Detect AprilTags
                // You would implement your AprilTag detection logic here.
                // This could involve using a library that detects AprilTags
                // and drawing them on the frame.

                // For demo, we just display the frame
                HighGui.imshow("Camera Feed", frame);
                if (HighGui.waitKey(30) >= 0) break; // Exit if any key is pressed
            } else {
                System.out.println("Frame is empty.");
            }
        }

        camera.release();
        System.exit(0);
    }
}

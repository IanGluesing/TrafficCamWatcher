
#include <opencv2/opencv.hpp>
#include <thread>
#include <chrono>
#include <opencv2/optflow/motempl.hpp>

using namespace cv;

std::unordered_map<std::string, cv::Mat> my_map = {
    {"https://iowadotsfs2.us-east-1.skyvdn.com:443/rtplive/ictv39lb/playlist.m3u8", cv::Mat()},
    {"https://iowadotsfs1.us-east-1.skyvdn.com:443/rtplive/ictv03lb/playlist.m3u8", cv::Mat()}
};

void optical_flow(cv::VideoCapture& cap) {
	Mat frame1, prvs;
    cap >> frame1;
    cvtColor(frame1, prvs, COLOR_BGR2GRAY);
 
    while(true){
        Mat frame2, next;
        cap >> frame2;
        if (frame2.empty())
            break;
        cvtColor(frame2, next, COLOR_BGR2GRAY);
 
        Mat flow(prvs.size(), CV_32FC2);
        calcOpticalFlowFarneback(prvs, next, flow, 0.5, 3, 15, 3, 5, 1.2, 0);
 
        // visualization
        Mat flow_parts[2];
        split(flow, flow_parts);
        Mat magnitude, angle, magn_norm;
        cartToPolar(flow_parts[0], flow_parts[1], magnitude, angle, true);
        normalize(magnitude, magn_norm, 0.0f, 1.0f, NORM_MINMAX);
        angle *= ((1.f / 360.f) * (180.f / 255.f));
 
        //build hsv image
        Mat _hsv[3], hsv, hsv8, bgr;
        _hsv[0] = angle;
        _hsv[1] = Mat::ones(angle.size(), CV_32F);
        _hsv[2] = magn_norm;
        merge(_hsv, 3, hsv);
        hsv.convertTo(hsv8, CV_8U, 255.0);
        cvtColor(hsv8, bgr, COLOR_HSV2BGR);
 		
 		imshow("original", frame2);
        imshow("frame2", bgr);
 
        int keyboard = waitKey(30);
        if (keyboard == 'q' || keyboard == 27)
            break;
 
        prvs = next;
    }
}

void mhi(cv::VideoCapture& cap) {
    double fps = 15.0;
	int frame_time_ms = static_cast<int>(1000.0 / fps);

    cv::Ptr<cv::BackgroundSubtractor> pBackSub = cv::createBackgroundSubtractorMOG2();
	cv::Mat prev_gray, motion_mask, mhi;
	double MHI_DURATION = 1; // in seconds
	double timestamp = 0;
	int frame_count = 0;

    cv::Mat frame, fgMask;
    while (true) {
        auto start = std::chrono::high_resolution_clock::now();

	    if (!cap.read(frame)) break;

	    pBackSub->apply(frame, fgMask);

	    cv::Mat gray;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        timestamp = static_cast<double>(cv::getTickCount()) / cv::getTickFrequency();

        if (!prev_gray.empty()) {
            // Step 1: Get motion mask
            cv::Mat diff;
            cv::absdiff(gray, prev_gray, diff);
            cv::threshold(diff, motion_mask, 30, 1, cv::THRESH_BINARY);

            // Step 2: Initialize mhi to match the frame size
            if (mhi.empty()) {
                mhi = cv::Mat::zeros(gray.size(), CV_32FC1);
            }

            // Step 3: Update motion history
            cv::motempl::updateMotionHistory(motion_mask, mhi, timestamp, MHI_DURATION);

            // Step 4: Convert to 8-bit image for display
            cv::Mat mhi_mask;
            mhi.convertTo(mhi_mask, CV_8UC1, 255.0 / MHI_DURATION);

            int erosion_type = 2;
            int erosion_elem = 2;

            int erosion_size = 1;

			if( erosion_elem == 0 ){ erosion_type = cv::MORPH_RECT; }
			else if( erosion_elem == 1 ){ erosion_type = cv::MORPH_CROSS; }
			else if( erosion_elem == 2) { erosion_type = cv::MORPH_ELLIPSE; }
			cv::Mat element = getStructuringElement( erosion_type,
			                   cv::Size( 2*erosion_size + 1, 2*erosion_size+1 ),
			                   cv::Point( erosion_size, erosion_size ) );
			erode( fgMask, fgMask, element );
			// erode( fgMask, fgMask, element );
			dilate(fgMask, fgMask, element);

            cv::imshow("Motion History", mhi_mask);
        }

        Mat frame_copy = frame.clone();
        std::vector<std::vector<cv::Point>> contours;
		cv::findContours(fgMask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
		for (const auto& contour : contours) {
		    double area = cv::contourArea(contour);
		    if (area < 30) continue;  // filter out small blobs

		    cv::Rect bbox = cv::boundingRect(contour);
		    cv::Point2f center(bbox.x + bbox.width/2, bbox.y + bbox.height/2);
		    
		    cv::rectangle(frame_copy, bbox, cv::Scalar(0, 255, 0), 2);
		    cv::circle(frame_copy, center, 3, cv::Scalar(0, 0, 255), -1);
		}
		cv::imshow("Tracked Blobs", frame_copy);


        prev_gray = gray.clone();
        cv::imshow("Current Frame", frame);
        imshow("FG Mask", fgMask);
        double timestamp = cap.get(cv::CAP_PROP_POS_MSEC);
        std::cout << "Frame Timestamp: " << timestamp << " ms" << std::endl;

        cv::waitKey(1);

	    int key = cv::waitKey(1);
	    if (key == 27) break;

	    auto end = std::chrono::high_resolution_clock::now();
	    auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

	    if (elapsed_ms < frame_time_ms) {
	        std::this_thread::sleep_for(std::chrono::milliseconds(frame_time_ms - elapsed_ms));
	    }

    }

}

int main(int argc, char* argv[]) {
    // Replace with your HLS (.m3u8) stream URL
    // std::string stream_url = "https://iowadotsfs1.us-east-1.skyvdn.com:443/rtplive/ictv03lb/playlist.m3u8";
    std::string stream_url = "https://iowadotsfs2.us-east-1.skyvdn.com:443/rtplive/ictv39lb/playlist.m3u8";
    // std::string stream_url = "https://iihrwc02.iowa.uiowa.edu/axis-cgi/mjpg/video.cgi?resolution=704x480&dummy=garb";

    std::string stream_url1 = "https://iowadotsfs1.us-east-1.skyvdn.com:443/rtplive/ictv03lb/playlist.m3u8";

    // Open the video stream
    cv::VideoCapture cap(stream_url1);
    if (!cap.isOpened()) {
        std::cerr << "Error: Could not open stream: " << cap.getBackendName() << std::endl;
        return -1;
    }

    mhi(cap);

    cap.release();
    cv::destroyAllWindows();

    return 0;
}

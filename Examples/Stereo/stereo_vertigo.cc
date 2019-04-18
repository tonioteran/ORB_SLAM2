/**
   @file stereo_vertigo.cc
   @brief ORBSLAM implementation for VERTIGO datasets
   @date Mar 25, 2019
   @author Antonio Teran (teran@mit.edu)
*/

#include <algorithm>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp> /*extra fix for CV_LOAD_IMAGE_UNCHANGED*/

#include <dynamic-slam/FrameManager.h>

#include <System.h>

using namespace std;

void LoadImages(const string &strPathToSequence, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &vTimestamps);

int main(int argc, char **argv) {
  if (argc != 4) {
    cerr << endl
         << "Usage: ./stereo_kitti path_to_vocabulary path_to_settings "
            "path_to_sequence"
         << endl;
    return 1;
  }

  // Configure the VERTIGO scenario and load the images
  const string yaml_file{
      "/home/tonio/repos/dynamic-slam/codebase/apps/config.yml"};
  // dps::FrameManager manager = dps::FrameManager(yaml_file);
  dps::FrameManager manager = dps::FrameManager(argv[3]);
  manager.loadImages();
  manager.print();

  // Retrieve paths to images
  vector<string> vstrImageLeft;
  vector<string> vstrImageRight;
  vector<double> vTimestamps;
  const int nImages = manager.getNumOfIms();

  // Create SLAM system. It initializes all system threads and gets ready to
  // process frames.
  ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::System::STEREO, true);

  cout << endl << "-------" << endl;
  cout << "Start processing sequence ..." << endl;
  cout << "Images in the sequence: " << nImages << endl << endl;

  // Main loop
  cv::Mat imLeft, imRight;
  for (int ni = 0; ni < nImages; ni++) {

    // Read left and right images from file
    dps::StereoFrame frame = manager.getStereoFrameByIdx(ni);
    imLeft = frame.img_left;
    imRight = frame.img_right;
    double tframe = frame.timestamp;

    if (imLeft.empty()) {
      cerr << endl << "Failed to load image at: " << ni << endl;
      return 1;
    }

#ifdef COMPILEDWITHC11
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
    std::chrono::monotonic_clock::time_point t1 =
        std::chrono::monotonic_clock::now();
#endif

    // Pass the images to the SLAM system
    SLAM.TrackStereo(imLeft, imRight, tframe);

#ifdef COMPILEDWITHC11
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
    std::chrono::monotonic_clock::time_point t2 =
        std::chrono::monotonic_clock::now();
#endif

    double ttrack =
        std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1)
            .count();

    // vTimesTrack[ni] = ttrack;

    // Wait to load the next frame
    /*
    double T = 0;
    if (ni < nImages - 1) {
      dps::StereoFrame next_frame = manager.getStereoFrameByIdx(ni + 1);
      T = next_frame.timestamp - tframe;
    } else if (ni > 0) {
      dps::StereoFrame prev_frame = manager.getStereoFrameByIdx(ni + 1);
      T = tframe - prev_frame.timestamp;
    }

    if (ttrack < T)
      usleep((T - ttrack) * 1e6);
    */
    usleep(300e3);
  }

  // Stop all threads
  cv::waitKey(0);
  SLAM.Shutdown();

  // Tracking time statistics
  /*
  sort(vTimesTrack.begin(), vTimesTrack.end());
  float totaltime = 0;
  for (int ni = 0; ni < nImages; ni++) {
    totaltime += vTimesTrack[ni];
  }
  */
  cout << "-------" << endl << endl;
  // cout << "median tracking time: " << vTimesTrack[nImages / 2] << endl;
  // cout << "mean tracking time: " << totaltime / nImages << endl;

  // Save camera trajectory
  SLAM.SaveTrajectoryKITTI("CameraTrajectory.txt");

  return 0;
}

void LoadImages(const string &strPathToSequence, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &vTimestamps) {
  ifstream fTimes;
  string strPathTimeFile = strPathToSequence + "/times.txt";
  fTimes.open(strPathTimeFile.c_str());
  while (!fTimes.eof()) {
    string s;
    getline(fTimes, s);
    if (!s.empty()) {
      stringstream ss;
      ss << s;
      double t;
      ss >> t;
      vTimestamps.push_back(t);
    }
  }

  string strPrefixLeft = strPathToSequence + "/image_0/";
  string strPrefixRight = strPathToSequence + "/image_1/";

  const int nTimes = vTimestamps.size();
  vstrImageLeft.resize(nTimes);
  vstrImageRight.resize(nTimes);

  for (int i = 0; i < nTimes; i++) {
    stringstream ss;
    ss << setfill('0') << setw(6) << i;
    vstrImageLeft[i] = strPrefixLeft + ss.str() + ".png";
    vstrImageRight[i] = strPrefixRight + ss.str() + ".png";
  }
}

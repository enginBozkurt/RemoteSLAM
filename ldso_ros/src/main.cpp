#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/PoseStamped.h>
#include "cv_bridge/cv_bridge.h"

#include <thread>
#include <clocale>
#include <csignal>
#include <cstdlib>
#include <cstdio>
#include <unistd.h>
#include <sys/time.h>
#include <glog/logging.h>
#include "frontend/FullSystem.h"
#include "frontend/Undistort.h"

#include "DatasetReader.h"


using namespace ldso;

std::string vignette = "/media/gaoxiang/Data1/Dataset/TUM-MONO/sequence_31/vignette.png";
std::string gammaCalib = "/media/gaoxiang/Data1/Dataset/TUM-MONO/sequence_31/pcalib.txt";
std::string source = "/media/gaoxiang/Data1/Dataset/TUM-MONO/sequence_31/";
std::string calib = "/media/gaoxiang/Data1/Dataset/TUM-MONO/sequence_31/camera.txt";
std::string output_file = "./results.txt";
std::string vocPath = "/home/yoni4/Desktop/LDSO/catkin_ws/src/ldso/vocab/orbvoc.dbow3";


double rescale = 1;
bool reversePlay = false;
bool disableROS = false;
int startIdx = 0;
int endIdx = 100000;
bool prefetch = false;
float playbackSpeed = 0;    // 0 for linearize (play as fast as possible, while sequentializing tracking & mapping). otherwise, factor on timestamps.
bool preload = false;
bool useSampleOutput = false;

void settingsDefault(int preset) {
    printf("\n=============== PRESET Settings: ===============\n");
    if (preset == 0 || preset == 1) {
        printf("DEFAULT settings:\n"
               "- %s real-time enforcing\n"
               "- 2000 active points\n"
               "- 5-7 active frames\n"
               "- 1-6 LM iteration each KF\n"
               "- original image resolution\n", preset == 0 ? "no " : "1x");

        playbackSpeed = (preset == 0 ? 0 : 1);
        preload = preset == 1;
        setting_desiredImmatureDensity = 1500;
        setting_desiredPointDensity = 2000;
        setting_minFrames = 5;
        setting_maxFrames = 7;
        setting_maxOptIterations = 6;
        setting_minOptIterations = 1;

        setting_logStuff = false;
    }

    if (preset == 2 || preset == 3) {
        printf("FAST settings:\n"
               "- %s real-time enforcing\n"
               "- 800 active points\n"
               "- 4-6 active frames\n"
               "- 1-4 LM iteration each KF\n"
               "- 424 x 320 image resolution\n",
               preset == 2 ? "no " : "5x");

        playbackSpeed = (preset == 2 ? 0 : 5);
        preload = preset == 3;
        setting_desiredImmatureDensity = 600;
        setting_desiredPointDensity = 800;
        setting_minFrames = 4;
        setting_maxFrames = 6;
        setting_maxOptIterations = 4;
        setting_minOptIterations = 1;

        benchmarkSetting_width = 424;
        benchmarkSetting_height = 320;

        setting_logStuff = false;
    }

    printf("==============================================\n");
}

void parseArgument(char *arg) {
    int option;
    float foption;
    char buf[1000];


    if (1 == sscanf(arg, "sampleoutput=%d", &option)) {
        if (option == 1) {
            useSampleOutput = true;
            printf("USING SAMPLE OUTPUT WRAPPER!\n");
        }
        return;
    }

    if (1 == sscanf(arg, "quiet=%d", &option)) {
        if (option == 1) {
            setting_debugout_runquiet = true;
            printf("QUIET MODE, I'll shut up!\n");
        }
        return;
    }

    if (1 == sscanf(arg, "preset=%d", &option)) {
        settingsDefault(option);
        return;
    }


    if (1 == sscanf(arg, "rec=%d", &option)) {
        if (option == 0) {
            disableReconfigure = true;
            printf("DISABLE RECONFIGURE!\n");
        }
        return;
    }


    if (1 == sscanf(arg, "noros=%d", &option)) {
        if (option == 1) {
            disableROS = true;
            disableReconfigure = true;
            printf("DISABLE ROS (AND RECONFIGURE)!\n");
        }
        return;
    }

    if (1 == sscanf(arg, "nolog=%d", &option)) {
        if (option == 1) {
            setting_logStuff = false;
            printf("DISABLE LOGGING!\n");
        }
        return;
    }
    if (1 == sscanf(arg, "reversePlay=%d", &option)) {
        if (option == 1) {
            reversePlay = true;
            printf("REVERSE!\n");
        }
        return;
    }
    if (1 == sscanf(arg, "nogui=%d", &option)) {
        if (option == 1) {
            disableAllDisplay = true;
            printf("NO GUI!\n");
        }
        return;
    }
    if (1 == sscanf(arg, "nomt=%d", &option)) {
        if (option == 1) {
            multiThreading = false;
            printf("NO MultiThreading!\n");
        }
        return;
    }
    if (1 == sscanf(arg, "prefetch=%d", &option)) {
        if (option == 1) {
            prefetch = true;
            printf("PREFETCH!\n");
        }
        return;
    }
    if (1 == sscanf(arg, "start=%d", &option)) {
        startIdx = option;
        printf("START AT %d!\n", startIdx);
        return;
    }
    if (1 == sscanf(arg, "end=%d", &option)) {
        endIdx = option;
        printf("END AT %d!\n", endIdx);
        return;
    }
    if (1 == sscanf(arg, "loopclosing=%d", &option)) {
        if (option == 1) {
            setting_enableLoopClosing = true;
        } else {
            setting_enableLoopClosing = false;
        }
        printf("Loopclosing %s!\n", setting_enableLoopClosing ? "enabled" : "disabled");
        return;
    }

    if (1 == sscanf(arg, "files=%s", buf)) {
        source = buf;
        printf("loading data from %s!\n", source.c_str());
        return;
    }

    if (1 == sscanf(arg, "vocab=%s", buf)) {
        vocPath = buf;
        printf("loading vocabulary from %s!\n", vocPath.c_str());
        return;
    }

    if (1 == sscanf(arg, "calib=%s", buf)) {
        calib = buf;
        printf("loading calibration from %s!\n", calib.c_str());
        return;
    }

    if (1 == sscanf(arg, "vignette=%s", buf)) {
        vignette = buf;
        printf("loading vignette from %s!\n", vignette.c_str());
        return;
    }

    if (1 == sscanf(arg, "gamma=%s", buf)) {
        gammaCalib = buf;
        printf("loading gammaCalib from %s!\n", gammaCalib.c_str());
        return;
    }

    if (1 == sscanf(arg, "rescale=%f", &foption)) {
        rescale = foption;
        printf("RESCALE %f!\n", rescale);
        return;
    }

    if (1 == sscanf(arg, "speed=%f", &foption)) {
        playbackSpeed = foption;
        printf("PLAYBACK SPEED %f!\n", playbackSpeed);
        return;
    }

    if (1 == sscanf(arg, "output=%s", buf)) {
        output_file = buf;
        LOG(INFO) << "output set to " << output_file << endl;
        return;
    }

    if (1 == sscanf(arg, "save=%d", &option)) {
        if (option == 1) {
            debugSaveImages = true;
            if (42 == system("rm -rf images_out"))
                printf("system call returned 42 - what are the odds?. This is only here to shut up the compiler.\n");
            if (42 == system("mkdir images_out"))
                printf("system call returned 42 - what are the odds?. This is only here to shut up the compiler.\n");
            if (42 == system("rm -rf images_out"))
                printf("system call returned 42 - what are the odds?. This is only here to shut up the compiler.\n");
            if (42 == system("mkdir images_out"))
                printf("system call returned 42 - what are the odds?. This is only here to shut up the compiler.\n");
            printf("SAVE IMAGES!\n");
        }
        return;
    }

    if (1 == sscanf(arg, "mode=%d", &option)) {
        if (option == 0) {
            printf("PHOTOMETRIC MODE WITH CALIBRATION!\n");
        }
        if (option == 1) {
            printf("PHOTOMETRIC MODE WITHOUT CALIBRATION!\n");
            setting_photometricCalibration = 0;
            setting_affineOptModeA = 0; //-1: fix. >=0: optimize (with prior, if > 0).
            setting_affineOptModeB = 0; //-1: fix. >=0: optimize (with prior, if > 0).
        }
        if (option == 2) {
            printf("PHOTOMETRIC MODE WITH PERFECT IMAGES!\n");
            setting_photometricCalibration = 0;
            setting_affineOptModeA = -1; //-1: fix. >=0: optimize (with prior, if > 0).
            setting_affineOptModeB = -1; //-1: fix. >=0: optimize (with prior, if > 0).
            setting_minGradHistAdd = 3;
        }
        return;
    }

    printf("could not parse argument \"%s\"!!!!\n", arg);
}



FullSystem* fullSystem = 0;
Undistort* undistorter = 0;
int frameID = 0;

void vidCb(const sensor_msgs::ImageConstPtr img)
{

    std::cout << " I AM HERE " << std::endl;

	cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
	assert(cv_ptr->image.type() == CV_8U);
	assert(cv_ptr->image.channels() == 1);

    std::cout << " I AM HERE 2" << std::endl;

	// if(setting_fullResetRequested)
	// {
	// 	std::vector<IOWrap::Output3DWrapper*> wraps = fullSystem->outputWrapper;
	// 	delete fullSystem;
	// 	for(IOWrap::Output3DWrapper* ow : wraps) ow->reset();
	// 	fullSystem = new FullSystem();
	// 	fullSystem->linearizeOperation=false;
	// 	fullSystem->outputWrapper = wraps;
	//     if(undistorter->photometricUndist != 0)
	//     	fullSystem->setGammaFunction(undistorter->photometricUndist->getG());
	// 	setting_fullResetRequested=false;
	// }

	MinimalImageB minImg((int)cv_ptr->image.cols, (int)cv_ptr->image.rows,(unsigned char*)cv_ptr->image.data);
    
    std::cout << " I AM HERE 2.1" << std::endl;

	ImageAndExposure* undistImg = undistorter->undistort<unsigned char>(&minImg, 1,0, 1.0f);
    std::cout << " I AM HERE 2.2" << std::endl;

	undistImg->timestamp=img->header.stamp.toSec(); // relay the timestamp to dso
    
    std::cout << " I AM HERE 3 " << std::endl;


	fullSystem->addActiveFrame(undistImg, frameID);
    std::cout << " I AM HERE 4 " << std::endl;

	frameID++;
    delete undistImg;
}


int main( int argc, char** argv )
{
	ros::init(argc, argv, "ldso_live");

    FLAGS_colorlogtostderr = true;
    for (int i = 1; i < argc; i++)
        parseArgument(argv[i]);

    // check setting conflicts
    if (setting_enableLoopClosing && (setting_pointSelection != 1)) {
        LOG(ERROR) << "Loop closing is enabled but point selection strategy is not set to LDSO, "
                      "use setting_pointSelection=1! please!" << endl;
        exit(-1);
    }

    if (setting_showLoopClosing) {
        LOG(WARNING) << "show loop closing results. The program will be paused when any loop is found" << endl;
    }

    shared_ptr<ImageFolderReader> reader(
            new ImageFolderReader(ImageFolderReader::TUM_MONO, source, calib, gammaCalib, vignette));

    reader->setGlobalCalibration();

    if (setting_photometricCalibration > 0 && reader->getPhotometricGamma() == 0) {
        LOG(ERROR) << "ERROR: dont't have photometric calibation. Need to use commandline options mode=1 or mode=2 ";
        exit(1);
    }

    shared_ptr<ORBVocabulary> voc(new ORBVocabulary());

    voc->load(vocPath);

    shared_ptr<FullSystem> fullSystem(new FullSystem(voc));
    fullSystem->setGammaFunction(reader->getPhotometricGamma());
    fullSystem->linearizeOperation = (playbackSpeed == 0);

    shared_ptr<PangolinDSOViewer> viewer = nullptr;
    if (!disableAllDisplay) {
        viewer = shared_ptr<PangolinDSOViewer>(new PangolinDSOViewer(wG[0], hG[0], false));
        fullSystem->setViewer(viewer);
    } else {
        LOG(INFO) << "visualization is disabled!" << endl;
    }

    ros::NodeHandle nh;
    ros::Subscriber imgSub = nh.subscribe("image", 1, &vidCb);
    ros::spin();

    if (viewer)
        viewer->run();  // mac os should keep this in main thread.


    LOG(INFO) << "EXIT NOW!";
    delete undistorter;
    // delete fullSystem;

	return 0;
}


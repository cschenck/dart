#include <iostream>
#include <memory>
#include <stdio.h>
#include <string.h>
#include <vector>

#include <GL/glew.h>
#include <GL/glut.h>

#include <cuda_runtime.h>
#include <cuda.h>
#include <cuda_gl_interop.h>

#include <ros/ros.h>

#include "tracker.h"
#include "tclap/CmdLine.h"
#include "ROSDepthSource.h"

#define EIGEN_DONT_ALIGN

using namespace std;


int main(int argc, char** argv)
{
    string depth_topic;
    string rgb_topic;
    string cmd_topic;
    string out_topic;
    try
    {
        TCLAP::CmdLine cmd("DART with ROS", ' ', "0.1");
        TCLAP::ValueArg<std::string> dt("d","depth_topic","The ROS topic to listen to for depth data.",false,"/camera/depth_registered/image_raw","string");
        TCLAP::ValueArg<std::string> rt("r","rgb_topic","The ROS topic to listen to for rgb data.",false,"/camera/rgb/image_color","string");
        TCLAP::ValueArg<std::string> ct("c","cmd_topic","The ROS topic to listen to for tracking commands.",false,"/dart/cmd","string");
        TCLAP::ValueArg<std::string> pt("p","pose_topic","The ROS topic to publish object poses to.",false,"/dart/pose","string");
        cmd.add(dt);
        cmd.add(rt);
        cmd.add(ct);
        cmd.add(pt);
        cmd.parse(argc, argv);
        depth_topic = dt.getValue();
        rgb_topic = rt.getValue();
        cmd_topic = ct.getValue();
        out_topic = pt.getValue();
    }
    catch(TCLAP::ArgException &e)
    {
        cerr << "error: " << e.error() << " for arg " << e.argId() << endl;
        return 1;
    }
    
    cout << "Listening for depth frames on topic: " << depth_topic << endl;
    cout << "Listening for rgb frames on topic: " << rgb_topic << endl;
    cout << "Listening for commands on topic: " << cmd_topic << endl;
    cout << "Publishing poses to topic: " << out_topic << endl;
    
    ROS_INFO("Starting dart node.");
    ros::init(argc, argv, "dart");
    
    cudaGLSetGLDevice(0);
    cudaDeviceReset();
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
    glutInitWindowPosition(-10000, -10000);
    glutInitWindowSize(1, 1);
    glutCreateWindow("ExtensionTest");
    glutHideWindow();
    
    
    if(glewInit() != GLEW_OK)
    {
        printf("Glew did not initialize properly.\n");
    }
    
    dart::Tracker tracker;
    auto depthSource = shared_ptr<ROSDepthSource>(new ROSDepthSource());
    depthSource->initialize(depth_topic, rgb_topic);
    depthSource->startRosSpinner();
    tracker.addDepthSource(depthSource.get());
    printf("here\n");
    cin.get();
    depthSource->stopRosSpinner();
    
    ros::shutdown();
        
    return 0;
}
    
    /*cudaSetDevice(0);
    cudaDeviceReset();

    const static int panelWidth = 180;
    pangolin::CreateWindowAndBind("Main",640+4*panelWidth+1,2*480+1);

    glewInit();
    dart::Tracker tracker;
    
    int glWidth = 640;
    int glHeight= 480;
    int glFL = 400;
    int glPPx = glWidth/2;
    int glPPy = glHeight/2;
    pangolin::OpenGlMatrixSpec glK = pangolin::ProjectionMatrixRDF_BottomLeft(glWidth,glHeight,glFL,glFL,glPPx,glPPy,0.01,1000);
    pangolin::OpenGlRenderState camState(glK);
    pangolin::View & camDisp = pangolin::Display("cam").SetAspect(640.0f/480.0f).SetHandler(new pangolin::Handler3D(camState));

    pangolin::View & imgDisp = pangolin::Display("img").SetAspect(640.0f/480.0f);
    pangolin::GlTexture imgTexDepthSize(320,240);
    pangolin::GlTexture imgTexPredictionSize(160,120);


    pangolin::Display("multi")
            .SetBounds(1.0, 0.0, pangolin::Attach::Pix(2*panelWidth), pangolin::Attach::Pix(-2*panelWidth))
            .SetLayout(pangolin::LayoutEqual)
            .AddDisplay(camDisp)
            .AddDisplay(imgDisp);
    
    auto depthSource = shared_ptr<ROSDepthSource>(new ROSDepthSource());
    depthSource->initialize(depth_topic, rgb_topic);
    depthSource->startRosSpinner();
    tracker.addDepthSource(depthSource.get());
    dart::Optimizer & optimizer = *tracker.getOptimizer();
    
    printf("1\n");
    
    int fpsWindow = 10;
    pangolin::basetime lastTime = pangolin::TimeNow();

    const int depthWidth = depthSource->getDepthWidth();
    const int depthHeight = depthSource->getDepthHeight();

    const int predWidth = tracker.getPredictionWidth();
    const int predHeight = tracker.getPredictionHeight();

    dart::MirroredVector<uchar3> imgDepthSize(depthWidth*depthHeight);
    dart::MirroredVector<uchar3> imgPredSize(predWidth*predHeight);
    dart::MirroredVector<const uchar3 *> allSdfColors(tracker.getNumModels());
    for (int m=0; m<tracker.getNumModels(); ++m) {
        allSdfColors.hostPtr()[m] = tracker.getModel(m).getDeviceSdfColors();
    }
    allSdfColors.syncHostToDevice();
    
    printf("2\n");

    // set up VBO to display point cloud
    GLuint pointCloudVbo,pointCloudColorVbo,pointCloudNormVbo;
    glGenBuffersARB(1,&pointCloudVbo);
    glBindBufferARB(GL_ARRAY_BUFFER_ARB,pointCloudVbo);
    glBufferDataARB(GL_ARRAY_BUFFER_ARB,depthWidth*depthHeight*sizeof(float4),tracker.getHostVertMap(),GL_DYNAMIC_DRAW_ARB);
    glGenBuffersARB(1,&pointCloudColorVbo);
    glBindBufferARB(GL_ARRAY_BUFFER_ARB,pointCloudColorVbo);
    glBufferDataARB(GL_ARRAY_BUFFER_ARB,depthWidth*depthHeight*sizeof(uchar3),imgDepthSize.hostPtr(),GL_DYNAMIC_DRAW_ARB);
    glGenBuffersARB(1,&pointCloudNormVbo);
    glBindBufferARB(GL_ARRAY_BUFFER_ARB,pointCloudNormVbo);
    glBufferDataARB(GL_ARRAY_BUFFER_ARB,depthWidth*depthHeight*sizeof(float4),tracker.getHostNormMap(),GL_DYNAMIC_DRAW_ARB);
    float fps;
    
    printf("3\n");
    
    for (int pangolinFrame=1; !pangolin::ShouldQuit(); ++pangolinFrame)
    {
        if (pangolin::HasResized()) {
            pangolin::DisplayBase().ActivateScissorAndClear();
        }
        
        if (pangolinFrame % fpsWindow == 0) {
            pangolin::basetime time = pangolin::TimeNow();
            fps = fpsWindow / pangolin::TimeDiff_s(lastTime,time);
            lastTime = time;
        }
        
        tracker.optimizePoses();
        
        float errPerObsPoint = optimizer.getErrPerObsPoint(1,0);
        float errPerModPoint = optimizer.getErrPerModPoint(1,0);

        
        glClearColor (1.0, 1.0, 1.0, 1.0);
        glShadeModel (GL_SMOOTH);
        float4 lightPosition = make_float4(normalize(make_float3(-0.4405,-0.5357,-0.619)),0);
        glLightfv(GL_LIGHT0, GL_POSITION, (float*)&lightPosition);

        camDisp.ActivateScissorAndClear(camState);

        glEnable(GL_DEPTH_TEST);
        glEnable(GL_LIGHT0);
        glEnable(GL_NORMALIZE);
        glEnable(GL_LIGHTING);

        camDisp.ActivateAndScissor(camState);

        glPushMatrix();
        
        glColor4ub(0xff,0xff,0xff,0xff);
        glEnable(GL_COLOR_MATERIAL);
        glPushMatrix();
        for (int m=0; m<tracker.getNumModels(); ++m) {
            tracker.updatePose(m);
            tracker.getModel(m).render();
        }
        glPopMatrix();
        
        glPointSize(1.0f);

        glPointSize(4.0f);
        glBindBufferARB(GL_ARRAY_BUFFER_ARB,pointCloudVbo);
        glBufferDataARB(GL_ARRAY_BUFFER_ARB,depthWidth*depthHeight*sizeof(float4),tracker.getHostVertMap(),GL_DYNAMIC_DRAW_ARB);

        glEnableClientState(GL_VERTEX_ARRAY);
        glDisableClientState(GL_NORMAL_ARRAY);
        glVertexPointer(4, GL_FLOAT, 0, 0);


        glBindBufferARB(GL_ARRAY_BUFFER_ARB,pointCloudColorVbo);
        glBufferDataARB(GL_ARRAY_BUFFER_ARB,depthWidth*depthHeight*sizeof(uchar3),depthSource->getColor(),GL_DYNAMIC_DRAW_ARB);
        glColorPointer(3,GL_UNSIGNED_BYTE, 0, 0);
        glEnableClientState(GL_COLOR_ARRAY);
        glDisable(GL_LIGHTING);

        glDrawArrays(GL_POINTS,0,depthWidth*depthHeight);
        glBindBuffer(GL_ARRAY_BUFFER_ARB,0);

        glDisableClientState(GL_VERTEX_ARRAY);
        glDisableClientState(GL_COLOR_ARRAY);
        glDisableClientState(GL_NORMAL_ARRAY);

        glPointSize(1.0f);
        
        glPopMatrix();

        imgDisp.ActivateScissorAndClear();
        glDisable(GL_LIGHTING);
        glColor4ub(255,255,255,255);
        
        cudaError_t err = cudaGetLastError();
        if (err != cudaSuccess) {
            std::cerr << cudaGetErrorString(err) << std::endl;
        }
        
        pangolin::FinishFrame();
        
        tracker.stepForward();

    }
     
    glDeleteBuffersARB(1,&pointCloudVbo);
    glDeleteBuffersARB(1,&pointCloudColorVbo);
    glDeleteBuffersARB(1,&pointCloudNormVbo);
    
    depthSource->stopRosSpinner();*/



































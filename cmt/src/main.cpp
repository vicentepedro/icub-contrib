/* 
 * Copyright (C) 2014 iCub Facility - Istituto Italiano di Tecnologia
 * Author: Ugo Pattacini
 * email:  ugo.pattacini@iit.it
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/

/** 
\defgroup icub_cmt CMT Tracker 
 
A tracker based on the <a 
href="http://www.gnebehay.com/cmt">CMT</a> algorithm. 
 
\section intro_sec Description 
This is a yarp wrapper of the CMT algorithm (a.k.a. 
<b>Consensus-based-Matching</b>) for tracking objects. The C++ 
version of the method is taken verbatim from the <a 
href="https://github.com/delmottea/libCMT">libCMT</a> library. 
 
Available RPC services are: 
 
(notation: [.] identifies a vocab, <.> specifies a double,
"." specifies a string) 
 
<b>INIT</b> \n
action: init CMT with the template given by the bounding box. \n
format: [init] <top-left.x> <top-left.y> <bottom-right.x> 
<bottom-right.y> \n 
 
<b>START</b> \n
format: [start] \n 
action: start tracking.
 
<b>STOP</b> \n
format: [stop] \n 
action: stop tracking.

\section lib_sec Libraries 
- YARP libraries. 
- OpenCV (with C++ layer). 

\section portsc_sec Ports Created 
- \e /<modName>/img:i receives the image acquired from the 
  camera.
 
- \e /<modName>/img:o streams out the image containing 
  tracked object.
 
- \e /<modName>/data:o streams out information about the tracked
  object in the following format: <i><top-left.x> <top-left.y>
  <top-right.x> <top-right.y> <bottom-right.x>
  <bottom-right.y> <bottom-left.x> <bottom-left.y></i>.
 
- \e /<modName>/rpc processes rpc requests.
 
\section parameters_sec Parameters 
--name \e name
- specify the module stem-name, which is \e cmt by default. The
  stem-name is used as prefix for all open ports.
 
\section tested_os_sec Tested OS
Linux, Windows

\author Ugo Pattacini
*/ 

#include <stdio.h>
#include <string>

#include <cv.h>
#include <cmt.h>

#include <yarp/os/all.h>
#include <yarp/sig/all.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;


/************************************************************************/
class TrackerModule: public RFModule
{
protected:
    CMT *tracker;
    Mutex mutex;

    cv::Point2f tl;
    cv::Point2f br;

    bool initBoundingBox;
    bool doCMT;

    BufferedPort<ImageOf<PixelBgr> > imgInPort;
    Port                             imgOutPort;
    Port                             dataOutPort;
    RpcServer                        rpcPort;

public:
    /************************************************************************/
    TrackerModule() : tracker(new CMT) { }

    /************************************************************************/
    bool configure(ResourceFinder &rf)
    {
        string name=rf.check("name",Value("cmt")).asString().c_str();

        imgInPort.open(("/"+name+"/img:i").c_str());
        imgOutPort.open(("/"+name+"/img:o").c_str());
        dataOutPort.open(("/"+name+"/data:o").c_str());
        rpcPort.open(("/"+name+"/rpc").c_str());
        attach(rpcPort);

        initBoundingBox=doCMT=false;
        return true;
    }

    /************************************************************************/
    bool respond(const Bottle &command, Bottle &reply)
    {
        int ack=Vocab::encode("ack");
        int nack=Vocab::encode("nack");

        if (command.size()>0)
        {
            switch (command.get(0).asVocab())
            {
                //-----------------
                case VOCAB4('i','n','i','t'):
                {
                    if (command.size()>=5)
                    {
                        mutex.lock();
                        tl.x=command.get(1).asInt();
                        tl.y=command.get(2).asInt();
                        br.x=command.get(3).asInt();
                        br.y=command.get(4).asInt();
                        initBoundingBox=true;
                        mutex.unlock();

                        reply.addVocab(ack);
                    }
                    else
                        reply.addVocab(nack);

                    return true;
                }

                //-----------------
                case VOCAB4('s','t','a','r'):
                {
                    mutex.lock();
                    doCMT=true;
                    mutex.unlock();

                    reply.addVocab(ack);
                    return true;
                }

                //-----------------
                case VOCAB4('s','t','o','p'):
                {
                    mutex.lock();
                    doCMT=false;
                    mutex.unlock();

                    reply.addVocab(ack);
                    return true;
                }

                //-----------------
                default:
                    return RFModule::respond(command,reply);
            }
        }

        reply.addVocab(nack);
        return true;
    }

    /************************************************************************/
    double getPeriod()
    {
        return 0.0;
    }

    /************************************************************************/
    bool updateModule()
    {
        ImageOf<PixelBgr> *img=imgInPort.read();
        if (img==NULL)
            return true;

        mutex.lock();

        ImageOf<PixelMono> imgMono;
        imgMono.resize(img->width(),img->height());

        cv::Mat imgMat((IplImage*)img->getIplImage());
        cv::Mat imgMonoMat((IplImage*)imgMono.getIplImage());
        cv::cvtColor(imgMat,imgMonoMat,CV_BGR2GRAY);

        if (initBoundingBox)
        {
            tracker->initialise(imgMonoMat,tl,br);
            initBoundingBox=false;
        }

        if (doCMT)
        {
            tracker->processFrame(imgMonoMat);
            if (dataOutPort.getOutputCount()>0)
            {
                Bottle data;                    
                data.addInt(tracker->topLeft.x);
                data.addInt(tracker->topLeft.y);
                data.addInt(tracker->topRight.x);
                data.addInt(tracker->topRight.y);
                data.addInt(tracker->bottomRight.x);
                data.addInt(tracker->bottomRight.y);
                data.addInt(tracker->bottomLeft.x);
                data.addInt(tracker->bottomLeft.y);
                dataOutPort.write(data);
            }

            if (imgOutPort.getOutputCount()>0)
            {
                cv::line(imgMat,tracker->topLeft,tracker->topRight,cv::Scalar(255,0,0));
                cv::line(imgMat,tracker->topRight,tracker->bottomRight,cv::Scalar(255,0,0));
                cv::line(imgMat,tracker->bottomRight,tracker->bottomLeft,cv::Scalar(255,0,0));
                cv::line(imgMat,tracker->bottomLeft,tracker->topLeft,cv::Scalar(255,0,0));
                imgOutPort.write(*img);
            }
        }

        mutex.unlock();

        return true;
    }

    /************************************************************************/
    bool interruptModule()
    {
        imgInPort.interrupt();
        imgOutPort.interrupt();
        dataOutPort.interrupt();
        rpcPort.interrupt();

        return true;
    }

    /************************************************************************/
    bool close()
    {
        imgInPort.close();
        imgOutPort.close();
        dataOutPort.close();
        rpcPort.close();

        return true;
    }

    /************************************************************************/
    ~TrackerModule()
    {
        delete tracker;
    }
};


/****************************************************************/
int main(int argc, char *argv[])
{
    Network yarp;
    if (!yarp.checkNetwork())
    {
        printf("YARP server not available!\n");
        return -1;
    }

    ResourceFinder rf;
    rf.configure(argc,argv);

    TrackerModule mod;
    return mod.runModule(rf);
}



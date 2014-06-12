/* 
 * Copyright (C) 2013 iCub Facility - Istituto Italiano di Tecnologia
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
\defgroup icub_tld TLD Tracker 
 
A tracker based on the <a 
href="http://gnebehay.github.io/OpenTLD/">OpenTLD</a> library. 
 
\section intro_sec Description 
This is a yarp wrapper of the TLD algorithm (a.k.a. 
<b>Predator</b>) performing Tracking, Learning and Detection. 
 
Available RPC services are: 
 
(notation: [.] identifies a vocab, <.> specifies a double,
"." specifies a string) 
 
<b>LOAD</b> \n
format #1: [load] "model.md" \n 
reply: [nack]|[ack] "path_to_model.md" \n
action: load the model from the specified file. \n
format #2: [load] <top-left.x> <top-left.y> <bottom-right.x> 
<bottom-right.y> \n 
action: init TLD with the template given by the bounding box.
 
<b>SAVE</b> \n
format: [save] "model.md" \n
reply: [nack]|[ack] "path_to_model.md" \n 
action: store the model in the specified file.
 
<b>START</b> \n
format: [start] \n 
action: start tracking/learning/detecting.
 
<b>STOP</b> \n
format: [stop] \n 
action: stop tracking/learning/detecting.
 
<b>CLEAR</b> \n
format: [clear] \n 
action: clear the content of TLD. 
 
<b>SET</b> \n
format: [set] [learn]|[alternating] [on]|[off] \n 
action: enable/disable learning | alternating mode. \n 
In alternating mode, the detection is off while tracking.
 
<b>GET</b> \n
format: [get] [learn]|[alternating] \n 
reply: [nack]|[ack] [on]|[off] \n 
action: retrieve learning | alternating current status.
 
\section lib_sec Libraries 
- YARP libraries. 
- OpenCV (with C++ layer). 
- OpenTLD. 
 
\section openTLD_sec Installing OpenTLD 
The steps are: \n
1. git clone https://github.com/gnebehay/OpenTLD.git .\n
2. compile the library with default values of cmake variables. 
\n 
3. set up the environment variable <b>OpenTLD_ROOT</b> pointing 
 to the root directory. \n
4. set up the environment variable <b>OpenTLD_DIR</b> pointing 
 to the build directory. \n
5. For Windows systems only, when cmaking TLD project, select 
 which sub path to the OpenTLD libraries is to be used, if
 Release (default) or Debug, by filling the
 <b>OpenTLD_WIN_LIB_SUBPATH</b> variable.
 
\section portsc_sec Ports Created 
- \e /<modName>/img:i receives the image acquired from the 
  camera.
 
- \e /<modName>/img:o streams out the image containing 
  tracked object.
 
- \e /<modName>/data:o streams out information about the tracked
  object in the following format: <i><top-left.x> <top-left.y>
  <bottom-right.x> <bottom-right.y> <confidence></i>.
 
- \e /<modName>/rpc processes rpc requests.
 
\section parameters_sec Parameters 
--name \e name
- specify the module stem-name, which is \e tld by default. The
  stem-name is used as prefix for all open ports.
 
--context \e path 
- specify the path to the model files to be loaded/written. 
 
--model \e file 
- specify the model file to load the TLD with at start-up. 

\section tested_os_sec Tested OS
Linux, Windows

\author Ugo Pattacini
*/ 

#include <stdio.h>
#include <string>

#include <cv.h>
#include <TLD.h>

#include <yarp/os/all.h>
#include <yarp/sig/all.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;


/************************************************************************/
class TrackerModule: public RFModule
{
protected:
    tld::TLD *tracker;

    ResourceFinder *rf;
    Mutex mutex;
    cv::Rect boundingBox;

    bool initDetectorCascade;
    bool loadBoundingBox;
    bool doTLD;

    BufferedPort<ImageOf<PixelBgr> > imgInPort;
    Port                             imgOutPort;
    Port                             dataOutPort;
    RpcServer                        rpcPort;

public:
    /************************************************************************/
    TrackerModule() : tracker(new tld::TLD) { }

    /************************************************************************/
    bool configure(ResourceFinder &rf)
    {
        this->rf=&rf;
        string name=rf.check("name",Value("tld")).asString().c_str();
        if (rf.check("model"))
        {            
            string model=rf.findFile("model").c_str();
            tracker->readFromFile(model.c_str());
            printf("%s loaded\n",model.c_str());
        }

        loadBoundingBox=doTLD=false;
        initDetectorCascade=true;

        imgInPort.open(("/"+name+"/img:i").c_str());
        imgOutPort.open(("/"+name+"/img:o").c_str());
        dataOutPort.open(("/"+name+"/data:o").c_str());
        rpcPort.open(("/"+name+"/rpc").c_str());
        attach(rpcPort);

        return true;
    }

    /************************************************************************/
    bool respond(const Bottle &command, Bottle &reply)
    {
        int on=Vocab::encode("on");
        int off=Vocab::encode("off");
        int ack=Vocab::encode("ack");
        int nack=Vocab::encode("nack");

        if (command.size()>0)
        {
            switch (command.get(0).asVocab())
            {
                //-----------------
                case VOCAB4('l','o','a','d'):
                {
                    if (command.size()>=2)
                    {
                        if (command.get(1).isString())
                        {
                            mutex.lock();
                            string model=rf->findFileByName(command.get(1).asString()).c_str();
                            tracker->readFromFile(model.c_str());
                            mutex.unlock();

                            reply.addVocab(ack);
                            reply.addString(model.c_str());
                        }
                        else if (command.size()>=5)
                        {
                            mutex.lock();
                            boundingBox.x=command.get(1).asInt();
                            boundingBox.y=command.get(2).asInt();
                            boundingBox.width=command.get(3).asInt()-boundingBox.x;
                            boundingBox.height=command.get(4).asInt()-boundingBox.y;                            
                            loadBoundingBox=true;
                            mutex.unlock();

                            reply.addVocab(ack);
                        }
                        else
                            reply.addVocab(nack);
                    }
                    else
                        reply.addVocab(nack);

                    return true;
                }

                //-----------------
                case VOCAB4('s','a','v','e'):
                {
                    if (command.size()>=2)
                    {
                        mutex.lock();
                        string model=rf->getHomeContextPath().c_str();
                        model+="/";
                        model+=command.get(1).asString().c_str();
                        tracker->writeToFile(model.c_str());
                        mutex.unlock();

                        reply.addVocab(ack);
                        reply.addString(model.c_str());
                    }
                    else
                        reply.addVocab(nack);

                    return true;
                }

                //-----------------
                case VOCAB4('s','t','a','r'):
                {
                    mutex.lock();
                    doTLD=true;
                    mutex.unlock();

                    reply.addVocab(ack);
                    return true;
                }

                //-----------------
                case VOCAB4('s','t','o','p'):
                {
                    mutex.lock();
                    doTLD=false;
                    mutex.unlock();

                    reply.addVocab(ack);
                    return true;
                }

                //-----------------
                case VOCAB4('c','l','e','a'):
                {
                    mutex.lock();
                    tracker->release();
                    mutex.unlock();

                    reply.addVocab(ack);
                    return true;
                }

                //-----------------
                case VOCAB3('s','e','t'):
                {
                    if (command.size()>=3)
                    {
                        int subcmd=command.get(1).asVocab();
                        int mode=command.get(2).asVocab();
                        switch (subcmd)
                        {
                            //-----------------
                            case VOCAB4('l','e','a','r'):
                            {
                                if ((mode==on) || (mode==off))
                                {
                                    mutex.lock();
                                    tracker->learningEnabled=(mode==on);
                                    mutex.unlock();
                                    reply.addVocab(ack);
                                }
                                else
                                    reply.addVocab(nack);

                                break;
                            }

                            //-----------------
                            case VOCAB4('a','l','t','e'):
                            {
                                if ((mode==on) || (mode==off))
                                {
                                    mutex.lock();
                                    tracker->alternating=(mode==on);
                                    mutex.unlock();
                                    reply.addVocab(ack);
                                }
                                else
                                    reply.addVocab(nack);

                                break;
                            }

                            //-----------------
                            default:
                                reply.addVocab(nack);
                        }
                    }
                    else
                        reply.addVocab(nack);

                    return true;
                }

                //-----------------
                case VOCAB3('g','e','t'):
                {
                    if (command.size()>=2)
                    {
                        int subcmd=command.get(1).asVocab();
                        switch (subcmd)
                        {
                            //-----------------
                            case VOCAB4('l','e','a','r'):
                            {
                                reply.addVocab(ack);
                                reply.addVocab(tracker->learningEnabled?on:off);
                                break;
                            }

                            //-----------------
                            case VOCAB4('a','l','t','e'):
                            {
                                reply.addVocab(ack);
                                reply.addVocab(tracker->alternating?on:off);
                                break;
                            }

                            //-----------------
                            default:
                                reply.addVocab(nack);
                        }
                    }
                    else
                        reply.addVocab(nack);

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

        cv::Mat imgMat((IplImage*)img->getIplImage());
        if (initDetectorCascade)
        {
            ImageOf<PixelMono> imgMono;
            imgMono.resize(img->width(),img->height());
            cv::Mat imgMonoMat((IplImage*)imgMono.getIplImage());

            tracker->detectorCascade->imgWidth=imgMono.width();
            tracker->detectorCascade->imgHeight=imgMono.height();
            tracker->detectorCascade->imgWidthStep=((IplImage*)imgMono.getIplImage())->widthStep;
            initDetectorCascade=false;
        }

        if (loadBoundingBox)
        {
            ImageOf<PixelMono> imgMono;
            imgMono.resize(img->width(),img->height());
            
            cv::Mat imgMonoMat((IplImage*)imgMono.getIplImage());
            cv::cvtColor(imgMat,imgMonoMat,CV_BGR2GRAY);

            tracker->selectObject(imgMonoMat,&boundingBox);
            loadBoundingBox=false;
        }

        if (doTLD)
        {
            tracker->processImage(imgMat);
            if (tracker->currBB!=NULL)
            {
                cv::Point tl,br;
                tl.x=tracker->currBB->x;
                tl.y=tracker->currBB->y;
                br.x=tl.x+tracker->currBB->width;
                br.y=tl.y+tracker->currBB->height;

                if (dataOutPort.getOutputCount()>0)
                {
                    Bottle data;                    
                    data.addInt(tl.x);
                    data.addInt(tl.y);
                    data.addInt(br.x);
                    data.addInt(br.y);
                    data.addDouble(tracker->currConf);
                    dataOutPort.write(data);
                }

                if (imgOutPort.getOutputCount()>0)
                {
                    cv::rectangle(imgMat,tl,br,(tracker->currConf>=0.5)?
                                  cv::Scalar(255,0,0):cv::Scalar(0,255,255),2);
                    imgOutPort.write(*img);
                }
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
    rf.setDefaultContext("tld");
    rf.configure(argc,argv);

    TrackerModule mod;
    return mod.runModule(rf);
}



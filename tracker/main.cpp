/* 
 * Copyright (C) 2016 iCub Facility - Istituto Italiano di Tecnologia
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

#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>

#include <yarp/os/all.h>
#include <yarp/sig/all.h>

using namespace yarp::os;
using namespace yarp::sig;


/**************************************************/
class TrackerModule : public RFModule
{
    cv::Ptr<cv::Tracker> tracker;

    BufferedPort<ImageOf<PixelRgb> > iPort;
    BufferedPort<ImageOf<PixelRgb> > oPort;
    RpcServer rpcPort;

    Mutex mutex;
    cv::Rect2d initRect;
    enum { idle, init, run };
    int state;

public:
    /**************************************************/
    bool configure(ResourceFinder &rf)
    {
        iPort.open("/tracker:i");
        oPort.open("/tracker:o");
        rpcPort.open("/tracker:rpc");
        attach(rpcPort);

        state=idle;
        return true;
    }

    /**************************************************/
    bool respond(const Bottle& command, Bottle& reply)
    {
        LockGuard lg(mutex);
        int cmd=command.get(0).asVocab();

        if (cmd==Vocab::encode("stop"))
        {
            state=idle;
            reply.addVocab(Vocab::encode("ack"));
            return true;
        }
        else if (cmd==Vocab::encode("start"))
        {
            if (command.size()>=5)
            {
                initRect.x=command.get(1).asInt();
                initRect.y=command.get(2).asInt();
                initRect.width=command.get(3).asInt();
                initRect.height=command.get(4).asInt();                
                state=init;
                reply.addVocab(Vocab::encode("ack"));
            }
            else
                reply.addVocab(Vocab::encode("nack"));
            return true;
        }
        else
            return RFModule::respond(command,reply);
    }

    /**************************************************/
    double getPeriod()
    {
        return 0.0;
    }

    /**************************************************/
    bool updateModule()
    {
        if (ImageOf<PixelRgb> *iImg=iPort.read())
        {
            LockGuard lg(mutex);
            ImageOf<PixelRgb> &oImg=oPort.prepare();
            oImg=*iImg;

            cv::Mat frame=cv::cvarrToMat((IplImage*)oImg.getIplImage());
            cv::Rect2d result;
            
            if (state==init)
            {
                tracker=cv::Tracker::create("BOOSTING");
                tracker->init(frame,initRect);
                result=initRect;
                state=run;
            }
            else if (state==run)
                tracker->update(frame,result);

            if (state!=idle)
                cv::rectangle(frame,
                              cv::Point((int)result.x,(int)result.y),
                              cv::Point((int)(result.x+result.width),(int)(result.y+result.height)),
                              cv::Scalar(0,255,0),2);

            oPort.write();
        }

        return true;
    }

    /**************************************************/
    bool interruptModule()
    {
        iPort.interrupt();
        return true;
    }

    /**************************************************/
    bool close()
    {
        rpcPort.close();
        iPort.close();
        oPort.close();
        return true;
    }
};


/**************************************************/
int main(int argc, char* argv[])
{
    Network yarp;
    if (!yarp.checkNetwork())
    {
        yError("YARP server not available!");
        return 1;
    }

    ResourceFinder rf;
    rf.configure(argc,argv);

    TrackerModule tracker;
    return tracker.runModule(rf);   
}


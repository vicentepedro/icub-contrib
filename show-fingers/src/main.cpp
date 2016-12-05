/*
 * Copyright (C) 2016 iCub Facility - Istituto Italiano di Tecnologia
 * Author: Vadim Tikhanoff, Ugo Pattacini
 * email:  vadim.tikhanoff@iit.it, ugo.pattacini@iit.it
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

#include <memory>
#include <string>
#include <deque>

#include <opencv2/opencv.hpp>

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/dev/all.h>
#include <yarp/math/Math.h>
#include <yarp/math/Rand.h>
#include <yarp/math/NormRand.h>

#include <iCub/ctrl/math.h>
#include <iCub/iKin/iKinFwd.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;
using namespace iCub::iKin;


/************************************************************************/
class FingerModule : public RFModule
{
protected:
    iCubFinger finger[3];
    int camSel,nEncs;

    PolyDriver         drvAnalog,drvArm,drvCart,drvGaze;
    IAnalogSensor     *ianalog;
    IEncoders         *iencs;
    ICartesianControl *iarm;
    IGazeControl      *igaze;

    BufferedPort<ImageOf<PixelRgb> > imgInPort,imgOutPort;

public:
    /************************************************************************/
    bool configure(ResourceFinder &rf)
    {
        string robot=rf.check("robot",Value("icub")).asString();
        string arm=rf.check("arm",Value("left")).asString();
        string eye=rf.check("eye",Value("left")).asString();
        bool analog=(rf.check("analog",Value(robot=="icub"?"on":"off")).asString()=="on");

        if ((arm!="left") && (arm!="right"))
        {
            yError()<<"Invalid arm requested";
            return false;
        }

        if ((eye!="left") && (eye!="right"))
        {
            yError()<<"Invalid eye requested";
            return false;
        }

        // open drivers
        if (analog)
        {
            Property optionAnalog("(device analogsensorclient)");
            optionAnalog.put("remote","/"+robot+"/"+arm+"_hand/analog:o");
            optionAnalog.put("local","/show-fingers/analog");
            if (!drvAnalog.open(optionAnalog))
            {
                yError()<<"Analog sensor not available";
                terminate();
                return false;
            }
        }

        Property optionArm("(device remote_controlboard)");
        optionArm.put("remote","/"+robot+"/"+arm+"_arm");
        optionArm.put("local","/show-fingers/joints");
        if (!drvArm.open(optionArm))
        {
            yError()<<"Joints arm controller not available";
            terminate();
            return false;
        }

        Property optionCart("(device cartesiancontrollerclient)");
        optionCart.put("remote","/"+robot+"/cartesianController/"+arm+"_arm");
        optionCart.put("local","/show-fingers/cartesian");
        if (!drvCart.open(optionCart))
        {
            yError()<<"Cartesian arm controller not available";
            terminate();
            return false;
        }

        Property optionGaze("(device gazecontrollerclient)");
        optionGaze.put("remote","/iKinGazeCtrl");
        optionGaze.put("local","/show-fingers/gaze");
        if (!drvGaze.open(optionGaze))
        {
            yError()<<"Gaze controller not available";
            terminate();
            return false;
        }

        if (analog)
            drvAnalog.view(ianalog);
        else
            ianalog=NULL;
        IControlLimits *ilim;
        drvArm.view(iencs);
        drvArm.view(ilim);
        drvCart.view(iarm);
        drvGaze.view(igaze);
        iencs->getAxes(&nEncs);

        finger[0]=iCubFinger(arm+"_thumb");
        finger[1]=iCubFinger(arm+"_index");
        finger[2]=iCubFinger(arm+"_middle");

        deque<IControlLimits*> lim;
        lim.push_back(ilim);
        for (int i=0; i<3; i++)
            finger[i].alignJointsBounds(lim);

        yInfo()<<"Using arm="<<arm<<" and eye="<<eye;

        imgInPort.open("/show-fingers/img:i");
        imgOutPort.open("/show-fingers/img:o");

        camSel=(eye=="left")?0:1;
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
        ImageOf<PixelRgb> *imgIn=imgInPort.read(true);
        if (imgIn==NULL)
            return false;

        ImageOf<PixelRgb> &imgOut=imgOutPort.prepare();
        imgOut=*imgIn;

        cv::Mat img=cv::cvarrToMat(imgOut.getIplImage());

        Vector xa,oa;
        iarm->getPose(xa,oa);

        Matrix Ha=axis2dcm(oa);
        xa.push_back(1.0);
        Ha.setCol(3,xa);

        Vector pc;
        igaze->get2DPixel(camSel,xa,pc);

        cv::Point point_c((int)pc[0],(int)pc[1]);
        cv::circle(img,point_c,4,cv::Scalar(0,255,0),4);

        Vector analogs,encs(nEncs),joints;
        if (ianalog!=NULL)
            ianalog->read(analogs);
        iencs->getEncoders(encs.data());

        for (int i=0; i<3; i++)
        {
            if (ianalog!=NULL)
                finger[i].getChainJoints(encs,analogs,joints);
            else
                finger[i].getChainJoints(encs,joints);
            finger[i].setAng(CTRL_DEG2RAD*joints);
        }

        for (int fng=0; fng<3; fng++)
        {
            deque<cv::Point> point_f;
            for (int i=-1; i<(int)finger[fng].getN(); i++)
            {
                Vector fc;
                igaze->get2DPixel(camSel,Ha*(i<0?finger[fng].getH0().getCol(3):
                                                 finger[fng].getH(i,true).getCol(3)),fc);
                point_f.push_front(cv::Point((int)fc[0],(int)fc[1]));
                cv::circle(img,point_f.front(),3,cv::Scalar(0,0,255),4);

                if (i>=0)
                {
                    cv::line(img,point_f.front(),point_f.back(),cv::Scalar(255,255,255),2);
                    point_f.pop_back();
                }
                else
                    cv::line(img,point_c,point_f.front(),cv::Scalar(255,0,0),2);
            }
        }

        imgOutPort.writeStrict();
        return true;
    }

    /************************************************************************/
    void terminate()
    {
        if (!imgInPort.isClosed())
            imgInPort.close();

        if (!imgOutPort.isClosed())
            imgOutPort.close();

        if (drvGaze.isValid())
            drvGaze.close();

        if (drvCart.isValid())
            drvCart.close();

        if (drvArm.isValid())
            drvArm.close();

        if (drvAnalog.isValid())
            drvAnalog.close();
    }

    /************************************************************************/
    bool interruptModule()
    {
        imgInPort.interrupt();
        return true;
    }

    /************************************************************************/
    bool close()
    {
        terminate();
        return true;
    }
};


/************************************************************************/
int main(int argc, char *argv[])
{
    Network yarp;
    if (!yarp.checkNetwork())
    {
        yError()<<"YARP seems unavailable";
        return 1;
    }

    ResourceFinder rf;
    rf.configure(argc,argv);

    FingerModule module;
    return module.runModule(rf);
}

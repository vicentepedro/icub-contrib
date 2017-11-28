/*
 * Copyright (C) 2016 iCub Facility - Istituto Italiano di Tecnologia
 * Copyright (C) 2017 VisLab, Institute for Systems and Robotics,
 *                Instituto Superior Técnico, Universidade de Lisboa, Lisbon, Portugal
 * Author: Vadim Tikhanoff, Ugo Pattacini, Pedro Vicente (adaptation)
 * email:  vadim.tikhanoff@iit.it, ugo.pattacini@iit.it, pvicente@isr.tecnico.ulisboa.pt
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
    int camSel,nEncs,nEncsT;

    PolyDriver         drvAnalog,drvArm,drvCart,drvGaze,drvTorso;
    IAnalogSensor     *ianalog;
    IEncoders         *iencs,*iencsTorso;
    ICartesianControl *iarm;
    IGazeControl      *igaze;

    BufferedPort<ImageOf<PixelRgb> > imgInPort,imgOutPort,imgOutPortCorrect;
    BufferedPort<Bottle> offsetsPort; //input

    BufferedPort<Bottle> armPort, headPort, torsoPort; //input for offline mode

    Bottle estimatedOffsets;
    iCubArm *armKin; 
    iCubTorso torsokin;
    iCubEye *eyeKin;
    Vector encArm, encHead, encTorso;
    Matrix intrinsics;
    bool withOffsets,offline;

public:
    /************************************************************************/
    bool configure(ResourceFinder &rf)
    {
        string robot=rf.check("robot",Value("icub")).asString();
        string arm=rf.check("arm",Value("left")).asString();
        string eye=rf.check("eye",Value("left")).asString();
        bool analog=(rf.check("analog",Value(robot=="icub"?"on":"off")).asString()=="on");
        offline = rf.check("offline",Value("off")).asString()=="on"?true:false;
        withOffsets=rf.check("offsets",Value(false)).asBool();

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

        finger[0]=iCubFinger(arm+"_thumb");
        finger[1]=iCubFinger(arm+"_index");
        finger[2]=iCubFinger(arm+"_middle");

        if (!offline) //Do not open devices to connect to the robot
        {
            // open drivers
            if (withOffsets) //Open Torso
            {
                Property optionTorso("(device remote_controlboard)");
                optionTorso.put("remote",("/"+robot+"/torso").c_str());
                optionTorso.put("local","/show-fingers/torso");
                if (!drvTorso.open(optionTorso))
                {
                    yError()<<"Joints Torso controller not available";
                    terminate();
                    return false;
                }
            }
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
            if(withOffsets)
                drvTorso.view(iencsTorso);
            else
                iencsTorso = NULL;
            yInfo() << "aqui1";
            drvCart.view(iarm);
            IControlLimits *ilim;
            drvArm.view(iencs);
            drvArm.view(ilim);
            drvGaze.view(igaze);
            iencs->getAxes(&nEncs);
            if(withOffsets) 
                iencsTorso->getAxes(&nEncsT);
            else
                nEncsT=0;
            deque<IControlLimits*> lim;
            lim.push_back(ilim);
            for (int i=0; i<3; i++)
                finger[i].alignJointsBounds(lim);
        }
        else //OFFLINE MODE
        {
            iarm=NULL;
            igaze=NULL;  
            ianalog=NULL;         
            iencsTorso = NULL; 
            ianalog=NULL;

            //open input ports
            armPort.open("/show-fingers/"+arm+"_arm:i");
            headPort.open("/show-fingers/head:i");
            torsoPort.open("/show-fingers/torso:i");            
        }

        yInfo()<<"Using arm="<<arm<<" and eye="<<eye;

        imgInPort.open("/show-fingers/img:i");
        imgOutPort.open("/show-fingers/img:o");

        if(withOffsets)
        {
            imgOutPortCorrect.open("/show-fingers/imgCorrect:o");
            offsetsPort.open("/show-fingers/offsets:i");        
        }
        camSel=(eye=="left")?0:1;
        eyeKin=new iCubEye(eye);//+"_v2"); // Hard Coded for head v2. //TODO change 
        armKin= new iCubArm(arm);
        armKin->setAllConstraints(false);
        eyeKin->setAllConstraints(false);
        intrinsics.resize(3,4);
        intrinsics.eye();

        // HARDED CODED INTRINSICS For iCub Lisboa01
        double fx,fy, cx, cy;
        if(eye=="left"){
            fx = 215.444;
            fy = 215.737;
            cx = 168.92;
            cy = 129.72;
        }
        else
        {
            fx = 219.88;
            fy = 220.363;
            cx = 167.297;
            cy = 125.909;
        }

        intrinsics(0,0)=fx; intrinsics(1,1)=fy;
        intrinsics(0,2)=cx; intrinsics(1,2)=cy;
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
        Matrix Ha;

        Vector xa_corr,oa_corr;
        Matrix Ha_corr;

        if (!withOffsets && !offline) // Default behaviour
        {
            iarm->getPose(xa,oa);
            Ha=axis2dcm(oa);
        }
        else
        {
            // read encoders from port
            //TODO Changing blocking read
            Bottle *armtemp   = armPort.read();
            Bottle *headtemp  = headPort.read();
            Bottle *torsotemp = torsoPort.read();
        
            
            // populate vectors
            encArm = readEncoders(armtemp);
            encHead = readEncoders(headtemp);
            encTorso = readEncoders(torsotemp);
            yInfo() << encArm.toString().c_str();
            yInfo() << encTorso.toString().c_str();
            yInfo() << encHead.toString().c_str();

            //IMPORTANT!!! Change this ASAP //TODO
            armKin->releaseLink(0);
            armKin->releaseLink(1);
            armKin->releaseLink(2);
            armKin->blockLink(0,CTRL_DEG2RAD*encTorso[2]);
            armKin->blockLink(1,CTRL_DEG2RAD*encTorso[1]);
            armKin->blockLink(2,CTRL_DEG2RAD*encTorso[0]);

            armKin->setAng(CTRL_DEG2RAD*encArm); // missing offsets
            Ha = armKin->getH();
            xa = armKin->EndEffPosition();  //Not Corrected

            if(withOffsets)
            {
                Bottle *estimatedOffsetsInput = offsetsPort.read(false);
                if(estimatedOffsetsInput!=NULL) {
                    if(estimatedOffsetsInput->size() == 8) { //offsets
                        if(estimatedOffsets.isNull ()) {
                            estimatedOffsets.clear();
                        }
                        estimatedOffsets.copy(*estimatedOffsetsInput);
                    }
                }
                if(!estimatedOffsets.isNull()) {
                    for(int i=0;i < (estimatedOffsets.size() -1); i++) {
                        encArm[i] += estimatedOffsets.get(i).asDouble();
                    }
                }
                armKin->setAng(CTRL_DEG2RAD*encArm); // missing offsets
                Ha_corr = armKin->getH();
                xa_corr = armKin->EndEffPosition();  //Not Corrected
            }
  
            
        }
        xa.push_back(1.0);
        Ha.setCol(3,xa);

        Vector pc,pc_corr;
        if(!offline) {
            igaze->get2DPixel(camSel,xa,pc);
            if(withOffsets)
                pc_corr = igaze->get2DPixel(camSel,xa_corr,pc);
        }
        else {
            pc = projectPointInEye(encTorso, encHead, xa);
            if(withOffsets)
                pc_corr = projectPointInEye(encTorso, encHead, xa_corr);
        }
        cv::Point point_c((int)pc[0],(int)pc[1]);
        cv::circle(img,point_c,4,cv::Scalar(0,255,0),4);

        Vector analogs,encs,joints;
        if(!offline) {
            encs.resize(nEncs);
            if (ianalog!=NULL)
                ianalog->read(analogs);
            iencs->getEncoders(encs.data());
        }
        else{
            encs=encArm;
        }
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
                if (offline)
                    fc = projectPointInEye(encTorso, encHead, Ha*(i<0?finger[fng].getH0().getCol(3):
                                                 finger[fng].getH(i,true).getCol(3)));
                else
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
        delete eyeKin;
        delete armKin;
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
    /************************************************************************/
    yarp::sig::Vector readEncoders(Bottle *received)
    {
        Vector encoders;
        encoders.resize(received->size());
        for (int i=0;i<received->size();i++ )
        {
            encoders[i]=received->get(i).asDouble();
        }
        return encoders;
    }
    yarp::sig::Vector projectPointInEye(Vector torso, Vector head, Vector x)
    {
        // Code from iKinGaze
        eyeKin->releaseLink(0);
        eyeKin->releaseLink(1);
        eyeKin->releaseLink(2);
        Vector q(8);
        Vector px;
        q[0]=torso[2]*CTRL_DEG2RAD;
        q[1]=torso[1]*CTRL_DEG2RAD;
        q[2]=torso[0]*CTRL_DEG2RAD;
        q[3]=head[0]*CTRL_DEG2RAD;
        q[4]=head[1]*CTRL_DEG2RAD;
        q[5]=head[2]*CTRL_DEG2RAD;
        q[6]=head[3]*CTRL_DEG2RAD;
        q[7]=(head[4]+head[5]/(camSel?-2.0:2.0))*CTRL_DEG2RAD;
        
        Vector xo=x;
        // impose homogeneous coordinates
        if (xo.length()<4)
            xo.push_back(1.0);
        else
        {
            xo=xo.subVector(0,3); 
            xo[3]=1.0;
        }

        // find position wrt the camera frame
        Vector xe=SE3inv(eyeKin->getH(q))*xo;

        // find the 2D projection
        px=intrinsics*xe;
        px=px/px[2];
        px.pop_back();

        return px;
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

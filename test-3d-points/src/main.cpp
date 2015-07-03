/* 
 * Copyright (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
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

#include <vector>
#include <string>
#include <fstream>
#include <iomanip>

#include <opencv2/opencv.hpp>

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;


/*******************************************************************************/
class TestModule : public RFModule, public PortReader
{
protected:
    vector<cv::Point> contour;
    string homeContextPath;
    Mutex mutex;    
    bool go;

    BufferedPort<ImageOf<PixelMono> > portDispIn;
    BufferedPort<ImageOf<PixelRgb> > portDispOut;
    BufferedPort<ImageOf<PixelRgb> > portImgIn;
    Port portContour;
    RpcClient portSFM;
    RpcServer portRpc;

    /*******************************************************************************/
    bool read(ConnectionReader &connection)
    {
        Bottle data; data.read(connection);
        if (data.size()>=2)
        {
            LockGuard lg(mutex);
            cv::Point point(data.get(0).asInt(),data.get(1).asInt());
            contour.push_back(point);
        }

        return true;
    }

public:
    /*******************************************************************************/
    bool configure(ResourceFinder &rf)
    {
        portDispIn.open("/test-3d-points/disp:i");
        portDispOut.open("/test-3d-points/disp:o");
        portImgIn.open("/test-3d-points/img:i");
        portContour.open("/test-3d-points/contour:i");
        portSFM.open("/test-3d-points/SFM:rpc");
        portRpc.open("/test-3d-points/rpc");

        portContour.setReader(*this);
        attach(portRpc);

        homeContextPath=rf.getHomeContextPath().c_str();
        go=false;

        return true;
    }

    /*******************************************************************************/
    bool interruptModule()
    {
        portDispIn.interrupt();
        portDispOut.interrupt();
        portImgIn.interrupt();
        portContour.interrupt();
        portSFM.interrupt();
        portRpc.interrupt();
        return true;
    }

    /*******************************************************************************/
    bool close()
    {
        portDispIn.close();
        portDispOut.close();
        portImgIn.close();
        portContour.close();
        portSFM.close();
        portRpc.close();
        return true;
    }

    /*******************************************************************************/
    double getPeriod()
    {
        return 0.0;
    }

    /*******************************************************************************/
    bool updateModule()
    {
        ImageOf<PixelMono> *imgDispIn=portDispIn.read();
        if (imgDispIn==NULL)
            return false;

        ImageOf<PixelRgb> *imgIn=portImgIn.read();
        if (imgIn==NULL)
            return false;

        LockGuard lg(mutex);
        
        ImageOf<PixelRgb> &imgDispOut=portDispOut.prepare();
        imgDispOut.resize(imgDispIn->width(),imgDispIn->height());

        cv::Mat imgDispInMat=cv::cvarrToMat((IplImage*)imgDispIn->getIplImage());
        cv::Mat imgDispOutMat=cv::cvarrToMat((IplImage*)imgDispOut.getIplImage());
        cv::cvtColor(imgDispInMat,imgDispOutMat,CV_GRAY2RGB);

        if (contour.size()>2)
        {
            vector<vector<cv::Point> > contours;
            contours.push_back(contour);
            cv::drawContours(imgDispOutMat,contours,0,cv::Scalar(255,25,0),2);

            if (go)
            {
                vector<Vector> points;
                for (int x=0; x<imgDispOut.width(); x++)
                {
                    for (int y=0; y<imgDispOut.height(); y++)
                    {
                        if (cv::pointPolygonTest(contour,cv::Point2f((float)x,(float)y),false)>0.0)
                        {
                            Bottle cmd,reply;
                            cmd.addInt(x); cmd.addInt(y);
                            if (portSFM.write(cmd,reply))
                            {
                                Vector point(6,0.0);
                                point[0]=reply.get(0).asDouble();
                                point[1]=reply.get(1).asDouble();
                                point[2]=reply.get(2).asDouble();
                                if (norm(point)>0.0)
                                {
                                    PixelRgb px=imgIn->pixel(x,y);
                                    point[3]=px.r;
                                    point[4]=px.g;
                                    point[5]=px.b;

                                    points.push_back(point);
                                }
                            }
                        }
                    }
                }

                if (points.size()>0)
                {
                    ofstream fout;
                    fout.open((homeContextPath+"/test-3d-points.off").c_str());
                    if (fout.is_open())
                    {
                        fout<<"COFF"<<endl;
                        fout<<points.size()<<" 0 0"<<endl;
                        fout<<endl;
                        for (size_t i=0; i<points.size(); i++)
                        {
                            fout<<points[i].subVector(0,2).toString(1,4).c_str()<<" "<<
                                  points[i].subVector(3,5).toString(0,3).c_str()<<endl;
                        }
                        fout<<endl;
                    }
                    fout.close();
                }

                go=false;
            }
        }

        portDispOut.write();
        return true;
    }

    /*******************************************************************************/
    bool respond(const Bottle &command, Bottle &reply)
    {
        string cmd=command.get(0).asString().c_str();
        int ack=Vocab::encode("ack");
        int nack=Vocab::encode("nack");

        if (cmd=="clear")
        {
            LockGuard lg(mutex);
            contour.clear();
            reply.addVocab(ack);
        }
        else if (cmd=="go")
        {
            LockGuard lg(mutex);
            if ((contour.size()>2) && (portSFM.getOutputCount()>0))
            {
                go=true;
                reply.addVocab(ack);
            }
            else
                reply.addVocab(nack);
        }
        else
            RFModule::respond(command,reply);
        
        return true;
    }
};


/*******************************************************************************/
int main(int argc,char *argv[])
{   
    Network yarp;
    if (!yarp.checkNetwork())
    {
        yError("unable to find YARP server!");
        return -1;
    }

    TestModule mod;
    ResourceFinder rf;
    rf.setDefaultContext("test-3d-points");
    rf.configure(argc,argv);
    return mod.runModule(rf);
}


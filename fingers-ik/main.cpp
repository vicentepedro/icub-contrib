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

#include <string>
#include <deque>

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>

#include <iCub/ctrl/Math.h>
#include <iCub/iKin/iKinFwd.h>

#include <IpTNLP.hpp>
#include <IpIpoptApplication.hpp>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;
using namespace iCub::iKin;


/****************************************************************/
class HandNLP : public Ipopt::TNLP
{
protected:
    deque<iCubFinger*> fingers;
    deque<iKinChain*> chains;

    deque<Vector> targets;
    deque<Vector> pee;
    deque<Vector> dist;
    deque<Matrix> J;
    Vector joints;

    /************************************************************************/
    virtual void fkin(const Ipopt::Number *x, const bool new_x)
    {        
        if (new_x)
        {
            for (size_t i=0; i<joints.length(); i++)
                joints[i]=x[i];

            // thumb
            (*chains[0])(0).setAng(x[0]);
            (*chains[0])(1).setAng(x[1]);
            (*chains[0])(2).setAng(x[2]/2.0);
            (*chains[0])(3).setAng(x[2]/2.0);

            // index
            (*chains[1])(0).setAng(x[3]/3.0);
            (*chains[1])(1).setAng(x[4]);
            (*chains[1])(2).setAng(x[5]/2.0);
            (*chains[1])(3).setAng(x[5]/2.0);

            // middle
            (*chains[2])(0).setAng(x[6]);
            (*chains[2])(1).setAng(x[7]/2.0);
            (*chains[2])(2).setAng(x[7]/2.0);

            for (size_t i=0; i<chains.size(); i++)
            {
                pee[i]=chains[i]->EndEffPosition();
                dist[i]=targets[i]-pee[i];
                J[i]=chains[i]->GeoJacobian().submatrix(0,2,0,chains[i]->getDOF()-1);
            }
        }
    }

    /****************************************************************/
    bool get_nlp_info(Ipopt::Index &n, Ipopt::Index &m, Ipopt::Index &nnz_jac_g,
                      Ipopt::Index &nnz_h_lag, IndexStyleEnum &index_style)
    {
        n=joints.length();
        m=nnz_jac_g=nnz_h_lag=0;
        index_style=TNLP::C_STYLE;
        return true;
    }

    /****************************************************************/
    bool get_bounds_info(Ipopt::Index n, Ipopt::Number *x_l, Ipopt::Number *x_u,
                         Ipopt::Index m, Ipopt::Number *g_l, Ipopt::Number *g_u)
    {
        // thumb
        x_l[0]=(*chains[0])(0).getMin(); x_u[0]=(*chains[0])(0).getMax();
        x_l[1]=(*chains[0])(1).getMin(); x_u[1]=(*chains[0])(1).getMax();
        x_l[2]=(*chains[0])(2).getMin(); x_u[2]=(*chains[0])(2).getMax()*2.0;

        // index
        x_l[3]=(*chains[1])(0).getMin(); x_u[3]=(*chains[1])(0).getMax()*3.0;
        x_l[4]=(*chains[1])(1).getMin(); x_u[4]=(*chains[1])(1).getMax();
        x_l[5]=(*chains[1])(2).getMin(); x_u[5]=(*chains[1])(2).getMax()*2.0;

        // middle
        x_l[6]=(*chains[2])(0).getMin(); x_u[6]=(*chains[2])(0).getMax();
        x_l[7]=(*chains[2])(1).getMin(); x_u[7]=(*chains[2])(1).getMax()*2.0;

        return true;
    }

    /****************************************************************/
    bool get_starting_point(Ipopt::Index n, bool init_x, Ipopt::Number *x,
                            bool init_z, Ipopt::Number *z_L, Ipopt::Number *z_U,
                            Ipopt::Index m, bool init_lambda, Ipopt::Number *lambda)
    {
        if (init_x)
        {
            for (Ipopt::Index i=0; i<n; i++)
                x[i]=joints[i];
        }
        return true;
    }

    /****************************************************************/
    bool eval_f(Ipopt::Index n, const Ipopt::Number *x, bool new_x,
                Ipopt::Number &obj_value)
    {
        fkin(x,new_x);
        obj_value=0.0;
        for (size_t i=0; i<chains.size(); i++)
            obj_value+=norm2(dist[i]);
        return true;
    }

    /****************************************************************/
    bool eval_grad_f(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
                     Ipopt::Number *grad_f)
    {
        fkin(x,new_x);

        // thumb        
        grad_f[0]=-2.0*dot(dist[0],J[0].getCol(0));
        grad_f[1]=-2.0*dot(dist[0],J[0].getCol(1));
        grad_f[2]=-dot(dist[0],J[0].getCol(2))-dot(dist[0],J[0].getCol(3));

        // index
        grad_f[3]=-2.0*dot(dist[1],J[1].getCol(0))/3.0;
        grad_f[4]=-2.0*dot(dist[1],J[1].getCol(1));
        grad_f[5]=-dot(dist[1],J[1].getCol(2))-dot(dist[1],J[1].getCol(3));

        // middle
        grad_f[6]=-2.0*dot(dist[2],J[2].getCol(0));
        grad_f[7]=-dot(dist[2],J[2].getCol(1))-dot(dist[2],J[2].getCol(2));

        return true;
    }

    /****************************************************************/
    bool eval_g(Ipopt::Index n, const Ipopt::Number *x, bool new_x,
                Ipopt::Index m, Ipopt::Number *g)
    {
        return true;
    }

    /****************************************************************/
    bool eval_jac_g(Ipopt::Index n, const Ipopt::Number *x, bool new_x,
                    Ipopt::Index m, Ipopt::Index nele_jac, Ipopt::Index *iRow,
                    Ipopt::Index *jCol, Ipopt::Number *values)
    {        
        return true;
    }

    /****************************************************************/
    bool eval_h(Ipopt::Index n, const Ipopt::Number *x, bool new_x,
                Ipopt::Number obj_factor, Ipopt::Index m, const Ipopt::Number *lambda,
                bool new_lambda, Ipopt::Index nele_hess, Ipopt::Index *iRow,
                Ipopt::Index *jCol, Ipopt::Number *values)
    {
        return true;
    }

    /****************************************************************/
    void finalize_solution(Ipopt::SolverReturn status, Ipopt::Index n,
                           const Ipopt::Number *x, const Ipopt::Number *z_L,
                           const Ipopt::Number *z_U, Ipopt::Index m,
                           const Ipopt::Number *g, const Ipopt::Number *lambda,
                           Ipopt::Number obj_value, const Ipopt::IpoptData *ip_data,
                           Ipopt::IpoptCalculatedQuantities *ip_cq)
    {
        for (Ipopt::Index i=0; i<n; i++)
            joints[i]=x[i];
    }

public:
    /****************************************************************/
    HandNLP(const string &hand)
    {
        iCubFinger *thumb=new iCubFinger(hand+"_thumb");
        iCubFinger *index=new iCubFinger(hand+"_index");
        iCubFinger *middle=new iCubFinger(hand+"_middle");

        fingers.push_back(thumb);
        fingers.push_back(index);
        fingers.push_back(middle);

        for (size_t i=0; i<fingers.size(); i++)
        {
            chains.push_back(fingers[i]->asChain());
            targets.push_back(Vector(3,0.0));
            pee.push_back(Vector(3,0.0));
            dist.push_back(Vector(3,0.0));
            J.push_back(Matrix(3,fingers[i]->getDOF()));
        }

        // thumb(3)+index(3)+middle(2)
        joints.resize(3+3+2,0.0);
    }

    /****************************************************************/
    virtual ~HandNLP()
    {
        for (size_t i=0; i<fingers.size(); i++)
            delete fingers[i];
    }

    /****************************************************************/
    virtual bool setInitialJoints(const Vector &joints)
    {
        if (joints.length()==this->joints.length())
        {
            this->joints=CTRL_DEG2RAD*joints;
            return true;
        }
        else
        {
            yError()<<"Wrongly sized joints";
            return false;
        }
    }

    /****************************************************************/
    virtual bool setTargets(const deque<Vector> &targets)
    {
        if (targets.size()==chains.size())
        {
            this->targets=targets;
            return true;
        }
        else
        {
            yError()<<"Wrongly sized targets";
            return false;
        }
    }

    /****************************************************************/
    virtual Vector getResult() const
    {
        return CTRL_RAD2DEG*joints;
    }

    /************************************************************************/
    virtual deque<Vector> fkin(const Vector &q)    
    {
        fkin((Ipopt::Number*)((CTRL_DEG2RAD*q).data()),true);
        return pee;
    }
};


/****************************************************************/
int main()
{
    Ipopt::SmartPtr<Ipopt::IpoptApplication> app=new Ipopt::IpoptApplication;
    app->Options()->SetNumericValue("tol",1e-6);
    app->Options()->SetIntegerValue("acceptable_iter",0);
    app->Options()->SetStringValue("mu_strategy","adaptive");
    app->Options()->SetIntegerValue("max_iter",1000);
    app->Options()->SetNumericValue("max_cpu_time",1.0);
    app->Options()->SetStringValue("nlp_scaling_method","gradient-based");
    app->Options()->SetStringValue("hessian_approximation","limited-memory");
    app->Options()->SetIntegerValue("print_level",0);
    app->Initialize();

    Ipopt::SmartPtr<HandNLP> nlp=new HandNLP("right");
    
    Vector q(3+3+2,0.0);
    q[0]=10.0; q[1]=30.0; q[2]=40.0;
    q[3]=30.0; q[4]=20.0; q[5]=30.0;
    q[6]=10.0; q[7]=50.0;
    yInfo()<<"qd = ("<<q.toString(3,3)<<") [deg]";
    deque<Vector> targets=nlp->fkin(q);
    nlp->setTargets(targets);
    for (size_t i=0; i<targets.size(); i++)
        yInfo()<<"targets["<<i<<"] = ("<<targets[i].toString(3,3)<<") [m]";

    q=0.0;
    yInfo()<<"q0 = ("<<q.toString(3,3)<<") [deg]";
    nlp->setInitialJoints(q);

    double t0=Time::now();
    Ipopt::ApplicationReturnStatus status=app->OptimizeTNLP(GetRawPtr(nlp));
    double t1=Time::now();

    q=nlp->getResult();
    yInfo()<<"qf = ("<<q.toString(3,3)<<") [deg]";
    yInfo()<<"elapsed time = "<<t1-t0<<" [s]";

    deque<Vector> pee=nlp->fkin(q);
    for (size_t i=0; i<pee.size(); i++)
        yInfo()<<"pee["<<i<<"] = ("<<pee[i].toString(3,3)<<") [m]; |e| = "<<norm(targets[i]-pee[i])<<" [m]";

    return 0;
}



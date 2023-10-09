#include "MyLinkManipulator.h"


Eigen::Matrix4d MyLinkManipulator::rotateZ(double ang) const
{
    Eigen::Matrix4d ret;
    ret << cos(ang), -sin(ang), 0, 0,
           sin(ang), cos(ang), 0, 0,
           0, 0, 1, 0,
           0, 0, 0, 1;
    return ret;
}

Eigen::Matrix4d MyLinkManipulator::translate(double dx, double dy, double dz) const
{
    Eigen::Matrix4d ret;
    ret << 1, 0, 0, dx,
           0, 1, 0, dy,
           0, 0, 1, dz,
           0, 0, 0, 1;
    return ret;
}

MyLinkManipulator::linkerState MyLinkManipulator::FK(vector<double> lengths, vector<double> angles) const
{
    MyLinkManipulator::linkerState newState;
    newState.lengths = lengths;
    newState.angles = angles;
    newState.numLink = lengths.size();
    vector<Eigen::Matrix4d> P;
    P.push_back(Eigen::Matrix4d::Identity()); 
    Eigen::Matrix4d R;
    Eigen::Matrix4d T;
    for(int i = 0; i < newState.numLink; i++)
    {
        R = this->rotateZ(angles[i]);
        T = this->translate(lengths[i], 0, 0);
        P.push_back(P.back()*R*T);
    }
    newState.jointPts = P;
    return newState;
}

MyLinkManipulator::linkerState MyLinkManipulator::IK(vector<double> lengths, Eigen::Vector2d target) const
{
    int max_itr = 1000;
    double err = 1e-12;

    bool solved = false;

    Eigen::Vector2d endToTarget;
    double errEndToTarget;
    Eigen::Vector2d currToEnd;
    double errCurrToEnd;
    Eigen::Vector2d currtoTarget;
    double currToTargetMag;
    double endTargetMag;
    double cosRotAng;
    double sinRotAng;
    double rotAng;

    MyLinkManipulator::linkerState curr;
    MyLinkManipulator::linkerState ret;

    vector<double> Ang;

    for(int i = 0; i < lengths.size(); i++)
    {
        Ang.push_back(0.1);
    }

    Eigen::Vector2d base = this->getBaseLocation();

    while(!solved && max_itr > 0)
    {
        for(int i = lengths.size()-1; i >= 0; i--)
        {
            curr = this->FK(lengths, Ang);
            endToTarget = target - Eigen::Vector2d{curr.jointPts.back().col(3)(0),curr.jointPts.back().col(3)(1)};
            errEndToTarget = sqrt(pow(endToTarget[0],2) + pow(endToTarget[1],2));
            if(errEndToTarget < err)
            {
                solved = true;
            }
            else
            {
                currToEnd = Eigen::Vector2d{curr.jointPts.back().col(3)(0),curr.jointPts.back().col(3)(1)} - 
                            Eigen::Vector2d{curr.jointPts[i].col(3)(0),curr.jointPts[i].col(3)(1)};
                errCurrToEnd = sqrt(pow(currToEnd[0],2) + pow(currToEnd[1],2));
                currtoTarget = target - Eigen::Vector2d{curr.jointPts[i].col(3)(0),curr.jointPts[i].col(3)(1)};
                currToTargetMag = sqrt(pow(currtoTarget[0],2) + pow(currtoTarget[1],2));
                
                endTargetMag = errCurrToEnd*currToTargetMag;

                if(endTargetMag <= 0.0001)
                {
                    cosRotAng = 1.0;
                    sinRotAng = 0.0;
                }
                else
                {
                    // printf("target to End Magnitude: %.2f\n", endTargetMag);
                    // printf("currtoEnd (%.2f, %.2f) currtoTarget (%.2f, %.2f) \n", currToEnd[0], currToEnd[1], currtoTarget[0], currtoTarget[1]);
                    cosRotAng = (currToEnd[0]*currtoTarget[0] + currToEnd[1]*currtoTarget[1])/endTargetMag;
                    sinRotAng = (currToEnd[0]*currtoTarget[1] - currToEnd[1]*currtoTarget[0])/endTargetMag;
                    // printf("cosRotAng: %.2f, sinRotAng: %.2f\n", cosRotAng, sinRotAng);
                }
                
                rotAng = acos(max(-1.0, min(1.0,cosRotAng)));

                if(sinRotAng < 0.0)
                {
                    rotAng = -rotAng;
                }
                // printf("cosRotAng: %.2f, sinRotAng: %.2f, rotAng: %.2f\n", cosRotAng, sinRotAng, rotAng);
                Ang[i] = Ang[i] + rotAng;

                if(Ang[i] >= 2.0*M_PI)
                {
                    Ang[i] = Ang[i] -(2.0*M_PI);
                }
                if(Ang[i] < 0.0)
                {
                    Ang[i] = (2.0*M_PI) + Ang[i];
                }
            }
        }
        ret = curr;
        ret.angles = Ang;
        curr.angles = Ang;
        // if(max_itr%20 == 0) printf("New angles are %.2f, %.2f, and %.2f\n", Ang[0]*180/M_PI, Ang[1]*180/M_PI, Ang[2]*180/M_PI);
        max_itr -= 1; 
        if(solved)
        {

            break;
        }
    }
    return ret;
}

Eigen::Vector2d MyLinkManipulator::getJointLocation(const ManipulatorState& state, uint32_t joint_index) const
{
    Eigen::Vector2d ret;
    vector<double> lens = this->getLinkLengths();
    MyLinkManipulator::linkerState links = this->FK(lens, state);
    Eigen::Vector2d base = this->getBaseLocation();
    ret = base + Eigen::Vector2d{links.jointPts[joint_index].col(3)(0),links.jointPts[joint_index].col(3)(1)};
    // printf("Location of end point on final link: (%.3f, %.3f)\n",links.jointPts.back().col(3)(0),links.jointPts.back().col(3)(1));
    return ret;
}

ManipulatorState MyLinkManipulator::getConfigurationFromIK(const Eigen::Vector2d& end_effector_location) const
{
    ManipulatorState ret;
    vector<double> lens = this->getLinkLengths();
    MyLinkManipulator::linkerState links = this->IK(lens, end_effector_location);
    // string output = "Configuration Angles are: ";
    // char * tmp;
    // for(int i = 0; i < lens.size(); i++)
    // {
    //     sprintf(tmp, " %.2f Degs ", links.angles[i]*180/M_PI);
    //     output += tmp;
    // }
    // printf("%s\n", output.c_str());
    ret = (ManipulatorState)links.angles;
    return ret;
}
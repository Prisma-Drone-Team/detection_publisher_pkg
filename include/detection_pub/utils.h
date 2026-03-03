#ifndef _utilities_hpp
#define _utilities_hpp

#include <Eigen/Dense>


inline Eigen::Vector4d r2quat(Eigen::Matrix3d R_iniz )
{
    Eigen::Vector4d epsilon;
    int iu, iv, iw;

    if ( (R_iniz(0,0) >= R_iniz(1,1)) && (R_iniz(0,0) >= R_iniz(2,2)) )
    {
        iu = 0; iv = 1; iw = 2;
    }
    else if ( (R_iniz(1,1) >= R_iniz(0,0)) && (R_iniz(1,1) >= R_iniz(2,2)) )
    {
        iu = 1; iv = 2; iw = 0;
    }
    else
    {
        iu = 2; iv = 0; iw = 1;
    }

    double r = sqrt(1 + R_iniz(iu,iu) - R_iniz(iv,iv) - R_iniz(iw,iw));
    Eigen::Vector3d q;
    q <<  0,0,0;
    if (r>0)
    {
        double rr = 2*r;
        double eta = (R_iniz(iw,iv)-R_iniz(iv,iw)/rr);
        epsilon[iu] = r/2;
        epsilon[iv] = (R_iniz(iu,iv)+R_iniz(iv,iu))/rr;
        epsilon[iw] = (R_iniz(iw,iu)+R_iniz(iu,iw))/rr;
        epsilon[3] = eta;
    }
    else
    {
        //eta = 1;
        epsilon << 0,0,0,1;
    }
    return epsilon;
}


inline Eigen::Vector3d R2XYZ(Eigen::Matrix3d R) {
		double phi=0.0, theta=0.0, psi=0.0;
		Eigen::Vector3d XYZ = Eigen::Vector3d::Zero();
		
		theta = asin(R(0,2));
		
		if(fabsf(cos(theta))>pow(10.0,-10.0))
		{
			phi=atan2(-R(1,2)/cos(theta), R(2,2)/cos(theta));
			psi=atan2(-R(0,1)/cos(theta), R(0,0)/cos(theta));
		}
		else
		{
			if(fabsf(theta-M_PI/2.0)<pow(10.0,-5.0))
			{
				psi = 0.0;
				phi = atan2(R(1,0), R(2,0));
				theta = M_PI/2.0;
			}
			else
			{
				psi = 0.0;
				phi = atan2(-R(1,0), R(2,0));
				theta = -M_PI/2.0;
			}
		}
		
		XYZ << phi,theta,psi;
		return XYZ;
        
}

inline Eigen::Matrix3d QuatToMat(Eigen::Vector4d Quat){
    
    Eigen::Matrix3d Rot;
    float s = Quat[0];
    float x = Quat[1];
    float y = Quat[2];
    float z = Quat[3];
    Rot << 1-2*(y*y+z*z),2*(x*y-s*z),2*(x*z+s*y),
    2*(x*y+s*z),1-2*(x*x+z*z),2*(y*z-s*x),
    2*(x*z-s*y),2*(y*z+s*x),1-2*(x*x+y*y);
    return Rot;
    
}


inline Eigen::Matrix3d XYZ2R(Eigen::Vector3d angles) {
    
    Eigen::Matrix3d R = Eigen::Matrix3d::Zero(); 
    Eigen::Matrix3d R1 = Eigen::Matrix3d::Zero(); 
    Eigen::Matrix3d R2 = Eigen::Matrix3d::Zero(); 
    Eigen::Matrix3d R3 = Eigen::Matrix3d::Zero();

    float cos_phi = cos(angles[0]);
    float sin_phi = sin(angles[0]);
    float cos_theta = cos(angles[1]);
    float sin_theta = sin(angles[1]);
    float cos_psi = cos(angles[2]);
    float sin_psi = sin(angles[2]);

    R1  << 1, 0      , 0, 
                0, cos_phi, -sin_phi, 
                0, sin_phi, cos_phi;

    R2  << cos_theta , 0, sin_theta, 
                0        , 1, 0       , 
                -sin_theta, 0, cos_theta;

    R3  << cos_psi, -sin_psi, 0, 
                sin_psi, cos_psi , 0,
                0      , 0       , 1;

    R = R1*R2*R3;

    return R;
}


#endif
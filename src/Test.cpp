#include "Test.h"

using namespace Eigen;
using namespace cnoid;



const double pi = 3.14159265358979;
const double tr = pi / 180;  //deg��rad

RobotLink::RobotLink()//�R���X�g���N�^ 
{
    ID=0;       
    parentID=0;  
    p = {0.0, 0.0, 0.0}; 
    R<<1.0,0.0,0.0,
        0.0,1.0,0.0,
        0.0,0.0,1.0;    
    a = {0.0, 0.0, 0.0};   
    b = {0.0, 0.0, 0.0};     
    q=0.0;
    qref=0.0;  
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////

void InitClass::LinkInit(RobotLink* link, int linknum)
{

    for (int i = 0; i < linknum; i++) {
        link[i].ID = i;  //�z��̃C���f�b�N�X�ɍ��킹��

    }
    //�������{�b�g�̃��f����ύX����Ȃ炱���Ŋ֐ߎ��x�N�g���A���Έʒu�x�N�g����ύX����΂悢

    link[0].a = {0.0, 0.0, 0.0};  //���[�g�����N
    link[0].b = {0.0, 0.0, 0.0};

    //�E��
    link[1].parentID = 0;
    link[1].a = {0.0, 0.0, 1.0};
    link[1].b = {0.0, -0.1, -0.175};

    link[2].parentID = 1;
    link[2].a = {1.0, 0.0, 0.0};
    link[2].b = {0.0, 0.0, 0.0};

    link[3].parentID = 2;
    link[3].a = {0.0, 1.0, 0.0};
    link[3].b = {0.0, 0.0, 0.0};

    link[4].parentID = 3;
    link[4].a = {0.0, 1.0, 0.0};
    link[4].b = {0.0, 0.0, -0.4};

    link[5].parentID = 4;
    link[5].a = {0.0, 1.0, 0.0};
    link[5].b = {0.0, 0.0, -0.4};

    link[6].parentID = 5;
    link[6].a = {1.0, 0.0, 0.0};
    link[6].b = {0.0, 0.0, 0.0};

    //����
    link[7].parentID = 0;
    link[7].a = {0.0, 0.0, 1.0};
    link[7].b = {0.0, 0.1, -0.175};

    link[8].parentID = 7;
    link[8].a = {1.0, 0.0, 0.0};
    link[8].b = {0.0, 0.0, 0.0};

    link[9].parentID = 8;
    link[9].a = {0.0, 1.0, 0.0};
    link[9].b = {0.0, 0.0, 0.0};

    link[10].parentID = 9;
    link[10].a = {0.0, 1.0, 0.0};
    link[10].b = {0.0, 0.0, -0.4};

    link[11].parentID = 10;
    link[11].a = {0.0, 1.0, 0.0};
    link[11].b = {0.0, 0.0, -0.4};

    link[12].parentID = 11;
    link[12].a = {1.0, 0.0, 0.0};
    link[12].b = {0.0, 0.0, 0.0};
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////

Timer::Timer()
{
    count = 0;
    dt = 0.0;
    t = 0.0;
}

void Timer::Countup()
{
    count++;
    t += dt;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////

void IOClass::IOinit(int jointnum) 
{
    for (int i=0;i<jointnum;i++) 
    {
        errorold[i] = 0.0;
    }

}

void IOClass::Actuate(RobotLink* link, Body* ioBody,double dt)
{
    for (int i = 0; i < ioBody->numJoints(); i++) 
    {
        Link* joint;
        joint = ioBody->joint(i);

        double q = joint->q();            //���݂̊֐ߕψʂ��擾
        double error = link[i+1].qref- q;  //�΍�
        double derror = (error - errorold[i]) / dt;//�΍��̎��ԕω���
        double u;

        u = Kp * error + Kd * derror;//PD���䑥

        joint->u() = limit(u);//�����N�Ƀg���N�����

        errorold[i] = error;
    }
}

double IOClass::limit(double u) 
{
    if (u_limit>=fabs(u))//�g���N�����Ȃ�
    {
        return u;    
    } 
    else //�g���N��������
    {
        if (u>0)//��
        {
            return u_limit;
        } 
        else//��
        {
            return -u_limit;
        }
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////

void Kinematics::ForwardKinematics(RobotLink* link, int tofrom, int start)
{
    //i=0�̓��[�g�����N
    for (int i = start; i < start+tofrom; i++) 
    {
        RobotLink Link;
        Link = link[i];

        link[i].p = link[link[i].parentID].R * link[i].b + link[link[i].parentID].p;

        link[i].R = link[link[i].parentID].R * Rodrigues(Link);
    }
}

void Kinematics::InverseKinematics(RobotLink* link, Vector3d pref, Matrix3d Rref, int tofrom, int start)
{
    
    //Step2:���[�g�����N����ڕW�����N�܂ł̊e�֐ߊp�x����ׂ��x�N�g�����`
    Vector3d dp, dw;
    Matrix<double, TOFROM,1> q, dq;
    Matrix<double, 6,1> dr;
    Matrix3d dR;
    Matrix<double, 6, TOFROM> J;      //���R�r�s��
    Matrix<double, TOFROM, 6> pinvJ;  //�^���t�s��
    Matrix<double, TOFROM, TOFROM> A;
    double e = 1.0e-3;
    double k = 0.1;

    for (int i = 0; i < tofrom; i++)  //���َp�����
    {
        link[i + start].q = 10 * tr;
    }


    for (int i = 0; i < tofrom; i++) 
    {
        q(i) = link[i + start].q;
    }
   
    
    ForwardKinematics(link, tofrom, start);
    

    for (int i = 0; i < 100; i++)  //i=100�ŏI��
    {
        
        //Step4:�덷��p,��R���Z�o
        dp = pref - link[start + (tofrom - 1)].p;
        dR = link[start + (tofrom - 1)].R.transpose() * Rref;
        dw = RotmattoAngvec(dR);

        dr(0) = dp(0);
        dr(1) = dp(1);
        dr(2) = dp(2);
        dr(3) = dw(0);
        dr(4) = dw(1);
        dr(5) = dw(2);

        //Step5:�덷���\���������Ȃ�I��
        if (err(dp, dw) < e) 
        {
            break;
        } 
        
        //Step6:�덷���傫���Ȃ�ʒu�E�p���̌덷������������悤�Ȋ֐ߊp�x�C���ʃ�q���j���[�g�����v�\���@�ɂ��Z�o
        J = Jacobian(link, tofrom, start);  //���R�r�s��̎Z�o
        A = J.transpose() * J;

        pinvJ = A.inverse() * J.transpose();  //���R�r�s��̋^���t�s����v�Z

        dq = pinvJ * dr;

        //Step7:qi+1���Z�o
        q = q + k * dq;

        for (int i = 0; i < tofrom; i++)
        {
            link[i + start].q = q(i);
        }
        //Step3:���^���w�v�Z�ɂ��qi�ɑΉ�����ڕW�����N�ʒu�E�p��pi,Ri�����߂�
        ForwardKinematics(link, tofrom, start);
    }
}


Matrix<double, 6, TOFROM> Kinematics::Jacobian(RobotLink* link, int tofrom,int start)
{
    Matrix<double, 6, TOFROM> J;  
    Vector3d a, jp;

    for (int i = 0; i < tofrom; i++)  
    {
        a = link[start + i].R * link[start + i].a;  //���[���h���W�n�ɂ�����֐ߎ��x�N�g��

        jp = a.cross(link[start + (tofrom - 1)].p - link[start + i].p);

        J(0, i) = jp(0);
        J(1, i) = jp(1);
        J(2, i) = jp(2);
        J(3, i) = a(0);
        J(4, i) = a(1);
        J(5, i) = a(2);
    }

    return J;
}

Matrix3d Kinematics::Rodrigues(RobotLink link)
{
    Matrix3d A, E, R;


    A << 0.0, -link.a(2), link.a(1), link.a(2), 0.0, -link.a(0), -link.a(1),
        link.a(0), 0.0;  //�Ђ��ݑΏ̍s��

    E << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0;  //�P�ʍs��

    R = E + A * sin(link.q) + A * A * (1 - cos(link.q));  //���h���Q�X�̎�

    return R;
}

Vector3d Kinematics::RotmattoAngvec(Matrix3d R)
{
    Matrix3d E;
    Vector3d w, r;
    double theta;

    E << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0;

    if (R == E) 
    {
        w = {0.0, 0.0, 0.0};
    }
    else 
    {
        r = {R(2, 1) - R(1, 2), R(0, 2) - R(2, 0), R(1, 0) - R(0, 1)};

        theta = acos((R(0, 0) + R(1, 1) + R(2, 2) - 1) / 2);

        w = (theta / (2 * sin(theta))) * r;
    }


    return w;  //�e���x�x�N�g��
}

double Kinematics::err(Vector3d p, Vector3d w)
{
    return p.norm() + w.norm();  //�덷
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////

WalkingParameters::WalkingParameters()
{
    Cpel = {0.0, 0.0};
    Cvel = {0.0, 0.0};
    pref = {0.0, 0.0};
    Cpd = {0.0, 0.0};
    Cvd = {0.0, 0.0};
    Cpi = {0.0, 0.0};
    Cvi = {0.0, 0.0};
    Cai = {0.0, 0.0};
    Cpf = {0.0, 0.0};
    Cvf = {0.0, 0.0};
    Caf = {0.0, 0.0};
    p = {0.0, 0.0};

    Tsup = 0.0;
    Tdbl = 0.0;
    S = {0.0, 0.0};
}

WalkingPatternGenerator::WalkingPatternGenerator()
{
    Cpref = {0.0, 0.0};
    Cvref = {0.0, 0.0};
    Caref = {0.0, 0.0};
    prefr = {0.0, 0.0, 0.0};
    prefl = {0.0, 0.0, 0.0};
    Rrefr << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0;
    Rrefl << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0;

    zc = ZC;
    g = 9.81;
    C = 0.0;
    S = 0.0;
    D = 0.0;
    a = 10;
    b = 1;
    Tc = sqrt(zc / g);
    stepcount = 0;
    dt = 0.001;
    t = 0.0;
    tcount = 0;

    tofrom = 6;
    Rstart = 1;
    Lstart = 7;

    Rx = 0.0;
    Ry = 0.0;
    Rz = 0.05;
    w = 0.0;

}


void WalkingPatternGenerator::PatternGeneratorInit()
{
    Cpref = {0.0, 0.0};
    Cvref = {0.0, 0.0};
    Caref = {0.0, 0.0};
    prefr = {0.0, 0.0, 0.0};                               
    prefl = {0.0, 0.0, 0.0};                               
    Rrefr << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0;  
    Rrefl << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0;  

    zc = ZC;  
    g = 9.81;
    C = 0.0;
    S = 0.0;
    D = 0.0;
    a = 10;
    b = 1;
    Tc = sqrt(zc / g);
    stepcount = 0;
    dt = 0.001;
    t = 0.0;
    tcount = 0;

    tofrom = 6;  
    Rstart = 1;  
    Lstart = 7;  

    Rx = 0.0;
    Ry = 0.0;
    Rz = 0.05;

    w = 0.0;
}

void WalkingPatternGenerator::PatternPlanner(WalkingParameters* wp, int numsteps)
{
    for (int n = 1; n < numsteps; n++)  //0���ڂ̃p�����[�^�ݒ�͕s�v
    {
        C = cosh(wp[n].Tsup / Tc);
        S = sinh(wp[n].Tsup / Tc);

        //Step5 �ڕW���n�ʒu��ݒ�
        wp[n].pref(0) = wp[n - 1].pref(0) + wp[n].S(0);
        wp[n].pref(1) = wp[n - 1].pref(1) - wp[n].S(1) * sign(n);

        //Step6 ���s�f�Ђ�ݒ�
        wp[n].Cpel(0) = wp[n + 1].S(0) / 2;
        wp[n].Cpel(1) = sign(n) * wp[n + 1].S(1) / 2;
        wp[n].Cvel(0) = (C + 1) * wp[n].Cpel(0) / (Tc * S);
        wp[n].Cvel(1) = (C - 1) * wp[n].Cpel(1) / (Tc * S);

        //Step7 �ڕW�ŏI�d�S��Ԃ�ݒ�
        wp[n].Cpd = wp[n].pref + wp[n].Cpel;
        wp[n].Cvd = wp[n].Cvel;
    }
}
/*
void WalkingPatternGenerator::PatternGenerator(RobotLink* link,WalkingParameters* wp,int numsteps)
{
    if (tcount == 0)  //t=0�̂Ƃ����s
    {
        Cpref(0) = wp[stepcount].Cpi(0);  //y0
        Cvref(0) = wp[stepcount].Cvi(0);  //y0
    } else                                //t=0�ȊO
    {
        Caref(0) = (1.0 / Tc * Tc)
                   * (Cpref(0) - wp[stepcount].pref(0));  //ddx //�Ƃ肠����pref
        Cvref(0) += Caref(0) * dt;
        Cpref(0) += Cvref(0) * dt;
    }


    if (tcount >= wp[stepcount].Tsup * (1 / dt))  //Step4 t : = t + Tsup n:=n+1
    {

        tcount = 1;  //���Z�b�g
        stepcount++;

        wp[stepcount - 1].Cpf(0) = Cpref(0);              //yf
        wp[stepcount - 1].Cvf(0) = Cvref(0);              //dyf
        wp[stepcount].Cpi(0) = wp[stepcount - 1].Cpf(0);  //yi(n)=yf(n-1)
        wp[stepcount].Cvi(0) = wp[stepcount - 1].Cvf(0);  //yi(n)=yf(n-1)

        //Step8 �C�����n�ʒu���Z�o
        C = cosh(wp[stepcount].Tsup / Tc);
        S = sinh(wp[stepcount].Tsup / Tc);
        D = a * (C - 1) * (C - 1) + b * (S / Tc) * (S / Tc);

        wp[stepcount].p(0) = (-a * (C - 1) / D)
                                 * (wp[stepcount].Cpd(0)
                                    - C * wp[stepcount].Cpi(0)
                                    - Tc * S * wp[stepcount].Cvi(0))
                             - (b * S / (Tc * D))
                                   * (wp[stepcount].Cvd(0)
                                      - S * wp[stepcount].Cpi(0) / Tc
                                      - C * wp[stepcount].Cvi(0));

    } 
    else 
    {
        tcount++;  //�������o��
    }
}
*/
void WalkingPatternGenerator::PatternGenerator2(RobotLink* link, WalkingParameters* wp,Vector3d Prefr,Vector3d Prefl,int numsteps)
{
    /////////////////////////////////////////////////
    //�V�r�O������
    if (sign(stepcount) == 1)  //�x���r���E��
    {
        w = (2 * pi) / wp[stepcount].Tsup;//�ŗL�p�U����

       //�x���r
       prefr(0) = wp[stepcount].p(0);
       prefr(1) = wp[stepcount].p(1);
       prefr(2) = 0.0;
       
       if (stepcount==0)
       {
           Rx=(wp[stepcount+1].p(0)-Prefl(0))/(2*pi);
           Ry=(wp[stepcount+1].p(1)-Prefl(1))/(2*pi);

          prefl(0) = Prefl(0)+Rx*(w*t-sin(w*t));//�T�C�N���C�h�Ȑ�
          prefl(1) =Prefl(1)+Ry*(w*t-sin(w*t));
          prefl(2) = Rz*(1-cos(w*t));

       } 
       else
       {
           Rx=(wp[stepcount+1].p(0)-wp[stepcount-1].p(0))/(2*pi);
           Ry=(wp[stepcount+1].p(1)-wp[stepcount-1].p(1))/(2*pi);

            
           prefl(0) = wp[stepcount-1].p(0)+Rx*(w*t-sin(w*t));
           prefl(1) = wp[stepcount-1].p(1)+Ry*(w*t-sin(w*t));
           prefl(2) = Rz*(1-cos(w*t));


       }
      

        kinematics.InverseKinematics(link, prefr, Rrefr, tofrom, Rstart);//�x���r
        kinematics.InverseKinematics(link, prefl, Rrefl, tofrom, Lstart);//�V�r

    } 
    else if (sign(stepcount) == -1)  //�x���r������
    {
        if (stepcount == NUMSTEPS-1)
        {
            //�V�r
            prefr(0) = wp[stepcount - 1].p(0);
            prefr(1) = wp[stepcount - 1].p(1);
            prefr(2) = 0.0;

            //�x���r
            prefl(0) = wp[stepcount].p(0);  //5���ڂ̒��n�ʒu�Ɠ����ɂ���
            prefl(1) = wp[stepcount].p(1);
            prefl(2) = 0.0;
        } 
        else 
        {
        
            //�V�r

            Rx = (wp[stepcount + 1].p(0) - wp[stepcount - 1].p(0))/ (2 * pi);
            Ry = (wp[stepcount + 1].p(1) - wp[stepcount - 1].p(1))/ (2 * pi);

            prefr(0) = wp[stepcount - 1].p(0) + Rx * (w * t - sin(w * t));
            prefr(1) = wp[stepcount - 1].p(1) + Ry * (w * t - sin(w * t));
            prefr(2) = Rz * (1 - cos(w * t));

            //�x���r
            prefl(0) = wp[stepcount].p(0);
            prefl(1) = wp[stepcount].p(1);
            prefl(2) = 0.0;
        }
        
      
        kinematics.InverseKinematics(link, prefr, Rrefr, tofrom, Rstart);  //�V�r
        kinematics.InverseKinematics(link, prefl, Rrefl, tofrom, Lstart);//�x���r
  
    }
    /////////////////////////////////////////////////
    //�d�S�O�������@�d�S�O�������ɂ͏C�����n�ʒu���K�v�B�䂦��PatternGenerator3()�����炩���ߎ��s����K�v������
    Cpref = (wp[stepcount].Cpi - wp[stepcount].p) * cosh(t / Tc)
            + Tc * wp[stepcount].Cvi * sinh(t / Tc) + wp[stepcount].p;
    Cvref = (wp[stepcount].Cpi - wp[stepcount].p) * sinh(t / Tc) / Tc
            + wp[stepcount].Cvi * cosh(t / Tc);
    link[0].p(0) = Cpref(0);
    link[0].p(1) = Cpref(1);
    link[0].p(2) = zc + 0.2;

    if (tcount >= wp[stepcount].Tsup * (1 / dt))  //Step4 t : = t + Tsup n:=n+1
    {
        t = 0.001;
        tcount = 1;  //���Z�b�g 0,0.1~0.8 (0.0) 0.1~0,8 0.0���͂��ނ���1�J�E���g�����Ȃ�(��t=0.1�������Ȃ�)���߁A1����n�߂�
        stepcount++;

        wp[stepcount - 1].Cpf = Cpref;              
        wp[stepcount - 1].Cvf = Cvref;              
        wp[stepcount].Cpi = wp[stepcount - 1].Cpf;  //Cp(n)=Cp(n-1)�@n-1���ڂ̍ŏI�d�S�ʒu��n���ڂ̏����d�S�ʒu�Ƃ��Ă���
        wp[stepcount].Cvi = wp[stepcount - 1].Cvf;  //Cv(n)=Cv(n-1)  n-1���ڂ̍ŏI�d�S���x��n���ڂ̏����d�S���x�Ƃ��Ă���

        //Step8 �C�����n�ʒu���Z�o
        C = cosh(wp[stepcount].Tsup / Tc);
        S = sinh(wp[stepcount].Tsup / Tc);
        D = a * (C - 1) * (C - 1) + b * (S / Tc) * (S / Tc);

        wp[stepcount].p(0) = (-a * (C - 1) / D)
                                 * (wp[stepcount].Cpd(0)
                                    - C * wp[stepcount].Cpi(0)
                                    - Tc * S * wp[stepcount].Cvi(0))
                             - (b * S / (Tc * D))
                                   * (wp[stepcount].Cvd(0)
                                      - S * wp[stepcount].Cpi(0) / Tc
                                      - C * wp[stepcount].Cvi(0));
        wp[stepcount].p(1) = (-a * (C - 1) / D)
                                 * (wp[stepcount].Cpd(1)
                                    - C * wp[stepcount].Cpi(1)
                                    - Tc * S * wp[stepcount].Cvi(1))
                             - (b * S / (Tc * D))
                                   * (wp[stepcount].Cvd(1)
                                      - S * wp[stepcount].Cpi(1) / Tc
                                      - C * wp[stepcount].Cvi(1));


    } 
    else
    {
        t += dt;
        tcount++;  //�������o��
    }
}

void WalkingPatternGenerator::PatternGenerator3(RobotLink* link,
                                                WalkingParameters* wp)
{
    Cpref = (wp[stepcount].Cpi - wp[stepcount].p) * cosh(t / Tc)
            + Tc * wp[stepcount].Cvi * sinh(t / Tc) + wp[stepcount].p;
    Cvref = (wp[stepcount].Cpi - wp[stepcount].p) * sinh(t / Tc) / Tc
            + wp[stepcount].Cvi * cosh(t / Tc);
    link[0].p(0) = Cpref(0);
    link[0].p(1) = Cpref(1);
    link[0].p(2) = zc + 0.2;

    if (tcount >= wp[stepcount].Tsup * (1 / dt))  //Step4 t : = t + Tsup n:=n+1
    {
        t = 0.001;
        tcount = 1;  //���Z�b�g 0,0.1~0.8 (0.0) 0.1~0,8 0.0���͂��ނ���1�J�E���g�����Ȃ�(��t=0.1�������Ȃ�)���߁A1����n�߂�
        stepcount++;

        wp[stepcount - 1].Cpf = Cpref;              //xf
        wp[stepcount - 1].Cvf = Cvref;              //dyf
        wp[stepcount].Cpi = wp[stepcount - 1].Cpf;  //yi(n)=yf(n-1)
        wp[stepcount].Cvi = wp[stepcount - 1].Cvf;  //yi(n)=yf(n-1)

        //Step8 �C�����n�ʒu���Z�o
        C = cosh(wp[stepcount].Tsup / Tc);
        S = sinh(wp[stepcount].Tsup / Tc);
        D = a * (C - 1) * (C - 1) + b * (S / Tc) * (S / Tc);

        wp[stepcount].p(0) = (-a * (C - 1) / D)
                                 * (wp[stepcount].Cpd(0)
                                    - C * wp[stepcount].Cpi(0)
                                    - Tc * S * wp[stepcount].Cvi(0))
                             - (b * S / (Tc * D))
                                   * (wp[stepcount].Cvd(0)
                                      - S * wp[stepcount].Cpi(0) / Tc
                                      - C * wp[stepcount].Cvi(0));
        wp[stepcount].p(1) = (-a * (C - 1) / D)
                                 * (wp[stepcount].Cpd(1)
                                    - C * wp[stepcount].Cpi(1)
                                    - Tc * S * wp[stepcount].Cvi(1))
                             - (b * S / (Tc * D))
                                   * (wp[stepcount].Cvd(1)
                                      - S * wp[stepcount].Cpi(1) / Tc
                                      - C * wp[stepcount].Cvi(1));


    } else {
        t += dt;
        tcount++;  //�������o��
    }
}

int WalkingPatternGenerator::sign(int n)
{
    if ((n % 2 == 0)||(n==0)) 
    {
        return 1;//�E���x���Ȃ�1��Ԃ�
    }
    else
    {
        return -1;//�����x���Ȃ�-1��Ԃ�
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////

void Main::Init(SimpleControllerIO* io)
{
    ioBody = io->body();
    timer.dt = io->timeStep();
    //dt = io->timeStep();
    jointnum = ioBody->numJoints();
    linknum = jointnum + 1;

    for (int i = 0; i < jointnum; i++) 
    {
        Link* joint;

        joint = ioBody->joint(i);

        joint->setActuationMode(Link::JointEffort);

        io->enableIO(joint);

    }
    
    init.LinkInit(link, linknum);
    IO.IOinit(jointnum);
   
    //�����p�����w��
    prefr = {0.0, 0.0, 0.0};   
    Rrefr << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0;

    prefl = {0.0, 0.2, 0.0};  
    Rrefl << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0;

    link[0].p = {0.0, 0.1, MAXZ};

    kinematics.InverseKinematics(link,
                                 prefr,
                                 Rrefr,
                                 tofrom,
                                 Rstart);  
    kinematics.InverseKinematics(link,
                                 prefl,
                                 Rrefl,
                                 tofrom,
                                 Lstart);  //�Ƃ肠�����E������

    for (int i = 0; i < tofrom; i++)  //�t�^���w�v�Z�ŎZ�o�����ڕW�֐ߊp�x�����
    {
        link[Rstart + i].qref = link[Rstart + i].q;
        link[Lstart + i].qref = link[Lstart + i].q;
    }

    
        /////////////////////////////////////////////////
    //���s�p�����[�^�ݒ�

    wp[0].Tsup = 0.5;//���s����
    wp[0].Tdbl = 0.24;//�����x����(����͂����ĂȂ�)
    wp[0].pref = {0.0, 0.0};  //�������n�ʒu
    wp[0].p = wp[0].pref;
    wp[0].Cpi = {0.0, 0.02};  //�����d�S�ʒu
    wp[0].Cvi = {0.0, 0.0};   //�����d�S���x


    wp[1].Tsup = 0.5;
    wp[1].Tdbl = 0.24;
    wp[1].S = {0.0, 0.2};//�O��A���E�����̕���

    wp[2].Tsup = 0.5;
    wp[2].Tdbl = 0.24;
    wp[2].S = {SX-0.1, 0.2};

    wp[3].Tsup = 0.5;
    wp[3].Tdbl = 0.24;
    wp[3].S = {SX, 0.2};

    wp[4].Tsup = 0.5;
    wp[4].Tdbl = 0.24;
    wp[4].S = {SX, 0.2};

    wp[5].Tsup = 0.5;
    wp[5].Tdbl = 0.24;
    wp[5].S = {SX, 0.2};

    wp[6].Tsup = 0.5;
    wp[6].Tdbl = 0.24;
    wp[6].S = {SX, 0.2};

    wp[7].Tsup = 0.5;
    wp[7].Tdbl = 0.24;
    wp[7].S = {SX, 0.2};

    wp[8].Tsup = 0.5;
    wp[8].Tdbl = 0.24;
    wp[8].S = {SX-0.1, 0.2};

    wp[9].Tsup = 0.5;
    wp[9].Tdbl = 0.24;
    wp[9].S = {0.0, 0.2};

    wp[10].Tsup = 0.5;
    wp[10].Tdbl = 0.24;
    wp[10].S = {0.0, 0.0};



    walkingpatterngenerator.PatternPlanner(wp, numsteps);//���n�_�A���s�f�Ђ��Z�o
    /////////////////////////////////////////////////

    for (int i=0;i<5000;i++) 
    {
        walkingpatterngenerator.PatternGenerator3(link, wp);//�C�����n�ʒu���Z�o
    }

    walkingpatterngenerator.PatternGeneratorInit();

}

void Main::Control()
{


    if (timer.count < WAITCOUNT)  //2.0s�Ԃ��̏�őҋ@
    {

        if (timer.count<2000) 
        {
            link[0].p = {0.0,
                         0.1
                             - ((0.08 / (2 * pi))
                                * ((2 * pi / WAITTIME) * timer.t
                                   + sin((2 * pi / WAITTIME) * timer.t))),
                         ((ZC + 0.2 - MAXZ) / 2.0) * timer.t + MAXZ};  //

        }
        else
        {
            link[0].p = {0.0,
                         0.1
                             - ((0.08 / (2 * pi))
                                * ((2 * pi / WAITTIME) * timer.t
                                   + sin((2 * pi / WAITTIME) * timer.t))),
                         ZC + 0.2};  //�����ύX-0.05*timer.t+0.9

        }


        kinematics.InverseKinematics(link,
                                     prefr,
                                     Rrefr,
                                     tofrom,
                                     Rstart);  
        kinematics.InverseKinematics(link,
                                     prefl,
                                     Rrefl,
                                     tofrom,
                                     Lstart);  

        for (int i = 0; i < tofrom; i++)  
        {
            link[Rstart + i].qref = link[Rstart + i].q;
            link[Lstart + i].qref = link[Lstart + i].q;
        }


    } 
    else if ((timer.count >= WAITCOUNT)
               && (timer.count <= WAITCOUNT + WALKCOUNT))  //5.0�ԕ��s
    {
    
        if ((timer.count % 1 == 0) || (timer.count == 0)) {//timer.count%n==0 n��ύX�ŕ��s�p�^�[������������ύX
            walkingpatterngenerator.PatternGenerator2(link,
                                                      wp,
                                                      prefr,
                                                      prefl,
                                                      numsteps);//���s�p�^�[������

            for (int i = 0; i < tofrom; i++) 
            {
                link[Rstart + i].qref = link[Rstart + i].q;
                link[Lstart + i].qref = link[Lstart + i].q;
            }
        }
    }


 
       
        
           
   

    
   
    IO.Actuate(link,ioBody,timer.dt);//PD����

    timer.Countup();//�����o��

}
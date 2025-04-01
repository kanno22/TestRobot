#ifndef Test
#define Test

#define TOFROM 6
#define NUMSTEPS 10//�����ύX
#define ZC 0.55//0.75~0.6 �Ƃ肠����
#define MAXZ 0.97
#define WAITCOUNT 4000
#define WAITTIME 4.0
#define WALKCOUNT 5000
#define SX 0.3//0.11~0.5 �Ƃ肠����

#include<cnoid/SimpleController>
#include<vector>
#include <cnoid/EigenTypes>

using namespace Eigen;
using namespace cnoid;

class RobotLink
{
    public:
        RobotLink();
        int ID;        //������ID
        int parentID;  //�e�����N��ID
        Vector3d p;     //��Έʒu
        Matrix3d R;     //��Ύp��
        Vector3d a;     //�֐ߎ��x�N�g��
        Vector3d b;     //���Έʒu�x�N�g��
        double q;      //�֐ߕψ�[rad]
        double qref;   //�ڕW�֐ߕψ�[rad]
};

class Timer//�^�C�}�[
{
    public:
        int count;
        double dt;
        double t;

        Timer();
        void Countup();

};

class InitClass
{
public:
    void LinkInit(RobotLink* link, int linknum);  //�����N�p�����[�^�̏�����
};


class IOClass
{
    private:
        const double Kp = 5000.0;  //���Q�C��
        const double Kd = 10.0;    //�����Q�C��
        const double u_limit = 200.0;  //100.0;//�ő�g���N
        double errorold[12];//1���[�v�O�̕΍�


    public:
        void Actuate(RobotLink* link, Body* ioBody,double dt);//PD���䕔
        void IOinit(int jointnum);//�����o�ϐ��̏�����
        double limit(double u);//�g���N�����֐�
};

class Kinematics
{
    public:
        void ForwardKinematics(RobotLink* link, int tofrom, int start);  //���^���w�v�Z
        void InverseKinematics(RobotLink* link, Vector3d pref, Matrix3d Rref, int tofrom, int start);//�t�^���w�v�Z
        Matrix3d Rodrigues(RobotLink link);  //���h���Q�X�̎��v�Z
        Matrix<double, 6, TOFROM> Jacobian(RobotLink* link, int tofrom, int start);  //�w�肳�ꂽ�����N�܂ł̃��R�r�s����v�Z
        Vector3d RotmattoAngvec(Matrix3d R);  //��]�s�񁨊p���x�x�N�g��
        double err(Vector3d p, Vector3d w);   //�덷�v�Z
};
/////////////////////////////////////////////////////////////////////////////////////////////////////////

class WalkingParameters
{
public:
    Vector2d Cpel;  //���s�f�Ђɂ�����ŏI�d�S�ʒu
    Vector2d Cvel;  //���s�f�Ђɂ�����ŏI�d�S���x
    Vector2d pref;  //�ڕW���n�ʒu
    Vector2d Cpd;   //�ڕW�ŏI�d�S�ʒu
    Vector2d Cvd;   //�ڕW�ŏI�d�S���x

    Vector2d Cpi;  //�����d�S�ʒu
    Vector2d Cvi;  //�����d�S���x
    Vector2d Cai;  //�����d�S�����x
    Vector2d Cpf;  //�ŏI�d�S�ʒu
    Vector2d Cvf;  //�ŏI�d�S���x
    Vector2d Caf;  //�ŏI�d�S�����x
    Vector2d p;    //���n�ʒu

    double Tsup;  //���s����
    double Tdbl;//�����x����
    Vector2d S;   //���s�p�����[�^

    WalkingParameters();  //�R���X�g���N�^
};

class WalkingPatternGenerator
{
private:
    Kinematics kinematics;

    Vector2d Cpref;  //�d�S�ʒu
    Vector2d Cvref;  //�d�S���x
    Vector2d Caref;  //�d�S�����x
    Vector3d prefr;  //�ڕW�E�������N�ʒu//add
    Vector3d prefl;  //�ڕW���������N�ʒu//add
    Matrix3d Rrefr;  //�ڕW�E�������N�p��//add
    Matrix3d Rrefl;  //�ڕW���������N�p��//add

    double zc;       //�d�S����
    double g;        //�d�͉����x
    double C;       //cosh(Tsup/Tc)
    double S;       //sinh(Tsup/Tc)
    double D;
    double a;
    double b;
    double Tc;
    int stepcount;
    double dt;
    double t;
    int tcount;

    int tofrom;
    int Rstart;
    int Lstart;

    double Rx, Ry, Rz;//�T�C�N���C�h�̔��a
    double w;//�T�C�N���C�h�Ȑ��̊p���x(�ŗL�p�U�������Ă������ق�����������)


public:
    void PatternPlanner(WalkingParameters* wp,
                        int numsteps);  //���s�p�����[�^�𐶐�
    /* void PatternGenerator(RobotLink* link,
                          WalkingParameters* wp,
                          int numsteps);  //���l�ϕ��ɂ����s�p�^�[���𐶐��i�������j*/
    void PatternGenerator2(RobotLink* link,
                           WalkingParameters* wp,Vector3d Prefr,Vector3d Prefl,
                           int numsteps);  //��͉���p���ĕ��s�p�^�[���𐶐�
    void PatternGenerator3(RobotLink* link, WalkingParameters* wp);  //���O�ɏC�����n�ʒu���v�Z
    WalkingPatternGenerator();  //�R���X�g���N�^
    void PatternGeneratorInit();//�p�����[�^��������
    int sign(int n);
};


/////////////////////////////////////////////////////////////////////////////////////////////////////////

class Main  //���{�b�g�̏������Ɛ��䃋�[�v�������o�Ƃ���N���X
{
    private:
        Body* ioBody;  //Body�N���X�̃C���X�^���X
        RobotLink link[13]; //�����N�̐ݒ�
        Timer timer;//�^�C�}�[
        InitClass init;//�����N�p�����[�^�̐ݒ�
        IOClass IO;    //���{�b�g�ւ̓��o�̓C���X�^���X
        Kinematics kinematics;//�^���w�v�Z�p�C���X�^���X
        WalkingParameters wp[NUMSTEPS + 1];//���s�p�����[�^
        WalkingPatternGenerator walkingpatterngenerator;//���s�p�^�[���W�F�l���[�^�p�C���X�^���X

        //double dt;
        int jointnum;//�֐ߐ�
        int linknum;//�����N��
        const int tofrom = 6;//�ʒu�E�p�������߂��������N��
        const int Rstart=1;//�^���w�v�Z���s���n�߂��ԏ��߂̃����N(�C���f�b�N�X)
        const int Lstart = 7;
        const int numsteps=10;//����

        //const double qref[12] = {0.0,0.0,10.0,-20.0,10.0,0.0,0.0,0.0,10.0,-20.0,10.0,0.0};  //�W���p��
        Vector3d prefr,prefl;//�Ƃ肠�����ڕW�����N�ʒu�E�p���͂�����
        Matrix3d Rrefr, Rrefl;


    public:
        void Init(SimpleControllerIO* io);  //���{�b�g�̏�����
        void Control();                       //���䃋�[�v
};

#endif
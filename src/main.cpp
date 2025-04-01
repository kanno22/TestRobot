//#include<cnoid/SimpleController>
#include"Test.h"

using namespace Eigen;
using namespace cnoid;

class TestRobotController :public SimpleController
{
	public:
		Main*  main;

	public:
		virtual bool initialize(SimpleControllerIO* io) override
		{
			main=new Main;

			main->Init(io);

			return true;
		}

		virtual bool control() override
        {
			main->Control();

			return true;
		}
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(TestRobotController)
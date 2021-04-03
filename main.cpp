
#include <iostream>
#include<vector>
#include<string>
#include"BTs.h"
using namespace std;

vector<double> path;
double speed(10);

void control() {
	while (true) {
		double curSpeed = 15;
		double steering;
		std::cout << "速度差为:" << curSpeed - speed << std::endl;
		if (path.size() > 0) {
			steering = 15;
			std::cout << "转向设置为：" << steering << std::endl;
		}
		else
			std::cout << "转向设置为：" << 0 << std::endl;
		std::this_thread::sleep_for(std::chrono::milliseconds(50));
		std::cout << "控制结束:" << std::endl;
	}
}

class MyCondition : public BT::ConditionNode {
public:
	MyCondition(std::string name);
	BT::ReturnStatus Tick();
};

MyCondition::MyCondition(std::string name) : ConditionNode::ConditionNode(name) {}

BT::ReturnStatus MyCondition::Tick() {
	return BT::SUCCESS;
}

class MyAction : public BT::ActionNode
{
public:
	MyAction(std::string name);
	BT::ReturnStatus Tick();
	void Halt();
};

MyAction::MyAction(std::string name) : ActionNode::ActionNode(name) {}

BT::ReturnStatus MyAction::Tick()
{
	if(speed<=10)
		std::cout << "The Action is doing some operations" << std::endl;
	path.push_back(10);
	path.push_back(20);
	std::this_thread::sleep_for(std::chrono::seconds(5));
	if (is_halted())
	{
		return BT::HALTED;
	}
	std::cout << "The Action has succeeded" << std::endl;
	return BT::SUCCESS;
}

void MyAction::Halt() {}



int main(int argc, char *argv[])
{
	//std::thread control(control);

	//BT::SequenceNode* root = new BT::SequenceNode("Sequence");
	//MyCondition* my_cond_1 = new MyCondition("Condition");
	//MyAction* my_act_1 = new MyAction("Action");
	//int tick_time_milliseconds = 1000;

	//root->AddChild(my_cond_1);
	//root->AddChild(my_act_1);

	//std::cout << "Start ticking!" << std::endl;

	//while (1)
	//{
	//	std::cout << root->get_name()<<": "<<root->get_status() << std::endl;

	//	root->Tick();

	//	std::this_thread::sleep_for(std::chrono::milliseconds(tick_time_milliseconds));
	//}

	int i = 1235;
	if (i & 1)
		std::cout << "hadf" << std::endl;
	std::cout << pow(2,800) << std::endl;
	return 0;
}

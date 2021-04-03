#pragma once
#include"BTs.h"


void Execute(BT::ControlNode* root, int TickPeriod_milliseconds)
{
	std::cout << "Start ticking!" << std::endl;

	while (true)
	{
		std::cout << "Ticking the root node !" << std::endl;

		root->Tick();

		std::this_thread::sleep_for(std::chrono::milliseconds(TickPeriod_milliseconds));
	}
}


TickEngine::TickEngine(int initial_value)
{
	value_ = initial_value;
}
TickEngine::~TickEngine() {}
void TickEngine::Wait()
{
	// Lock acquire (need a unique lock for the condition variable usage)
	std::unique_lock<std::mutex> UniqueLock(mutex_);

	// If the state is 0 then we have to wait for a signal
	if (value_ == 0)
		condition_variable_.wait(UniqueLock);

	// Once here we decrement the state
	value_--;
}
void TickEngine::Tick()
{
	// Lock acquire

	std::lock_guard<std::mutex> LockGuard(mutex_);

	// State increment
	value_++;

	// Notification
	condition_variable_.notify_all();
}


BT::TreeNode::TreeNode(std::string name) : tick_engine(0)
{
	// Initialization
	name_ = name;
	is_state_updated_ = false;
	set_status(BT::IDLE);
}
BT::TreeNode::~TreeNode() {}
void BT::TreeNode::set_status(ReturnStatus new_status)
{
	if (new_status != BT::IDLE)
	{
		set_color_status(new_status);
	}

	// Lock acquistion
	std::unique_lock<std::mutex> UniqueLock(state_mutex_);

	// state_ update
	status_ = new_status;
}
BT::ReturnStatus BT::TreeNode::get_status()
{
	// Lock acquistion
	//DEBUG_STDOUT(get_name() << " is setting its status to " << status_);

	std::lock_guard<std::mutex> LockGuard(state_mutex_);

	return status_;
}
BT::ReturnStatus BT::TreeNode::get_color_status()
{
	// Lock acquistion
	std::lock_guard<std::mutex> LockGuard(color_state_mutex_);

	return color_status_;
}
void BT::TreeNode::set_color_status(ReturnStatus new_color_status)
{
	// Lock acquistion
	std::lock_guard<std::mutex> LockGuard(color_state_mutex_);
	// state_ update
	color_status_ = new_color_status;
}
float BT::TreeNode::get_x_pose()
{
	return x_pose_;
}
void BT::TreeNode::set_x_pose(float x_pose)
{
	x_pose_ = x_pose;
}
float BT::TreeNode::get_x_shift()
{
	return x_shift_;
}
void BT::TreeNode::set_x_shift(float x_shift)
{
	x_shift_ = x_shift;
}
void BT::TreeNode::set_name(std::string new_name)
{
	name_ = new_name;
}
std::string BT::TreeNode::get_name()
{
	return name_;
}
BT::NodeType BT::TreeNode::get_type()
{
	return type_;
}
bool BT::TreeNode::is_halted()
{
	return get_status() == BT::HALTED;
}


BT::ControlNode::ControlNode(std::string name) : TreeNode::TreeNode(name)
{
	type_ = BT::CONTROL_NODE;

	// TODO(...) In case it is desired to set to idle remove the ReturnStatus
	// type in order to set the member variable
	// ReturnStatus child_i_status_ = BT::IDLE;  // commented out as unused
}
BT::ControlNode::~ControlNode() {}
void BT::ControlNode::AddChild(TreeNode* child)
{
	//    // Checking if the child is not already present
	//    for (unsigned int i=0; i<children_nodes_.size(); i++)
	//    {
	//        if (children_nodes_[i] == child)
	//        {
	//            throw BehaviorTreeException("'" + child->get_name() + "' is already a '" + get_name() + "' child.");
	//        }
	//    }

	children_nodes_.push_back(child);
	children_states_.push_back(BT::IDLE);
}
unsigned int BT::ControlNode::GetChildrenNumber()
{
	return children_nodes_.size();
}
void BT::ControlNode::Halt()
{
	//DEBUG_STDOUT("HALTING: " << get_name());
	HaltChildren(0);
	set_status(BT::HALTED);
}
std::vector<BT::TreeNode*> BT::ControlNode::GetChildren()
{
	return children_nodes_;
}
void BT::ControlNode::ResetColorState()
{
	set_color_status(BT::IDLE);
	for (unsigned int i = 0; i < children_nodes_.size(); i++)
	{
		children_nodes_[i]->ResetColorState();
	}
}
void BT::ControlNode::HaltChildren(int i)
{
	for (unsigned int j = i; j < children_nodes_.size(); j++)
	{
		if (children_nodes_[j]->get_type() == BT::CONDITION_NODE)
		{
			children_nodes_[i]->ResetColorState();
		}
		else
		{
			if (children_nodes_[j]->get_status() == BT::RUNNING)
			{
				//DEBUG_STDOUT("SENDING HALT TO CHILD " << children_nodes_[j]->get_name());
				children_nodes_[j]->Halt();
			}
			else
			{
				//DEBUG_STDOUT("NO NEED TO HALT " << children_nodes_[j]->get_name()
				//	<< "STATUS" << children_nodes_[j]->get_status());
			}
		}
	}
}
int BT::ControlNode::Depth()
{
	int depMax = 0;
	int dep = 0;
	for (unsigned int i = 0; i < children_nodes_.size(); i++)
	{
		dep = (children_nodes_[i]->Depth());
		if (dep > depMax)
		{
			depMax = dep;
		}
	}
	return 1 + depMax;
}


BT::LeafNode::LeafNode(std::string name) : TreeNode(name) {}
BT::LeafNode::~LeafNode() {}
void BT::LeafNode::ResetColorState()
{
	color_status_ = BT::IDLE;
}
int BT::LeafNode::Depth()
{
	return 0;
}


BT::ConditionNode::ConditionNode(std::string name) : LeafNode::LeafNode(name)
{
	type_ = BT::CONDITION_NODE;
}
BT::ConditionNode::~ConditionNode() {}
void BT::ConditionNode::Halt() {}
int BT::ConditionNode::DrawType()
{
	return BT::CONDITION;
}


BT::ActionNode::ActionNode(std::string name) : LeafNode::LeafNode(name)
{
	type_ = BT::ACTION_NODE;
	thread_ = std::thread(&ActionNode::WaitForTick, this);
}
BT::ActionNode::~ActionNode() {}
void BT::ActionNode::WaitForTick()
{

	while (true)
	{
		// Waiting for the tick to come
		//DEBUG_STDOUT(get_name() << " WAIT FOR TICK");

		tick_engine.Wait();
		//DEBUG_STDOUT(get_name() << " TICK RECEIVED");

		// Running state
		set_status(BT::RUNNING);
		BT::ReturnStatus status = Tick();
		set_status(status);
	}
}
int BT::ActionNode::DrawType()
{
	return BT::ACTION;
}


BT::SequenceNode::SequenceNode(std::string name) : ControlNode::ControlNode(name) {}
BT::SequenceNode::~SequenceNode() {}
BT::ReturnStatus BT::SequenceNode::Tick()
{
	// gets the number of children. The number could change if, at runtime, one edits the tree.
	N_of_children_ = children_nodes_.size();

	// Routing the ticks according to the sequence node's logic:

	for (unsigned int i = 0; i < N_of_children_; i++)
	{
		/*      Ticking an action is different from ticking a condition. An action executed some portion of code in another thread.
				We want this thread detached so we can cancel its execution (when the action no longer receive ticks).
				Hence we cannot just call the method Tick() from the action as doing so will block the execution of the tree.
				For this reason if a child of this node is an action, then we send the tick using the tick engine. Otherwise we call the method Tick() and wait for the response.
		*/
		if (children_nodes_[i]->get_type() == BT::ACTION_NODE)
		{
			// 1) If the child i is an action, read its state.
			child_i_status_ = children_nodes_[i]->get_status();

			if (child_i_status_ == BT::IDLE || child_i_status_ == BT::HALTED)
			{
				// 1.1) If the action status is not running, the sequence node sends a tick to it.
				//DEBUG_STDOUT(get_name() << "NEEDS TO TICK " << children_nodes_[i]->get_name());
				children_nodes_[i]->tick_engine.Tick();

				// waits for the tick to arrive to the child
				do
				{
					child_i_status_ = children_nodes_[i]->get_status();
					std::this_thread::sleep_for(std::chrono::milliseconds(10));
				} while (child_i_status_ != BT::RUNNING && child_i_status_ != BT::SUCCESS
					&& child_i_status_ != BT::FAILURE);
			}
		}
		else
		{
			// 2) if it's not an action:
			// Send the tick and wait for the response;
			child_i_status_ = children_nodes_[i]->Tick();
			children_nodes_[i]->set_status(child_i_status_);
		}
		// Ponderate on which status to send to the parent
		if (child_i_status_ != BT::SUCCESS)
		{
			// If the  child status is not success, halt the next children and return the status to your parent.
			if (child_i_status_ == BT::FAILURE)
			{
				children_nodes_[i]->set_status(BT::IDLE);  // the child goes in idle if it has returned failure.
			}

			//DEBUG_STDOUT(get_name() << " is HALTING children from " << (i + 1));
			HaltChildren(i + 1);
			set_status(child_i_status_);
			return child_i_status_;
		}
		else
		{
			// the child returned success.
			children_nodes_[i]->set_status(BT::IDLE);

			if (i == N_of_children_ - 1)
			{
				// If the  child status is success, and it is the last child to be ticked,
				// then the sequence has succeeded.
				set_status(BT::SUCCESS);
				return BT::SUCCESS;
			}
		}
	}
	return BT::EXIT;
}
int BT::SequenceNode::DrawType()
{
	return BT::SEQUENCE;
}



BT::SelectorNode::SelectorNode(std::string name) : ControlNode::ControlNode(name) {}
BT::SelectorNode::~SelectorNode() {}
BT::ReturnStatus BT::SelectorNode::Tick()
{
	{
		// gets the number of children. The number could change if, at runtime, one edits the tree.
		N_of_children_ = children_nodes_.size();

		// Routing the ticks according to the fallback node's logic:

		for (unsigned int i = 0; i < N_of_children_; i++)
		{
			/*      Ticking an action is different from ticking a condition. An action executed some portion of code in another thread.
					We want this thread detached so we can cancel its execution (when the action no longer receive ticks).
					Hence we cannot just call the method Tick() from the action as doing so will block the execution of the tree.
					For this reason if a child of this node is an action, then we send the tick using the tick engine. Otherwise we call the method Tick() and wait for the response.
			*/
			if (children_nodes_[i]->get_type() == BT::ACTION_NODE)
			{
				// 1) If the child i is an action, read its state.
				child_i_status_ = children_nodes_[i]->get_status();

				if (child_i_status_ == BT::IDLE || child_i_status_ == BT::HALTED)
				{
					// 1.1) If the action status is not running, the sequence node sends a tick to it.
					//DEBUG_STDOUT(get_name() << "NEEDS TO TICK " << children_nodes_[i]->get_name());
					children_nodes_[i]->tick_engine.Tick();

					// waits for the tick to arrive to the child
					do
					{
						child_i_status_ = children_nodes_[i]->get_status();
						std::this_thread::sleep_for(std::chrono::milliseconds(10));
					} while (child_i_status_ != BT::RUNNING && child_i_status_ != BT::SUCCESS
						&& child_i_status_ != BT::FAILURE);
				}
			}
			else
			{
				// 2) if it's not an action:
				// Send the tick and wait for the response;
				child_i_status_ = children_nodes_[i]->Tick();
				children_nodes_[i]->set_status(child_i_status_);

			}
			// Ponderate on which status to send to the parent
			if (child_i_status_ != BT::FAILURE)
			{
				if (child_i_status_ == BT::SUCCESS)
				{
					children_nodes_[i]->set_status(BT::IDLE);  // the child goes in idle if it has returned success.
				}
				// If the  child status is not failure, halt the next children and return the status to your parent.
				//DEBUG_STDOUT(get_name() << " is HALTING children from " << (i + 1));
				HaltChildren(i + 1);
				set_status(child_i_status_);
				return child_i_status_;
			}
			else
			{
				// the child returned failure.
				children_nodes_[i]->set_status(BT::IDLE);
				if (i == N_of_children_ - 1)
				{
					// If the  child status is failure, and it is the last child to be ticked,
					// then the sequence has failed.
					set_status(BT::FAILURE);
					return BT::FAILURE;
				}
			}
		}
	}
	return BT::EXIT;
}
int BT::SelectorNode::DrawType()
{
	// Lock acquistion
	return BT::SELECTOR;
}

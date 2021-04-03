#pragma once
#include<string>
#include<iostream>
#include <thread>
#include <chrono>
#include <mutex>
#include <condition_variable>
#include<vector>


class TickEngine
{
private:
	int value_;
	std::mutex mutex_;
	std::condition_variable condition_variable_;
public:
	TickEngine(int initial_value);
	~TickEngine();
	void Wait();
	void Tick();
};


namespace BT
{
	// Enumerates the possible types of a node, for drawinf we have do discriminate whoich control node it is:

	enum NodeType { ACTION_NODE, CONDITION_NODE, CONTROL_NODE };
	enum DrawNodeType { PARALLEL, SELECTOR, SEQUENCE, SEQUENCESTAR, SELECTORSTAR, ACTION, CONDITION, DECORATOR };
	// Enumerates the states every node can be in after execution during a particular
	// time step:
	// - "Success" indicates that the node has completed running during this time step;
	// - "Failure" indicates that the node has determined it will not be able to complete
	//   its task;
	// - "Running" indicates that the node has successfully moved forward during this
	//   time step, but the task is not yet complete;
	// - "Idle" indicates that the node hasn't run yet.
	// - "Halted" indicates that the node has been halted by its father.
	enum ReturnStatus { RUNNING, SUCCESS, FAILURE, IDLE, HALTED, EXIT };

	// Enumerates the options for when a parallel node is considered to have failed:
	// - "FAIL_ON_ONE" indicates that the node will return failure as soon as one of
	//   its children fails;
	// - "FAIL_ON_ALL" indicates that all of the node's children must fail before it
	//   returns failure.
	enum FailurePolicy { FAIL_ON_ONE, FAIL_ON_ALL };
	enum ResetPolity { ON_SUCCESS_OR_FAILURE, ON_SUCCESS, ON_FAILURE };

	// Enumerates the options for when a parallel node is considered to have succeeded:
	// - "SUCCEED_ON_ONE" indicates that the node will return success as soon as one
	//   of its children succeeds;
	// - "BT::SUCCEED_ON_ALL" indicates that all of the node's children must succeed before
	//   it returns success.
	enum SuccessPolicy { SUCCEED_ON_ONE, SUCCEED_ON_ALL };

	// If "BT::FAIL_ON_ONE" and "BT::SUCCEED_ON_ONE" are both active and are both trigerred in the
	// same time step, failure will take precedence.

	// Abstract base class for Behavior Tree Nodes
	class TreeNode
	{
	private:
		// Node name
		std::string name_;

	protected:
		// The node state that must be treated in a thread-safe way
		bool is_state_updated_;
		ReturnStatus status_;
		ReturnStatus color_status_;

		std::mutex state_mutex_;
		std::mutex color_state_mutex_;
		std::condition_variable state_condition_variable_;
		// Node type
		NodeType type_;
		//position and offset for horizontal positioning when drawing
		float x_shift_, x_pose_;

	public:
		// The thread that will execute the node
		std::thread thread_;

		// Node semaphore to simulate the tick
		// (and to synchronize fathers and children)
		TickEngine tick_engine;

		// The constructor and the distructor
		TreeNode(std::string name);
		~TreeNode();

		// The method that is going to be executed when the node receive a tick
		virtual BT::ReturnStatus Tick() = 0;

		// The method used to interrupt the execution of the node
		virtual void Halt() = 0;

		// The method that retrive the state of the node
		// (conditional waiting and mutual access)
	   // ReturnStatus GetNodeState();
		void SetNodeState(ReturnStatus new_state);
		void set_color_status(ReturnStatus new_color_status);

		// Methods used to access the node state without the
		// conditional waiting (only mutual access)
		ReturnStatus ReadState();
		ReturnStatus get_color_status();
		virtual int DrawType() = 0;
		virtual void ResetColorState() = 0;
		virtual int Depth() = 0;
		bool is_halted();

		//Getters and setters
		void set_x_pose(float x_pose);
		float get_x_pose();

		void set_x_shift(float x_shift);
		float get_x_shift();

		ReturnStatus get_status();
		void set_status(ReturnStatus new_status);

		std::string get_name();
		void set_name(std::string new_name);

		NodeType get_type();
	};


	class ControlNode : public TreeNode
	{
	protected:
		// Children vector
		std::vector<TreeNode*> children_nodes_;

		// Children states
		std::vector<ReturnStatus> children_states_;

		// Vector size
		unsigned int N_of_children_;
		//child i status. Used to rout the ticks
		ReturnStatus child_i_status_;

	public:
		// Constructor
		ControlNode(std::string name);
		~ControlNode();

		// The method used to fill the child vector
		void AddChild(TreeNode* child);

		// The method used to know the number of children
		unsigned int GetChildrenNumber();
		std::vector<TreeNode*> GetChildren();
		// The method used to interrupt the execution of the node
		void Halt();
		void ResetColorState();
		void HaltChildren(int i);
		int Depth();

		// Methods used to access the node state without the
		// conditional waiting (only mutual access)
		bool WriteState(ReturnStatus new_state);
	};


	class LeafNode : public TreeNode
	{
	protected:
	public:
		LeafNode(std::string name);
		~LeafNode();
		void ResetColorState();
		int Depth();
	};


	class ConditionNode : public LeafNode
	{
	public:
		// Constructor
		ConditionNode(std::string name);
		~ConditionNode();

		// The method that is going to be executed by the thread
		virtual BT::ReturnStatus Tick() = 0;

		// The method used to interrupt the execution of the node
		void Halt();

		// Methods used to access the node state without the
		// conditional waiting (only mutual access)
		bool WriteState(ReturnStatus new_state);
		int DrawType();
	};


	class ActionNode : public LeafNode
	{
	public:
		// Constructor
		ActionNode(std::string name);
		~ActionNode();

		// The method that is going to be executed by the thread
		void WaitForTick();
		virtual BT::ReturnStatus Tick() = 0;

		// The method used to interrupt the execution of the node
		virtual void Halt() = 0;

		// Methods used to access the node state without the
		// conditional waiting (only mutual access)
		bool WriteState(ReturnStatus new_state);
		int DrawType();
	};


	class SequenceNode : public ControlNode
	{
	public:
		// Constructor
		SequenceNode(std::string name);
		~SequenceNode();
		int DrawType();
		// The method that is going to be executed by the thread
		BT::ReturnStatus Tick();
	};

	class SelectorNode : public ControlNode
	{
	public:
		// Constructor
		SelectorNode(std::string name);
		~SelectorNode();
		int DrawType();
		// The method that is going to be executed by the thread
		BT::ReturnStatus Tick();
	};
};


void Execute(BT::ControlNode* root, int TickPeriod_milliseconds);



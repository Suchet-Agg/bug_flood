#include <ros/ros.h>
#include <bug_flood/bug.h>
#include <bug_flood/bug_flood.h>

using namespace std;

int Bug::IDENTIFIER_COUNTER = 0;

int main(int argCount, char ** argValues)
{
	//check if proper inputs are passed
	if(argCount < 4)// Check the value of passedArgumentCount. if filename is not passed
	{
		std::cout << "usage -> rosrun path_planning nodeName <sourceGoal> <map> <output>\n";
		// Inform the user on how to use the program
		exit(0);
	}
	//path pruning is on by default
	bool isPruneON = true;
	if(argCount >= 5)
	{
		if(!strcmp("false", argValues[4]))
		{
			isPruneON = false;
		}
	}

	//initializing ROS
	ros::init(argCount,argValues,"bug_flood");
	ros::NodeHandle n;


#ifdef PUBLISH_PATH
	ros::Publisher pathPublisher = n.advertise<Marker>("bug_flood_path",1);
	ros::Duration(1).sleep();
	Marker pathTree;
	Marker bugs;
	initMarkers(bugs, pathTree);
#endif

	Environment environment(argValues[1],argValues[2]);

	Bug finalBug;
	finalBug.setCost(INT_MAX); //this bug will be used to store the min path's bug

	vector<Bug> bugList;
	//add the first bug to the list
	bugList.push_back(Bug(environment.getSource()));

	//start timer
	ros::Time startTime = ros::Time::now();

	while (ros::ok())
	{
#ifdef PUBLISH_PATH
		publish_bugs(bugList,pathPublisher, bugs, pathTree);
#endif
		//Kill bugs that are done
		KillBugs(bugList, finalBug);

		//if all the bugs are killed
		if(bugList.empty())
		{
			break;
		}

		//for all bugs in the bugList
		for (Bug &bug: bugList)
		{
			if (bug.getState() == Bug::TOWARDS_GOAL)
			{
				bug.TowardsGoal(environment, bugList);
				continue; //to avoid running a bug twice when changing transition through TOWARDS_GOAL to BOUNDARY_FOLLOW
			}
			if(bug.getState() == Bug::BOUNDARY_FOLLOWING)
			{
				bug.BoundaryFollow(environment);
			}
		}
	}

	//print the algo's running time
	ros::Time endTime = ros::Time::now();
	int nsec = endTime.nsec - startTime.nsec;
	int sec = endTime.sec - startTime.sec;
	if(nsec < 0)
	{
		sec -= 1;
		nsec += 1000000000;
	}
	ROS_INFO("End, Total Time = %d, %d", sec, nsec);

	double mainTime = sec + (nsec / 1000000000.0);
	double pruneTime = 0;

	//do path pruning only if pruning is on
	if(isPruneON)
	{
		startTime = ros::Time::now();
		vector<Point> resultingPath = finalBug.getpath();
		double cost;
		prunePath(resultingPath, environment, cost);
		finalBug.setCost(cost);
		endTime = ros::Time::now();
		nsec = endTime.nsec - startTime.nsec;
		sec = endTime.sec - startTime.sec;
		if (nsec < 0) {
			sec -= 1;
			nsec += 1000000000;
		}
		ROS_INFO("Pruning Time = %d, %d", sec, nsec);
		pruneTime = sec + (nsec / 1000000000.0);
	}
	ROS_INFO("COST %f", finalBug.getCost());

	//if path is to be published
	#ifdef PUBLISH_PATH
		publish_bug(finalBug, pathPublisher, pathTree);
		ros::Duration(1).sleep();
		ros::spinOnce();
	#endif

	//logfile
	ofstream logFile;
	logFile.open("bugLog.txt",ofstream::app);

	logFile << finalBug.getCost() << "," << mainTime << "," << pruneTime << endl;

	logFile.close();

	//dump final path in a fixed log file
	finalBug.DumpPath("/tmp/bug_flood_path.txt");

	return 1;
}
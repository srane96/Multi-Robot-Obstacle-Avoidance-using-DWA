This package serves as a warehouse manager, that assignes tasks to the robots when asked for and generate a report to record the method performance for benchmarking.

Following is the folder structure

``` 

* src
    - warehouse_manager
        - include
            - warehouse_manager
                - environment_master.h
        - msg
            - TaskInfo.msg
        - params
            - location.txt
        - src
            - main.cpp
            - environment_master.cpp
        - srv
            - Robot_Task_Complete.srv
	        - Robot_Task_Request.srv
            -Robot_Gen_Report.srv	
        - CmakeList.txt
        - package.xml
        - ReadMe.md
    - CmakeList.txt

```

Current functionality of `EnvironmentMaster` class:

* Hosts a service called `request_available_task` to assign tasks(goals) to the robot requesting for goals which has (request: <string>name of robot (eg. 0, 1, .., n) response: <float> x , <float> y, <bool> task_available)
* Hosts a service called `report_task_complete` to store the time taken and min_distance to goal by robot which has (request: <string>name of robot (eg. 0, 1, .., n) , <float>time, <float>distance)
* Hosts a service called `gen_final_report` to generate a text file which only has total time taken, total distance travelled and number of collisions (request: <bool> generate_report)
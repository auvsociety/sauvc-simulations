# rov_ctrl

## To execute
```
rosrun v2_control rov_ctrl
```
- Configure parameters for the ROV mode in the `config_ROV.h` file.

## About
- This node is used to control the AUV_v2 vehicle in ROV mode.
- Uses vectored thruster fusion algorithm. More information can be found [here](../../../docs/On-thruster-configuration.md).
- PID controller for heave and yaw have been implemented for this node. The PID gain values can be changed in the `.yaml` file located in `/path/to/v2_control/config`. The PID gain values are loaded into the parameter server and the gain values will change with `rosparam set` command while the `rov_ctrl` node is running.
- **Known issue**: Sometimes after executing the *Start/stop traversing* command in the node, the vehicle snaps back to the origin. If it happens, just re-start the whole simulation, everything should be fine. This bug has been eliminated the issue didn't arise for many test runs as per the author's knowledge; but if it occurs, please inform.
- Video demonstration of this node: [link](https://youtu.be/kQNYnM7btyY)

## Understanding implementation of stoppable and reusable threads
The following snippet forms the core that is responsible for:
- Executing PID controller on a separate thread; continuously without any interrupts
- Stop the background thread
- Re-start the background thread

This snippet has been taken from `rov_ctrl.cpp` file. Rest of the code may differ from application to application but for continuos turning ON and OFF of PID controller, the following snippet is a must.
```
...
...
int main()
{
  ...
  ...
  
  /*this same thread object will be used to create a thread where the callable is the same*/

  std::thread pid_thread;

  // do_not_quit_ is set/reset by user
  while (do_not_quit_ && ros::ok())
  {
    if (v2_.is_traversing_)
    {
	  /* execute if the thread object is not joinable =>
	  	thread previous thread that was created using this
		object is terminated/killed.
	  */
      if(!pid_thread.joinable())
      {
		/* new thread assigned to our old 
		thread object using lambda expression
		*/
        pid_thread = std::thread([&](){
		  
		  /* reuseThread is inherited function 
		  from StoppableThread. This function creates a
		  new future_obj using new exit_signal_obj
		  that helps in creating stoppable  thread.
		  Read StoppableThread class to learn more.
		  */
          v2_.reuseThread();

		  /* run is inherited from UWV and was 
		  virtual in StoppableThread. This 
		  executes the PID loop in the background.
		  */
          v2_.run();
        });
      }
    }
    else
    {
	  /* stopRequested is inherited from StoppableThread.
	  Once the value is True, it remains true until
	  we create a new future_obj using new exit_signal_obj.
	  */
      if(!v2_.stopRequested())
      {
		/* Executing this would send a signal that would
		eventually end the PID thread in the background.
		*/
        v2_.stop();
      }

	  /* Joins the background thread with the
	  current thread so that it can terminate.
	  */
      if(pid_thread.joinable())
      {
        pid_thread.join();
      }
    }
    ros::spinOnce();
  }

  /* Closing the background thread at the end of the 
  application.
  */
  if(!v2_.stopRequested()){
    v2_.stop();
  }
  if(pid_thread.joinable()){
    pid_thread.join();
  }
  ROS_INFO_STREAM("Sayonara.");

  return 0;
}
...
...

```

[Back to inter-package navigation](../../docs/v2_control.md)

[Back to Home](../../docs/Home.md)
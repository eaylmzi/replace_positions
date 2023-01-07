Firstly, I created 3 robots 
I use direction to determine which direction the robot is facing. In the coordinate system, I named 
the area where X and Y are positive 1, where X is negative Y is positive 2, where both are negative 3, 
where X is positive Y is negative 4.
After that, I move my robots a little bit because odometry doesn't send me the robots' locations 
without moving the robots.
Here, i got the coordinates of my robots to use later. 
I have get angle function , simply it includes mathemethical stuff to calculate degree to rotate.
And then I wrote find way function.
My find way function do some process the location of the robot and the location of its destination 
and calculates which coordinate area the robot will be in . Sets the current direction and it return 
which direction the robot will turn. -1 is clockwise and 1 is counter clockwise.
A negative value can come from the getangle function, I convert it to positive And I send this to find 
angle method
The find angel method is similar to the findway method. If the region it will return to is not the 
regions next to the coordinate system, it adds another 90 degrees and reaches the correct result.
And with this information we send the required parameters to the rotate task and it rotetes
Finally, we send the necessary coordinates to our move2goal function with 1.5 tolarence to not 
collide and run the robot and set the direction 1 for the next robot. We apply these same processes 
in the remaining 2 robots.
Since the places we went to 1.5 tolerance could not reach the exact place, I call move2goal function 
again with 0.05 tolerance all the robots have arrived and i reset the World.
In launch file ,I specified the path to be able to use my world file and I give the pyhton file I will use
here.
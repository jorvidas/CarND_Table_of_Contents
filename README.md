# CarND Table of Contents
Links and descriptions to all CarNd projects

## Term 3

### Path Planning Project ([Github](https://github.com/jorvidas/CarND-Path-Planning-Project))

Utilizes path planning to navigate a car around a three-lane highway with traffic. The car is able to adjust speed based on cars that it sees ahead of it as well as change lanes to get past traffic.  The car does not exceed the 50mph speed limit of the road.

### Semantic Segmentation ([Github](https://github.com/jorvidas/CarND-Semantic-Segmentation))

Uses labeled images from a car's cameras to train a deep network to learn how to classify the individual pixels in an image as road pixels to identify where the road is.  After training, runs on the images and colors the areas identified as road in green to show what the classifier has identified as road.

### Programming a Real Self-Driving Car ([Github](https://github.com/jorvidas/CarND-Capstone))

## Term 2
### Extended Kalman Filters ([Github](https://github.com/jorvidas/CarND-Extended-Kalman-Filter-Project))

Utilizes an extended kalman filter to estimate the state of a moving object.  The data from the RADAR and LIDAR is noisy and the extended kalman filter gives a best guess as to what the state is at the current time from a combination of the most recent data from the sensors and our guess of the car's position at the current time based on our previous estimate of the car's state.

### Unscented Kalman Filters ([Github](https://github.com/jorvidas/CarND-Unscented-Kalman-Filter-Project))

Utilizes an unscented kalman filter to estimate the state of a moving object.  The data from the RADAR and LIDAR is noisy and the unscented kalman filter gives a best guess as to what the state is at the current time from a combination of the most recent data from the sensors and our guess of the car's position at the current time based on our previous estimate of the car's state.

Instead of using derivatives to calculate the covariance like the extended kalman filter, the unscented kalman filter uses sigma points.

### Kidnapped Vehicle ([Github](https://github.com/jorvidas/CarND-Kidnapped-Vehicle-Project))

Localizes the vehicle using initial noisy data from a GPS, a map of the area, noisy sensor data, and noisy control data using a two-dimensional particle filter.

### PID Controller ([Github](https://github.com/jorvidas/CarND-PID-Control-Project))

Using the same track as the behavioral cloning project, the car is driven around the track using a PID controller, data on the cross-track error (distance from the center of the track) for the car's current position, and the car's current speed.  The controller calculates steering angles and throttle values.

### Model Predictive Control ([Github](https://github.com/jorvidas/CarND-MPC-Quizzes))

Using the same track as the previous project, this uses a model predicitive controller instead.  The model predictive controller looks ahead at the road and responds with steering angle deltas and accelerations that steer the car around the track.  Compared to the PID controller which is responding to the car's immediate position on the road instead of what is coming up in the future, this controller has a much smoother ride and is capable of traveling and a faster speed while remaining on the track.

## Term 1
### Finding Lane Lines on the Road ([Github](https://github.com/jorvidas/CarND-LaneLines-P1))

This project is an initial look at processing images and video feeds from a car camera. Here, images and videos are processed and lines are drawn on the image representing our best guess of the lane lines on the road. The project looks for changes in color to determine where lane lines are.  This works well when the road is one color, the lines are another and there are no other sharp color changes.  As can be seen in the challenge video, shadows and changes in the color of the pavement can wreak havoc on the programs ability to accurately detect lane lines.

### Traffic Sign Classifier ([Github](https://github.com/jorvidas/CarND-Traffic-Sign-Classifier-Project))

Using the INI German Traffic Sign Database to create a classifier capable of recognizing german traffic signs.  The classifier uses a deep neural net to achieve 93%+ success at recognizing traffic signs from the test set.  On a test of 5 images pulled from an outside source, 4 of of 5 were correctly classified.

### Behavioral Cloning ([Github](https://github.com/jorvidas/CarND-Behavioral-Cloning-P3))

Uses a deep neural network to generate desired steering angles to keep a car on the road based on feed from the car's camera.  The neural network was trained with camera feeds and corresponding steering angles for a successful driving around (and recovering back onto) the track.

### Advanced Lane Finding ([Github](https://github.com/jorvidas/CarND-Advanced-Lane-Lines))

A more advanced take on the first project.  A better way of finding the lane lines, draws a transparent light green box on the area that is identified as the lane.  Improved usage of different color spaces and using histograms to find the lane lines results in a more stable box that is less fooled by shadows, changes in pavement color, and other tricks of the road.

### Vehicle Detection and Tracking ([Github](https://github.com/jorvidas/CarND-Vehicle-Detection))

Detects and tracks vehicles in camera images from the car's feed. Bounding boxes are then drawn around the car to show its location on the camera feed.  The vehicle detection is performed on image stills and then stitched back into a video showing the cars being detected.  This uses Support Vector Machine to perform the classification.


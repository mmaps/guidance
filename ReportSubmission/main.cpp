#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <fcntl.h>
#include <pthread.h>
#include <stdio.h>
#include <unistd.h>
#include <fstream>
#include <iostream>
#include <string>
#include <map>
#include <errno.h>    /* Error number definitions */
#include <termios.h>  /* POSIX terminal control definitions */
#include <sys/ioctl.h>
#include <getopt.h>

using namespace std;
using namespace cv;

double get_polar_slope(float rho, float theta);
double get_cartesian_slope(int x1, int y1, int x2, int y2);
// Finds edges
void run_canny(cv::Mat a_frame, cv::Mat& ret_val);
// Finds probabilistic hough lines, a small set of finite cartesian lines returned
void run_houghP(cv::Mat a_frame, vector<Vec4i>& ret_val);
// Finds standard hough lines, a large set of polar lines
void run_hough(cv::Mat a_frame, vector<Vec2f>& ret_val);
// Finds tracking features
void run_GFTT(cv::Mat a_frame, vector<Point2f>& ret_val);
// Draw cartesian
void draw_lines(vector<Vec4i> lines, cv::Mat& a_frame);
// Draw polar
void draw_rho_lines(vector<Vec2f> lines, cv::Mat& a_frame);
// Create circles around points
void draw_points(vector<Point2f> lines, cv::Mat& a_frame);
// DEAD for now: Find usable end points of polar line
void get_points_from_line(Vec2f line, Point2f& pt1, Point2f& pt2);
// Finds the intersection of two line segments (a and b)  formed by 4 points, or returns false.
bool intersection (Point2f a1, Point2f a2, Point2f b1, Point2f b2, Point2f *ret_val);
// Finds the intersection of two line segments given only the lines
Point2f compute_intersect(Vec4i line1, Vec4i line2);
// Calc slope of input vector of polar lines and separate/store into neg, pos, vert, and horiz
void separate_slopes(vector<Vec2f> lines, vector<Vec2f>& negatives, vector<Vec2f>& positives, vector<Vec2f>& verticals, vector<Vec2f>& horizontals);
// Calc intersection of two sets of polar lines, returns intersection
bool find_hough_intersection(vector<Vec2f> neg, vector<Vec2f> pos, Point2f& intersection);
// If neg and pos slope this will try and find intersection of neg,neg or pos,pos
bool find_same_slope_intersection(vector<Vec2f> hough_lines_neg, vector<Vec2f> hough_lines_pos, Point2f& intersection_pt);
// Function used in remove_if. Returns true if horiz or vert slope on polar only
bool slope_test(Vec2f line);
// Function used in remove_if. Returns true if not horiz or vert slope, on cartesian only
bool vert_slope_test(Vec4i line);
// Polar line, remove_if test for removing all but verticals
bool polar_vert_slope_test(Vec2f line);
// Looks for rectangles and then writes frame to disk, calls tesseract to parse, and attempts to interpret result
void check_for_box(vector<Vec4i> lines, vector<Point2f>& ret_val, Mat frame, Point2f& ret_min, Point2f& ret_max);
// Attempts to read the rectangle of space found by check for box
void read_sign(Point2f min, Point2f max, Mat frame);
// Checks intersection points and issues directional command if necessary, walks down halls
void guide_down_hall(vector<Point2f>& points);
// Forks a proc to speak command
void speak(string str);
// Open the arduino serial port
int open_arduino();
// Check for IR warnings
void check_arduino(int arduino);
// Carries out the first instruction to the user on localization
void orient_user_to_goal();
// Directional commands
void turn_90(int direction);
// Guide the user at an intersection
void make_a_turn();
bool made_a_turn = false;
// Begin final approach
void begin_docking_run(VideoCapture& camera);

int canny_low = 25;
int canny_high = 100;
// Playing a voice command or not
bool playing = false;
// Do we know where we are?
bool localized = false;
// Intentionally stopped
bool full_stop = false;
// At last hall
bool goal_hallway = false;
// Room#, directions from room to goal
map<int, string> room_directions;
// Directions list
string direct;
// Front Wall Alarm
bool front_alarm = false;
// Begin pathfinding flag
bool good_to_go = false;
RNG rng(12345);

int main(int argc, char* argv[])
{
    // Open video file, check validity
    VideoCapture camera1(atoi(argv[1]));
    if(!camera1.isOpened())
        return 1;
    VideoCapture camera2(atoi(argv[2]));
    if(!camera2.isOpened())
        return 1;
    // Set capture size to 320x240 to improve performance
    camera1.set(CV_CAP_PROP_FRAME_WIDTH, 320);
    camera1.set(CV_CAP_PROP_FRAME_HEIGHT, 240);
    camera2.set(CV_CAP_PROP_FRAME_WIDTH, 320);
    camera2.set(CV_CAP_PROP_FRAME_HEIGHT, 240);

    /************** LOCAL VAR *********************/
    int turn_counter = 0;
    int loop_counter = 0;
    // Frame is the individual frame read in each loop iteration
    Mat frame1;
    Mat frame2;
    // Final frames to display
    Mat result1;
    Mat result2;
    // Edge detection Mats
    Mat canny_result1;
    Mat canny_result2;
    Mat frame_copy;
    Mat threshold_img;
    // Windows to output results (will not be used in final)
    namedWindow("Camera 1");
    namedWindow("Camera 2");
    namedWindow("Original");
    // Vec2f for polar lines from standard hough transform
    vector<Vec2f> hough_lines1;
    vector<Vec2f> hough_lines2;
    // Vec4i for cartesian lines from probabilistic hough transform
    vector<Vec4i> hough_linesP;
    // Intersection points gathered used for finding mean point
    vector<Point2f> int_points;
    vector<Point2f> int_points2;
    // Makes up int_points vector
    Point2f intersection_pt;
    Point2f intersection_pt2;
    int counter;
    // Open Arduino connection
    int arduino = open_arduino();
    if (arduino < 0)
    {
        printf("Couldn't open Arduino!\n");
        speak("could, not, open, arduino");
    }
    // Get room directions
    ifstream directions_file;
    directions_file.open("map/directions.txt");
    if(!directions_file.is_open())
    {
        printf("Cannot open directions file\n");
        speak("Cannot, open, directions, file");
        exit(1);
    }
    string buffer;
    int key;
    string value;
    while(!directions_file.eof())
    {
        getline(directions_file, buffer);
        if(buffer[0] == '5')
            key = atoi(buffer.c_str());
            getline(directions_file, value);
            room_directions.insert(pair<int, string>(key, value));
    }
    map<int, string>::iterator it = room_directions.begin();
    directions_file.close();
    speak("Loaded directions");

    // Pause before starting
    while(good_to_go)
    {
        if(!camera1.read(frame_copy))
        {
            speak("Camera 1, error");
            break;
        }
        vector<Point2f> corners_1;
        Point2f min, max;
        GaussianBlur(frame_copy, frame1, Size(3,3), 0);
        run_canny(frame1, canny_result1);
        run_houghP(canny_result1, hough_linesP);
        hough_linesP.erase(remove_if(hough_linesP.begin(), hough_linesP.end(), vert_slope_test), hough_linesP.end());
        // Change canny result to color for lines
        cvtColor(canny_result1, result1, CV_GRAY2BGR);
        draw_lines(hough_linesP, result1);
        if(hough_linesP.size()>1)
        {
            check_for_box(hough_linesP, corners_1, frame1, min, max);
            if(corners_1.size() > 2 && min.x < 999 && min.y < 999 && max.x > 0 && max.x > 0)
            {
                read_sign(min, max, frame_copy);
            }
        }
        imshow("Camera 1", result1);
        if(waitKey(10)>=0)
            exit(0);
    }
    speak("Guidance system initialized");
    sleep(1);

/********************************************************************/
/*************************** MAIN LOOP  *****************************/
/********************************************************************/
    while(true)
    {
        loop_counter++;
        if(loop_counter == 10000000)
            loop_counter = 0;
        // Voice command index
        char* cmd;

        /************** ARDUINO *********************/
        check_arduino(arduino);

        /********** READ FRAMES **********************/
        if(!camera1.read(frame1))
        {
            speak("Camera 1, error");
            break;
        }
        if(!camera2.read(frame2))
        {
            speak("Camera 2, error");
            break;
        }

        /************** CAMERA 1 *********************/
        run_canny(frame1, canny_result1);
        run_hough(canny_result1, hough_lines1);
        vector<Vec2f> hough_lines_neg;
        vector<Vec2f> hough_lines_pos;
        vector<Vec2f> hough_lines_ver;
        vector<Vec2f> hough_lines_hor;
        separate_slopes(hough_lines1, hough_lines_neg, hough_lines_pos, hough_lines_ver, hough_lines_hor);
        // Change canny result to color for lines
        cvtColor(canny_result1, result1, CV_GRAY2BGR);
        //draw_rho_lines(hough_lines_neg, result1);
        //draw_rho_lines(hough_lines_pos, result1);
        //draw_rho_lines(hough_lines_ver, result1);
        //draw_rho_lines(hough_lines_hor, result1);
        // If no result from vanishing point of opposite slopes...
        if(!find_hough_intersection(hough_lines_neg, hough_lines_pos, intersection_pt))
        {
            //...then check the same slopes of negatives and positives
            find_same_slope_intersection(hough_lines_neg, hough_lines_pos, intersection_pt);
        }
        circle(result1, intersection_pt, 10, Scalar(255,0,0), 10);
        int_points.push_back(intersection_pt);

        /************** CAMERA 2 *********************/
        GaussianBlur(frame2, frame2, Size(3,3), 0);
        vector<Point2f> corners;
        run_canny(frame2, canny_result2);
        // If you know where you are use second camera to look for intersections
        vector<Vec2f> hough_lines_neg2;
        vector<Vec2f> hough_lines_pos2;
        vector<Vec2f> hough_lines_ver2;
        vector<Vec2f> hough_lines_hor2;
        // Change canny result to color for lines
        cvtColor(canny_result2, result2, CV_GRAY2BGR);
        if(localized && !goal_hallway && (!made_a_turn || (loop_counter - turn_counter > 100)))
        {
            if(loop_counter - turn_counter > 100)
                made_a_turn = false;
            counter++;
            run_hough(canny_result2, hough_lines2);
            separate_slopes(hough_lines2, hough_lines_neg2, hough_lines_pos2, hough_lines_ver2, hough_lines_hor2);
            draw_rho_lines(hough_lines_neg2, result2);
            draw_rho_lines(hough_lines_pos2, result2);
            // Check for vanishing points indicating an intersection
            if(find_hough_intersection(hough_lines_neg2, hough_lines_pos2, intersection_pt2))
            {
                circle(result2, intersection_pt2, 10, Scalar(255,255,0), 10);
                int_points2.push_back(intersection_pt2);
            }
            // Accumulate some points and check them
            if(int_points2.size() > 10)
            {
                if(counter < 50)
                {
                    int sum = 0;
                    double avg = 0;
                    for(int i=0; i<int_points2.size(); i++)
                    {
                        sum += int_points2[i].x;
                    }
                    avg = sum / int_points2.size();
                    if(avg > 80 && avg < 240)
                    {
                        if(!made_a_turn)
                        {
                            turn_counter = loop_counter;
                            make_a_turn();
                        }
                    }
                    int_points2.clear();
                }else
                {
                    counter = 0;
                    int_points2.clear();
                }
            }
        }
        // If you don't know where you are Look for room signs
        if(!localized)
        {
            Point2f min, max;
            run_houghP(canny_result2, hough_linesP);
            hough_linesP.erase(remove_if(hough_linesP.begin(), hough_linesP.end(), vert_slope_test), hough_linesP.end());
            if(hough_linesP.size()>1)
            {
                check_for_box(hough_linesP, corners, frame2, min, max);
                if(corners.size() > 2 && min.x < 999 && min.y < 999 && max.x > 0 && max.x > 0)
                {
                    read_sign(min, max, frame2);
                    if(localized)
                    {
                        orient_user_to_goal();
                    }
                }
            }
            // Change canny result to color for lines
            cvtColor(canny_result2, result2, CV_GRAY2BGR);
            draw_lines(hough_linesP, result2);
            //printf("Hough P Lines.size: %d\n", hough_linesP.size());
        }

        /********* COMMAND CHECK *********************/
        if(!full_stop)
        {
            /*
            if(goal_hallway)
            {
                //destroyAllWindows();
                //begin_docking_run(camera1);
                guide_down_hall(int_points);
            }
            else 
            */
            if(int_points.size() > 15)
            {
                guide_down_hall(int_points);
            }
            if(front_alarm &&  int_points.size() < 2)
            {
                make_a_turn();
            }
        }

        // DISPLAY RESULTS, INCREMENT LOOP, CHECK FOR EXIT KEY
        imshow("Original", frame2);
        imshow("Camera 1", result1);
        imshow("Camera 2", result2);
        if(waitKey(10)>=0)
            break;
    }
}

/********************************************************************/
/********************************************************************/
/********************************************************************/

void orient_user_to_goal()
{
    char cmd = direct[0];
    if(cmd == 'B')
    {
        speak("your, path, is, behind, you");
        speak("i, will, ask, you, to, turn, left, twice");
        speak("please, turn, left, 90, degrees");
        turn_90(0);
        speak("please, turn, left, 90, degrees, again");
        turn_90(0);
    }
    direct[0] = 'X';
    speak("thank you for your patience. please continue ahead, slowly");
    if(direct[strlen(direct.c_str())-1] == 'X')
    {
        goal_hallway = true;
    }
}

void make_a_turn()
{
    made_a_turn = true;
    char cmd;
    if(strlen(direct.c_str()) == 0)
    {
        speak("Please turn left");
        sleep(2);
        return;
    }/*
    for(int i=0; i<strlen(direct.c_str()); i++)
    {
        cmd = direct[i];
        if(cmd != 'X'){
            break;
        }
    }*/
    int i=1;
    cmd = direct[i];
    while(cmd == 'X'){
        i++;
        cmd = direct[i];
    }
    speak("you are at a turning point. please, turn, to, the");
    if(cmd == 'R')
        speak("right.");
    if(cmd == 'L')
        speak("left.");
    if(cmd == 'B')
    {
        speak("left.");
        sleep(2);
        speak("please, turn left 90 degrees, again");
    }
    sleep(2);
    direct[i] = 'X';
    if(direct[strlen(direct.c_str())-1] == 'X')
    {
        goal_hallway = true;
    }
}

void begin_docking_run(VideoCapture& camera)
{
    speak("you,are,facing,down,the,hall,to,room,5804");
    speak("please,walk,forward,slowly");
    namedWindow("DOCKING");
    Mat frame, canny_result, result;
    while(true)
    {
        if(!camera.read(frame))
            break;
        //
        Point2f pt1, pt2;
        int l_sum = 0, r_sum=0, l_avg = 0, r_avg=0;
        double ratio = 0.;
        vector<Point2f> l_points;
        vector<Point2f> r_points;
        vector<Vec2f> hough_lines;
        vector<Vec2f> hough_lines_neg;
        vector<Vec2f> hough_lines_pos;
        vector<Vec2f> hough_lines_ver;
        vector<Vec2f> hough_lines_hor;
        //
        GaussianBlur(frame, frame, Size(3,3), 0);
        run_canny(frame, canny_result);
        //run_hough(canny_result, hough_lines);
        HoughLines(canny_result, hough_lines, 1, CV_PI/180, 40, 0, 0);
        hough_lines.erase(remove_if(hough_lines.begin(), hough_lines.end(), polar_vert_slope_test), hough_lines.end());
        //separate_slopes(hough_lines, hough_lines_neg, hough_lines_pos, hough_lines_ver, hough_lines_hor);
        cvtColor(canny_result, result, CV_GRAY2BGR);
        draw_rho_lines(hough_lines_pos, result);
        draw_rho_lines(hough_lines_neg, result);
        // Find averages
        for(int i=0; i<hough_lines.size(); i++){
            get_points_from_line(hough_lines[i], pt1, pt2);
            printf("POINTS %d,%d    %d,%d\n", pt1.x,pt1.y,pt2.x,pt2.y);
            if(pt1.x > frame.cols/2)
            {
                r_points.push_back(pt1);
                r_sum += pt1.x;
            }
            else
            {
                l_points.push_back(pt1);
                l_sum += pt1.x;
            }
            //printf("R SUM: %d\n", r_sum);
            //printf("L SUM: %d\n", l_sum);
        }
        if(r_points.size() == 0 || l_points.size() == 0)
            continue;
        l_avg = (l_sum) / l_points.size();
        printf("L AVG: %d\n", l_avg);
        r_avg = (r_sum) / r_points.size();
        printf("R AVG: %d\n", r_avg);
        // Compare averages
        ratio = abs(l_avg-170.f) / abs(r_avg-170.f);
        printf("RATIO: %f\n", ratio);
        if(ratio > 1.25)
        {
            speak("bear left");
        }
        else if(ratio < .75)
        {
            speak("bear right");
        }
        // Change canny result to color for lines
        cvtColor(canny_result, result, CV_GRAY2BGR);
        draw_rho_lines(hough_lines_ver, result);
        imshow("DOCKING", result);
        if(waitKey(10)>=0)
            exit(0);
    }
}

void turn_90(int direction)
{
    sleep(3);
}

void guide_down_hall(vector<Point2f>& int_points)
{
    // Averaging intersection points sum/avg
    double sum = 0, avg = 0;
    for(int i=0; i<int_points.size(); i++)
    {
        //printf("X %f\n",int_points[i].x);
        sum = sum+int_points[i].x;
    }
    //printf("SUM %f\n", sum);
    //printf("LEN %d\n", int_points.size());
    avg = sum/int_points.size();
    //printf("AVG %f\n", avg);
    int_points.clear();
    // Guidance loop, serves to guide person down hall
    if(!playing)
    {
        playing = true;
        //printf("%f\n", avg);
        if (!fork())
        {
            if (avg < 80)
            {
                //printf("LEFT\n");
                system("espeak \"bear left\"");
            }
            else if (avg > 240)
            {
                //printf("RIGHT\n");
                system("espeak \"bear right\"");
            }
            else
            {
                //printf("FORWARD\n");
                system("espeak \"step forward\"");
            }
            exit(0);
        }
        playing = false;
    }
}

void speak(string str){
    string command = string("espeak \"" + str + "\"");
    system(command.c_str());
}

void check_for_box(vector<Vec4i> lines, vector<Point2f>& ret_val, Mat frame, Point2f& ret_min, Point2f& ret_max)
{
    //printf("Check for box\n");
    Point2f min(999.f,999.f), max(0.f,0.f);
    Point2f pt;
    for (int i = 0; i < lines.size(); i++)
    {
        for (int j = i+1; j < lines.size(); j++)
        {
            pt = compute_intersect(lines[i], lines[j]);
            if(pt.x > 0 && pt.y > 0){
                if(pt.x < frame.cols && pt.y < frame.rows){
                    ret_val.push_back(pt);
                    if(pt.x > max.x && pt.y > max.y && abs(pt.x-min.x) > 30 && abs(pt.y-min.y)>30){
                        max = pt;
                    }
                    else if(pt.x < abs(min.x) && pt.y < abs(min.y) && abs(pt.x-max.x) > 30 && abs(pt.y-max.y)>30){
                        min = pt;
                    }
                }
            }
        }
    }
    ret_min = min;
    ret_max = max;
    namedWindow("CORNERS");
    Mat corner_pic = Mat::zeros(Size(320,240), CV_8UC3);
    draw_points(ret_val, corner_pic);
    rectangle(corner_pic, min, max, Scalar(255,255,0));
    imshow("CORNERS", corner_pic);
}

void read_sign(Point2f min, Point2f max, Mat frame)
{
    if(min.x < frame.cols && min.y < frame.rows && min.x > 0 && min.y > 0 && max.x > 0  && max.y > 0 && max.x < frame.cols && max.y < frame.rows){
        Rect ROI(min, max);
        Mat ROImage(frame, ROI);
        imwrite("panel.tiff", ROImage);
        system("tesseract panel.tiff out &> /dev/null");
        ifstream file_in;
        string str;
        file_in.open("out.txt");
        if(!file_in.is_open())
        {
            printf("Cannot open tesseract file\n");
            speak(string("Cannot, open, tesseract, file"));
        }
        while(!file_in.eof()){
            getline(file_in, str);
            int room = atoi(str.c_str());
            if(room > 5000 && room <6000){
                speak("please, stop");
                speak("You, are, outside, room");
                speak(str);
                speak("calculating, path, to, goal");
                direct = room_directions[room];
                printf("Directions: %s\n", direct.c_str());
                localized = true;
                break;
            }
            if(room > 400 && room < 500)
            {
                good_to_go = true;
                sleep(1);
                break;
            }
            if(room > 50 && room < 100){
                good_to_go = true;
                sleep(1);
                break;
            }
            if(str[0] == 'x' || str[0] == 'X')
            {
                good_to_go = true;
                sleep(1);
                break;
            }
        }
        file_in.close();
    }
}

Point2f compute_intersect(Vec4i line1, Vec4i line2)
{
    //printf("Compute intersect\n");
    int x1 = line1[0], y1 = line1[1], x2 = line1[2], y2 = line1[3];
    int x3 = line2[0], y3 = line2[1], x4 = line2[2], y4 = line2[3];
    if (float d = ((float)(x1-x2) * (y3-y4)) - ((y1-y2) * (x3-x4)))
    {
        Point2f pt;
        pt.x = ((x1*y2 - y1*x2) * (x3-x4) - (x1-x2) * (x3*y4 - y3*x4)) / d;
        pt.y = ((x1*y2 - y1*x2) * (y3-y4) - (y1-y2) * (x3*y4 - y3*x4)) / d;
        return pt;
    }
    else
        return Point2f(-1, -1);
}

bool find_same_slope_intersection(vector<Vec2f> hough_lines_neg, vector<Vec2f> hough_lines_pos, Point2f& intersection_pt)
{
    //printf("Same Slope\n");
    bool result;
    result = find_hough_intersection(hough_lines_neg, hough_lines_neg, intersection_pt);
    if(!result){
        result = find_hough_intersection(hough_lines_pos, hough_lines_pos, intersection_pt);
    }
    if(intersection_pt.x < 50 || intersection_pt.x > 250)
        result = false;
    if(intersection_pt.y < 35 || intersection_pt.y > 200)
        result = false;
    return result;
}

void separate_slopes(vector<Vec2f> lines, vector<Vec2f>& negatives, vector<Vec2f>& positives, vector<Vec2f>& verticals, vector<Vec2f>& horizontals)
{
    //printf("Separate slopes\n");
    for(int i=0; i<lines.size(); i++)
    {
        Vec2f line = lines[i];
        float rho = line[0], theta = line[1];
        double a = cos(theta), b = sin(theta);
        double x0 = a*rho, y0 = b*rho;
        double m = (-a)/b;
        if(m < -.2 && m > -7)
        {
            negatives.push_back(line);
        }
        else if(m > .2 && m < 7)
        {
            positives.push_back(line);
        }
        else if(m > 7 || m < -7)
        {
            verticals.push_back(line);
        }
        else
        {
            horizontals.push_back(line);
        }
    }
    negatives.erase(remove_if(negatives.begin(), negatives.end(), slope_test), negatives.end());
    positives.erase(remove_if(positives.begin(), positives.end(), slope_test), positives.end());
}

void run_GFTT(cv::Mat a_frame, vector<Point2f>& ret_val)
{
    //goodFeaturesToTrack(a_frame, ret_val, 10, .9, 10.0, cv::noArray(), 5, false);
    goodFeaturesToTrack(a_frame, ret_val, 10, .9, 10.0);
}

void draw_points(vector<Point2f> points, cv::Mat& ret_val)
{
    //printf("Draw points\n");
    for(size_t i=0; i<points.size(); i++){
        circle(ret_val, points.at(i), 3, Scalar(255,0,0), 2);
    }
}

void draw_lines(vector<Vec4i> lines, cv::Mat& a_frame)
{
    //printf("Draw lines\n");
    for(size_t i=0; i<lines.size(); i++){
        Vec4i l = lines[i];
        //if(abs(l[0] - l[2]) < 30 || abs(l[1] - l[3]) < 30)
        line(a_frame, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 3, CV_AA);
    }
}

bool find_hough_intersection(vector<Vec2f> neg, vector<Vec2f> pos, Point2f& intersection_pt)
{
    //printf("Find Hough\n");
    if(neg.size()==0 || pos.size()==0)
        return false;
    int votes = 0;
    float rho, theta;
    double a, b;
    double x0, y0;
    Point2f a1, a2, b1, b2, intersect;
    bool  result;
    int counter = 0;
    neg.erase(remove_if(neg.begin(), neg.end(), slope_test), neg.end());
    pos.erase(remove_if(pos.begin(), pos.end(), slope_test), pos.end());
    if(neg.size()==0 || pos.size()==0)
        return false;
    do{
        counter++;
        votes=0;
        int idx1 = rng.uniform(0, neg.size());
        int idx2 = rng.uniform(0, pos.size());
        //
        rho = neg[idx1][0]; theta = neg[idx1][1];
        a = cos(theta); b = sin(theta);
        x0 = a*rho; y0 = b*rho;
        double m_a =abs((-a)/b);
        a1.x = cvRound(x0 + 1000*(-b));
        a1.y = cvRound(y0 + 1000*(a));
        a2.x = cvRound(x0 - 1000*(-b));
        a2.y = cvRound(y0 - 1000*(a));
        //
        rho = pos[idx2][0]; theta = pos[idx2][1];
        a = cos(theta); b = sin(theta);
        x0 = a*rho; y0 = b*rho;
        double m_b =abs((-a)/b);
        if(abs((m_a/m_b)-1) < .1){
            return false;
        }
        b1.x = cvRound(x0 + 1000*(-b));
        b1.y = cvRound(y0 + 1000*(a));
        b2.x = cvRound(x0 - 1000*(-b));
        b2.y = cvRound(y0 - 1000*(a));
        //
        result = intersection(a1, a2, b1, b2, &intersect);
        for(size_t i = 0; i<neg.size(); i++){
          rho = neg[i][0]; theta = neg[i][1];
          a = cos(theta); b = sin(theta);
          double distance = intersect.x*a + intersect.y*b - rho;
          if(distance < 10){
              votes++;
          }
        }
        for(size_t i = 0; i<pos.size(); i++){
          rho = pos[i][0]; theta = pos[i][1];
          a = cos(theta); b = sin(theta);
          double distance = intersect.x*a + intersect.y*b - rho;
          if(distance < 10){
              votes++;
          }
        }
    }while((votes < (neg.size()/2.)) && (counter < 10));
    //printf("INTERSECTION %f, %f\n", intersect.x, intersect.y);
    if(result){
      intersection_pt = intersect;
      return result;
    }
}

void draw_rho_lines(vector<Vec2f> lines, cv::Mat& a_frame)
{
    //printf("Draw Rho Lines\n");
    float rho, theta;
    double a, b;
    double x0, y0;
    Point2f a1, a2, b1, b2, intersect;
    bool  result;
    for( size_t i = 0; i < lines.size(); i++ )
    {
      rho = lines[i][0]; theta = lines[i][1];
      Point pt1, pt2;
      a = cos(theta); b = sin(theta);
      double m =abs((-a)/b);
      double x0 = a*rho, y0 = b*rho;
      pt1.x = cvRound(x0 + 1000*(-b));
      pt1.y = cvRound(y0 + 1000*(a));
      pt2.x = cvRound(x0 - 1000*(-b));
      pt2.y = cvRound(y0 - 1000*(a));
      line( a_frame, pt1, pt2, Scalar(0,0,255), 1, CV_AA);
    }
}

double get_cartesian_slope(int x1, int y1, int x2, int y2)
{
    //printf("Get cartestian\n");
    //printf("    %d,%d    %d,%d\n", x1,y1,x2,y2);
    double m;
    if(x2-x1>0){
        m = (y2-y1)/(x2-x1);
        return m;
    }
    else
        return 999.0;
}

double get_polar_slope(float rho, float theta)
{
    //printf("Get polar slope\n");
    double a = cos(theta), b = sin(theta);
    double x0 = a*rho, y0 = b*rho;
    return abs((-a)/b);
}

bool slope_test(Vec2f line)
{
    //printf("Slope test\n");
    double m = get_polar_slope(line[0], line[1]);
    if(m < .2 || m > 5)
      return true;
    return false;
}

bool polar_vert_slope_test(Vec2f line)
{
    double m = get_polar_slope(line[0], line[1]);
    if(m > .1 && m < 7)
      return true;
    return false;
}

bool vert_slope_test(Vec4i line)
{
    //printf("Vert Slope test\n");
    /*
    int x1 = line[0], y1 = line[1], x2 = line[2], y2 = line[3];
    double m = abs((y2-y1)/(x2-x1));
    */
    double m = abs(get_cartesian_slope(line[0], line[1], line[2], line[3]));
    if(m != 999)
    {
        if(m > .2 && m < 5)
          return true;
    }
    else
    {
        return false;
    }
}

void get_points_from_line(Vec2f line, Point2f& pt1, Point2f& pt2)
{
    float rho = line[0], theta = line[1];
    double a = cos(theta), b = sin(theta);
    double x0 = a*rho, y0 = b*rho;
    pt1.x = cvRound(x0 + 1000*(-b));
    pt1.y = cvRound(y0 + 1000*(a));
    pt2.x = cvRound(x0 - 1000*(-b));
    pt2.y = cvRound(y0 - 1000*(a));
}

void run_houghP(cv::Mat a_frame, vector<Vec4i>& ret_val)
{
    HoughLinesP(a_frame, ret_val, 1, CV_PI/180, 60, 80, 10);
}

void run_hough(cv::Mat a_frame, vector<Vec2f>& ret_val)
{
    HoughLines(a_frame, ret_val, 1, CV_PI/180, 50, 0, 0);
}

void run_canny(cv::Mat a_frame, cv::Mat& ret_val)
{
    Mat small;
    Scalar img_mean = 0.;
    cvtColor(a_frame, ret_val, CV_BGR2GRAY);
    equalizeHist(ret_val, ret_val);
    // Median blur produces blobular images at lower resolutions
    //medianBlur(ret_val, ret_val, 11); 
    pyrDown(ret_val, small, Size(a_frame.cols/2, a_frame.rows/2));
    pyrUp(small, ret_val, Size(small.cols*2, small.rows*2));
    img_mean = mean(ret_val);
    double wong_hi = img_mean[0] * 1.66;
    double wong_lo = 0.4 * wong_hi;
    Canny(ret_val, ret_val, wong_hi, wong_lo);
}

bool intersection (Point2f a1, Point2f a2, Point2f b1, Point2f b2, Point2f *r)
{
   Point2f x = b1 - a1;
   Point2f d1 = a2 - a1;
   Point2f d2 = b2 - b1;

   float cross = d1.x*d2.y - d1.y*d2.x;
   if (abs(cross) < /*EPS*/1e-8)
     return false;

   double t1 = (x.x * d2.y - x.y * d2.x)/cross;
   *r = a1 + d1 * t1;
   return true;
}


/* Setup and return a serial port file descriptor to read from the arduino.
 * Credit: http://todbot.com/blog/2006/12/06/arduino-serial-c-code-to-talk-to-arduino/
 */
int open_arduino() {
    struct termios toptions;
    int fd;
    
    //fprintf(stderr,"init_serialport: opening port %s @ %d bps\n",
    //        serialport,baud);

    fd = open("/dev/ttyACM0", O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1)  {
        perror("init_serialport: Unable to open port ");
        return -1;
    }
    
    if (tcgetattr(fd, &toptions) < 0) {
        perror("init_serialport: Couldn't get term attributes");
        return -1;
    }
    
    speed_t brate = B9600;
    
    cfsetispeed(&toptions, brate);
    cfsetospeed(&toptions, brate);

    // 8N1
    toptions.c_cflag &= ~PARENB;
    toptions.c_cflag &= ~CSTOPB;
    toptions.c_cflag &= ~CSIZE;
    toptions.c_cflag |= CS8;
    // no flow control
    toptions.c_cflag &= ~CRTSCTS;

    toptions.c_cflag |= CREAD | CLOCAL;  // turn on READ & ignore ctrl lines
    toptions.c_iflag &= ~(IXON | IXOFF | IXANY); // turn off s/w flow ctrl

    toptions.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // make raw
    toptions.c_oflag &= ~OPOST; // make raw

    // see: http://unixwiz.net/techtips/termios-vmin-vtime.html
    toptions.c_cc[VMIN]  = 0;
    toptions.c_cc[VTIME] = 20;
    
    if( tcsetattr(fd, TCSANOW, &toptions) < 0) {
        perror("init_serialport: Couldn't set term attributes");
        return -1;
    }

    return fd;
}

// Check that there is no warning on the front sensor
bool front_sensor_clear(int arduino) {
    char reading = -1;
    char req = 1;
    write(arduino, &req, 1); // Make data available to send request
    read(arduino, &reading, 1);
    return (reading != 'f' && reading != 'F');

    /* To query for the reading itself: */
    // int reading = -1;
    // char req = 'f';
    // write(arduino, &req, 1); // Request the front sensor reading
    // read(arduino, &reading, sizeof(reading));
    // return reading; // Returns an int now
}

// Check the IR sensors
void check_arduino(int arduino) {
    char reading;
    char req = 1;
    write(arduino, &req, 1); // Make data available to send request
    // Read from the serial port
    while (read(arduino, &reading, 1) > 0) {
        if (reading == 'F') {
            printf("WARN FRONT!\n");
            front_alarm = true;
            speak("Stop, stop, please, just stop");
        } else if (reading == 'R') {
            printf("WARN RIGHT!\n");
            speak("Stop");
            speak("wall on right");
            speak("Step, left");
        } else if (reading == 'L') {
            printf("WARN LEFT!\n");
            speak("Stop");
            speak("wall on left");
        } else if (reading == 'f') {
            printf("Wall ahead...\n");
            front_alarm = true;
            speak("Slow down, wall ahead");
        } else {
            front_alarm = false;
            return; // Sensors are clear!
        }
        
        sleep(1);      // Pause while user acts on request
        write(arduino, &req, 1); // Make data available to send request
    }
}

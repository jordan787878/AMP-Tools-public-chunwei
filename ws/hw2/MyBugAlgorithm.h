#pragma once
#include "AMPCore.h"
#include "hw/HW2.h"

using namespace std;
struct MyVect2D{
    double x,y;
};

struct PosDir{
    MyVect2D r;
    MyVect2D tdir;
    MyVect2D ndir;
    double x, y, dx, dy;
};

struct Sense{
    double x1, y1, x2, y2;
};

struct line2D{
    double a,b,c;
};

struct Matrix2D{
    double M11, M12, M21, M22;
};

struct MyRay2D{
    MyVect2D pos;
    MyVect2D dir;
    double t;
};

struct LineInfo{
    MyRay2D ray;
    line2D  l;
};

struct BugMemory{
    MyVect2D Q_Hit;
    MyVect2D Q_Leave;
    int steps_follow;
    MyVect2D Q_Close;
    double d_to_goal;
    int steps_close;
    double gap;
    LineInfo mline_info;
    double t_intersect_mray;
};


/// @brief Declare your bug algorithm class here. Note this class derives the bug algorithm class declared in HW2.h
class MyBugAlgorithm : public amp::BugAlgorithm {
    public:
        // Override and implement the bug algorithm in the plan method. The methods are declared here in the `.h` file
        virtual amp::Path2D plan(const amp::Problem2D& problem) override;
        int bu1orbug2 = 2;
    
    private:
        double path_length = 0;
        MyVect2D r_goal;
        PosDir posdir;
        double stepsize = 0.05;
        double forward_or_backward = 1.0;
        int max_steps = 100000;
        vector<vector<LineInfo>> Obs_Info;
        Sense SensePoints;
        BugMemory BugMem;
        bool DEBUG_PRINT = false;

        //Bug1 or Bug2 plan
        amp::Path2D plan_bug1(const amp::Problem2D& problem);
        amp::Path2D plan_bug2(const amp::Problem2D& problem);

        /// Init Functions
        void ini_plan();
        void ini_goal(const amp::Problem2D& problem);
        void init_posdir(const amp::Problem2D& problem);
        void init_obs_info(const amp::Problem2D& problem);
        void init_bug_mem();
        vector<LineInfo> get_singleobs_info(const amp::Polygon single_obs);
        ///////

        /// Action Functions
        void move_forward(const amp::Problem2D& problem);
        void left_turn();
        bool hit_obstacle();
        bool point_reached(const MyVect2D);
        //bool encounter_qhit();
        ///////

        /// Follow Boundary Functions
        void follow_boundary_update_dir(const amp::Problem2D& problem);
        MyVect2D follow_boundary_get_local_tangent(const MyVect2D, const vector<vector<LineInfo>>);
        ///////

        /// Helper Functions
        line2D cal_line2D(double x1, double y1, double x2, double y2);
        MyRay2D cal_ray2D(double, double, double, double);
        double cal_distance(MyVect2D, MyVect2D);
        MyVect2D norm_vect(MyVect2D);
        MyVect2D rotate_theta(MyVect2D, double);
        MyVect2D matrix_mul(Matrix2D, MyVect2D);
        MyVect2D get_two_line_intersect(line2D, line2D);
        MyVect2D ray_cast_single_obs(const LineInfo, vector<LineInfo>);
        void set_sense_point(const LineInfo l1_info, 
                             const LineInfo l2_info,
                             const vector<LineInfo> l_singleobs_info);
        MyVect2D regression(const vector<MyVect2D>);
        void update_dir_to_local_tangent(const MyVect2D );
        void update_bug_mem();
        void update_gap();
        ///////

        ///Print Functions
        void print_line2D(line2D);
        void print_myray2D(const MyRay2D);
        void print_mylineinfo(const LineInfo);
        void print_obslineinfo(const vector<LineInfo>);
        void print_posdir();
        void print_bugmemory();
        ///////


        //vector<bool> cal_single_col_bools(const amp::Polygon single_obs);
        //void init_col_bools(const amp::Problem2D& problem);
        //bool move_to_goal(const amp::Problem2D& problem);
        //vector<float> cal_col_singleobs(vector<line2D>single_obs_primitive);
        //void set_obs_col(vector<vector<line2D>> obs_primitivies);
        //void detect_collision();
        //bool goal_is_reached(const amp::Problem2D& problem);
        //bool hit_obstacle(const vector<vector<float>> Obs_Col_Old);
        //vector<bool> cal_singleobs_colbools(const amp::Polygon single_obs, vector<line2D> singleobs_primitive);
        //void print_singleobs_colbools(vector<bool>);
        //void init_colbools(const amp::Problem2D& problem);
        //void print_colbools();
        /*vector<line2D> cal_singleobs_primitive(const amp::Polygon single_obs);
        void init_primitives(const amp::Problem2D& problem);
        void print_primitives();*/

};

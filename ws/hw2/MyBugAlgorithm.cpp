#include "MyBugAlgorithm.h"

amp::Path2D MyBugAlgorithm::plan(const amp::Problem2D& problem){
    amp::Path2D path;
    if(bu1orbug2 == 1){
        path = plan_bug1(problem);
    }
    else{
        path = plan_bug2(problem);
    }
    return path;
}

amp::Path2D MyBugAlgorithm::plan_bug1(const amp::Problem2D& problem){
    cout << "\n\n=====BUG1=====\n\n";
    ini_plan();
    ini_goal(problem);
    init_posdir(problem);
    init_obs_info(problem);

    amp::Path2D path;
    path.waypoints.push_back(problem.q_init);

    while(true){
        if(point_reached(r_goal) || hit_obstacle()){
            // [Reach Goal]
            if(point_reached(r_goal)){
                path.waypoints.push_back(problem.q_goal);
                cout << "GOAL REACH\n";
                cout << "Path Length: " << path_length << endl;
                return path;
            }

            // [Hit Obstacle]
            // record (x,y) into (Q_hit) and (path) after hitting the obstacle
            init_bug_mem();
            path.waypoints.push_back(Eigen::Vector2d(posdir.r.x, posdir.r.y));

            // [Left Turn Robot]
            left_turn();
            
            // [Follow the Boundary]
            cout << "Follow The Boundary\n";
            while(true){
                if(point_reached(r_goal)){
                    path.waypoints.push_back(problem.q_goal);
                    cout << "GOAL REACH\n";
                    cout << "Path Length: " << path_length << endl;
                    return path;
                }
                if(point_reached(BugMem.Q_Hit) && BugMem.steps_follow > 100){
                    break;
                }
                //Failure Check
                if(BugMem.steps_follow > max_steps){
                    cout << "Bug1 Fail in Boundary Follow\n";
                    return path;
                }
  
                update_bug_mem();
                update_gap();
                move_forward(problem);
                follow_boundary_update_dir(problem);
                path.waypoints.push_back(Eigen::Vector2d(posdir.r.x, posdir.r.y));
            }

            cout << "Encounter Q_Hit\n";
            cout << "Set Forward/Backward: ";
            if(BugMem.steps_close > 0.5*BugMem.steps_follow){
                cout << "Backward\n";
                forward_or_backward = -1.0;
            }
            if(DEBUG_PRINT){
                print_bugmemory();
            }
            path.waypoints.push_back(Eigen::Vector2d(posdir.r.x, posdir.r.y));

            // [Move To Q_Close]
            // decides moving direction: (1,-1)
            while(true){
                if(point_reached(BugMem.Q_Close)){
                    break;
                }
                //Failure Check
                if(BugMem.steps_follow > max_steps){
                    cout << "Bug1 Fail in Go To Q_Close\n";
                    return path;
                }

                // Follow The Boundary to Q_Hit
                update_gap();
                move_forward(problem);
                follow_boundary_update_dir(problem);
                path.waypoints.push_back(Eigen::Vector2d(posdir.r.x, posdir.r.y));
            }

            cout << "End Follow Boundary\n";
            // update moving direction
            posdir.tdir = norm_vect(MyVect2D{
                                   r_goal.x-posdir.r.x, 
                                   r_goal.y-posdir.r.y});
            forward_or_backward = 1.0;

            // debugging
            //break;
        }
        // Move To Gaol
        move_forward(problem);
    }
    return path;
}

amp::Path2D MyBugAlgorithm::plan_bug2(const amp::Problem2D& problem){
    cout << "\n\n=====BUG2=====\n\n";
    ini_plan();
    ini_goal(problem);
    init_posdir(problem);
    init_obs_info(problem);

    amp::Path2D path;
    path.waypoints.push_back(problem.q_init);

    while(true){
        if(point_reached(r_goal) || hit_obstacle()){
            // [Reach Goal]
            if(point_reached(r_goal)){
                path.waypoints.push_back(problem.q_goal);
                cout << "GOAL REACH\n";
                cout << "Path Length: " << path_length << endl;
                return path;
            }

            // [Hit Obstacle]
            // record (x,y) into (Q_hit) and (path) after hitting the obstacle
            init_bug_mem();
            path.waypoints.push_back(Eigen::Vector2d(posdir.r.x, posdir.r.y));

            // [Left Turn Robot]
            left_turn();
            
            // [Follow the Boundary]
            cout << "Follow The Boundary\n";
            while(true){
                if(point_reached(BugMem.Q_Hit) && BugMem.steps_follow > 500){
                    cout << "No PATH EXISTS\n";
                    return path;
                    break;
                }
                if(point_reached(r_goal)){
                    path.waypoints.push_back(problem.q_goal);
                    cout << "GOAL REACH\n";
                    cout << "Path Length: " << path_length << endl;
                    return path;
                }
                if(BugMem.t_intersect_mray < BugMem.mline_info.ray.t){
                    break;
                }

                //Failure Check
                if(BugMem.steps_follow > max_steps){
                    cout << "Bug2 Fail in Boundary Follow\n";
                    return path;
                }

                update_bug_mem();
                update_gap();
                move_forward(problem);
                follow_boundary_update_dir(problem);
                path.waypoints.push_back(Eigen::Vector2d(posdir.r.x, posdir.r.y));
            }
            cout << "End Follow Boundary\n";
            if(DEBUG_PRINT){
                print_bugmemory();
            }

            // 2 Cases:
            // (2) encouter q_hit: ???
            if(point_reached(BugMem.Q_Hit) && BugMem.steps_follow > 500){
                cout << "Encounter Q_Hit\n";
                break;
            }
            // (3) find a point on m-ray to leave: move to goal
            if(BugMem.t_intersect_mray < BugMem.mline_info.ray.t){
                cout << "Time to LEAVE!\n";
                // update direction
                MyVect2D t_vect = norm_vect(MyVect2D{r_goal.x-posdir.r.x,
                                                       r_goal.y-posdir.r.y});
                update_dir_to_local_tangent(t_vect);
            }
        }
        // DEBUG USAGE
        // path.waypoints.push_back(Eigen::Vector2d(posdir.r.x, posdir.r.y));
        // if(posdir.r.x > 8.9){
        //     DEBUG_PRINT = true;
        //     int stop;
        //     cout << "stop? ";
        //     cin >> stop;
        //     if(stop == 1){
        //         return path;
        //     }
        // }
        // int stop;
        // cout << "stop? ";
        // cin >> stop;
        // if(stop == 1){
        //     return path;
        // }
        move_forward(problem);
    }
    return path;
}

void MyBugAlgorithm::ini_plan(){
    stepsize = 0.005;
    forward_or_backward = 1.0;
    Obs_Info.clear();
    DEBUG_PRINT = false;
    path_length = 0;
}
void MyBugAlgorithm::ini_goal(const amp::Problem2D& problem){
    cout << "init_goal() ";
    r_goal = MyVect2D{problem.q_goal[0], problem.q_goal[1]};
    cout << "r_goal(" << r_goal.x << " " << r_goal.y << ")\n";
}
void MyBugAlgorithm::init_posdir(const amp::Problem2D& problem){
    cout << "init_posdir()\n";

    posdir.r = MyVect2D{problem.q_init[0], problem.q_init[1]};
    posdir.tdir = norm_vect(MyVect2D{problem.q_goal[0]-problem.q_init[0],
                                     problem.q_goal[1]-problem.q_init[1]});                           

    //calculate cooridnate angle, and set ndir 
    MyVect2D v1 = {0.0, 1.0};
    double dot = v1.x*posdir.tdir.x + v1.y*posdir.tdir.y;   
    double det = v1.x*posdir.tdir.y - v1.y*posdir.tdir.x;
    double angle = atan2(det, dot);
    posdir.ndir = rotate_theta(MyVect2D{1.0,0.0}, angle);
    print_posdir();
}
void MyBugAlgorithm::init_obs_info(const amp::Problem2D& problem){
    cout << "init_obs_info()\n";
    for(int i = 0; i < problem.obstacles.size(); i++){
        vector<LineInfo> l_info = get_singleobs_info(problem.obstacles[i]);
        // debug
        //cout << "Obs: " << i << "\t";
        //print_obslineinfo(l_info);

        // set private variables
        Obs_Info.push_back(l_info);
    }
}
void MyBugAlgorithm::init_bug_mem(){
    BugMem.Q_Hit = posdir.r;
    BugMem.steps_follow = 0;
    BugMem.d_to_goal = 999;
    BugMem.steps_close = 0;
    cout << "Init Bug Memory\n";
    // init m-ray
    BugMem.mline_info.ray = cal_ray2D(posdir.r.x, posdir.r.y, r_goal.x, r_goal.y);
    BugMem.mline_info.l = cal_line2D(posdir.r.x, posdir.r.y, r_goal.x, r_goal.y);
    // init t with m-ray
    BugMem.t_intersect_mray = BugMem.mline_info.ray.t;
    
}
vector<LineInfo> MyBugAlgorithm::get_singleobs_info(const amp::Polygon single_obs){
    //init
    vector<LineInfo> l_info_list;
    for (int i=0; i < single_obs.verticesCCW().size(); i++){
        double x1,y1,x2,y2 = 0;
        if(i+1 != single_obs.verticesCCW().size()){
            x1 = single_obs.verticesCCW()[i][0];
            y1 = single_obs.verticesCCW()[i][1];
            x2 = single_obs.verticesCCW()[i+1][0];
            y2 = single_obs.verticesCCW()[i+1][1];
        }
        else{
            x1 = single_obs.verticesCCW()[i][0];
            y1 = single_obs.verticesCCW()[i][1];
            x2 = single_obs.verticesCCW()[0][0];
            y2 = single_obs.verticesCCW()[0][1];
        }
        line2D line = cal_line2D(x1,y1,x2,y2);
        MyRay2D ray = cal_ray2D(x1,y1,x2,y2);
        LineInfo l_info;
        l_info.l = line;
        l_info.ray = ray;
        l_info_list.push_back(l_info);
    }
    return l_info_list;
}
///////

/// Action Funtions
void MyBugAlgorithm::move_forward(const amp::Problem2D& problem){
    //print_posdir();
    double x_new = posdir.r.x + posdir.tdir.x*stepsize * forward_or_backward;
    double y_new = posdir.r.y + posdir.tdir.y*stepsize * forward_or_backward;
    posdir.r = MyVect2D{x_new, y_new};

    //record path length
    path_length = path_length + stepsize;

    if(DEBUG_PRINT){
        print_posdir();
    }
}
void MyBugAlgorithm::left_turn(){
    // 1st ray:
    LineInfo l1_info;
    l1_info.l = cal_line2D(posdir.r.x, posdir.r.y, 
                           posdir.r.x + posdir.tdir.x*stepsize, 
                           posdir.r.y + posdir.tdir.y*stepsize);            
    l1_info.ray.dir = norm_vect(posdir.tdir);
    l1_info.ray.pos = posdir.r;
    l1_info.ray.t = 1;

    MyVect2D local_tangent = follow_boundary_get_local_tangent(
                             l1_info.ray.dir,
                             Obs_Info);
    
    // // 2nd ray:
    // MyVect2D l2_vect = rotate_theta(posdir.tdir, 30*3.14/180);
    // l2_vect = norm_vect(l2_vect);
    // line2D l2 = cal_line2D(posdir.r.x, posdir.r.y, posdir.r.x+l2_vect.x, posdir.r.y+l2_vect.y);
    // LineInfo l2_info;
    // l2_info.l = l2;
    // l2_info.ray.dir = l2_vect;
    // l2_info.ray.pos = posdir.r;
    // l2_info.ray.t = 1;
    
    // set_sense_point(l1_info, l2_info, Obs_Info[index_col_obs]);
    // cout << "\ndebugging. set_sense points: \n";
    // cout << SensePoints.x1 << " " << SensePoints.y1 << "\n" 
    //      << SensePoints.x2 << " " << SensePoints.y2 << "\n";

    // MyVect2D tang_vect = norm_vect({SensePoints.x2 - SensePoints.x1, 
    //                                 SensePoints.y2 - SensePoints.y1});
    if(DEBUG_PRINT){
        cout << "tangent_vector: " << local_tangent.x << " " 
                                   << local_tangent.y << endl;
    } 
    update_dir_to_local_tangent(local_tangent);
    //print_posdir();
    //cout << posdir.dx << " " << posdir.dy << " " << posdir.ndir.x << " " << posdir.ndir.y << endl;
}
bool MyBugAlgorithm::hit_obstacle(){
    // 1st ray along tdir
    // LineInfo l_info;
    // l_info.l = cal_line2D(posdir.r.x, posdir.r.y, 
    //                       posdir.r.x+posdir.tdir.x, 
    //                       posdir.r.y+posdir.tdir.y);
    // l_info.ray.dir = posdir.tdir;
    // l_info.ray.pos = posdir.r;
    // l_info.ray.t = 1;
    // for(int i=0; i<Obs_Info.size(); i++){
    //     MyVect2D p = ray_cast_single_obs(l_info, Obs_Info[i]);
    //     double d = cal_distance(p, l_info.ray.pos);
    //     if(d<0.05){
    //         //index_col_obs = i;
    //         cout << "Hit Obs: " << i << " at "
    //              << "p: " << p.x << " " << p.y << endl;
    //         if(DEBUG_PRINT){
    //             print_obslineinfo(Obs_Info[i]);
    //             print_mylineinfo(l_info);
    //         }
    //         return true;
    //     }
    // }

    // Multiple ray
    int N = 72;
    double delta_angle = 360.0/(double)N;
    for(int i=0; i<N; i++){
        double angle = (double)i*(delta_angle)*(3.14/180.0);
        LineInfo l_info;
        MyVect2D v = rotate_theta(l_info.ray.dir, angle);
        l_info.l = cal_line2D(posdir.r.x, posdir.r.y, 
                            posdir.r.x+v.x, 
                            posdir.r.y+v.y);
        l_info.ray.dir = v;
        l_info.ray.pos = posdir.r;
        l_info.ray.t = 1;
        for(int j=0; j<Obs_Info.size(); j++){
            MyVect2D p = ray_cast_single_obs(l_info, Obs_Info[j]);
            double d = cal_distance(p, l_info.ray.pos);
            if(d<0.05){
                if(DEBUG_PRINT){
                    cout << "Hit Obs: " << j << " at "
                     << "p: " << p.x << " " << p.y << endl;
                    print_obslineinfo(Obs_Info[j]);
                    print_mylineinfo(l_info);
                }
                return true;
            }
        }
    }
    return false;

    /*for (int i=0; i<Obs_Primitivies.size(); i++){
        // init count
        int count = 0;
        for (int j=0; j<Obs_Primitivies[i].size(); j++){
            line2D p = Obs_Primitivies[i][j];
            double ans = p.a*posdir.x + p.b*posdir.y + p.c;
            //init bool
            bool pos_bool;
            if(ans > 0){
                pos_bool = true;
            }
            else{
                pos_bool = false;
            }
            //cout << "(" << i << j << "):" << pos_bool << " | " << Col_Bools_Ref[i][j] << "\n";
            if(pos_bool == Col_Bools_Ref[i][j]){
                count = count + 1;
            }
        }
        if(count == Obs_Primitivies[i].size()){
            index_col_obs = i;
            cout << "collide with Obs: " << i << "\n";
            cout << "(x,y): " << posdir.x << " " << posdir.y << "\n";
            return true;
        }
    }
    return false;
    */
}
bool MyBugAlgorithm::point_reached(const MyVect2D p){
    double d = cal_distance(posdir.r, p);
    if( abs(d) < 0.1){
        return true;
    }
    else{
        return false;
    }
}
///////

/// Follow Boundary Functions
void MyBugAlgorithm::follow_boundary_update_dir(const amp::Problem2D& problem){
    // estimate local tangent
    MyVect2D local_tangent = follow_boundary_get_local_tangent(
        posdir.ndir,
        Obs_Info);
    //cout << "local tangent: " << local_tangent.x << " " << local_tangent.y << endl;
    update_dir_to_local_tangent(local_tangent);
}
MyVect2D MyBugAlgorithm::follow_boundary_get_local_tangent(
    const MyVect2D vect, 
    const vector<vector<LineInfo>> l_info_obs
    )
{
    //vector<double> d_list;
    vector<MyVect2D> q_list;

    double scan_angle = 150.0;
    double N = 15.0;
    double delta_angle = scan_angle/N;
    int half_N = floor(N*0.5);
    //cout << "vect: " << vect.x << " " << vect.y << endl;
    for(int i = 0; i<N; i++){
        double angle = (double)(i-half_N)*(delta_angle)*3.14/180.0;
        MyVect2D dir = rotate_theta(vect, angle);
        line2D l = cal_line2D(posdir.r.x,  posdir.r.y, 
                              posdir.r.x + dir.x,
                              posdir.r.y + dir.y);
        LineInfo l_info;
        l_info.l = l;
        l_info.ray.dir = dir;
        l_info.ray.pos = posdir.r;
        l_info.ray.t = 1;
        /*if(DEBUG_PRINT && i ==0){
            print_myray2D(l_info.ray);
            //cout << "DEBUG: " << l_info.ray.pos.x << endl;
        }*/
  
        //Multi Obstacle Ray Casting
        // j: index of obstacle
        double d_rayi_obsj = 999;
        MyVect2D qi;
        double index_obsj = -1;
        for(int j=0; j<l_info_obs.size(); j++){
            MyVect2D qij = ray_cast_single_obs(l_info, l_info_obs[j]);
            //cout <<  i << " : " << angle*180.0/3.14 << " " << qij.x << " " << qij.y << endl;
            double d = cal_distance(posdir.r, qij);
            if(d < d_rayi_obsj){
                d_rayi_obsj = d;
                qi = qij;         
            }

            /*if(DEBUG_PRINT && i==0){
                 cout << "ray at obs " << j << "): " << qij.x << " " << qij.y << endl;
            }*/
        }
        if(d_rayi_obsj < 0.5){
            q_list.push_back(qi);
        }
    }

    //debug
    if(DEBUG_PRINT){
        cout << "q_list size: " << q_list.size() << endl;
        for(int i =0; i < q_list.size(); i++){
            cout << "q(" << i << ") " 
                << q_list[i].x << " "<< q_list[i].y << endl;
        }
    }

    // No local tangent
    if(q_list.size()<1){
        return posdir.tdir;
    } 

    // Store the gap with the following obstacle
    BugMem.gap = cal_distance(posdir.r, q_list[half_N]);
    //cout << "Gap: " << BugMem.gap << endl;

    // Estimate Local Tangent By Two End Points.
    MyVect2D p1 = q_list[0];
    MyVect2D p2 = q_list[q_list.size()-1];
    MyVect2D result = norm_vect({p2.x-p1.x, p2.y- p1.y});

    /*if(DEBUG_PRINT){
        cout << "p1: " << p1.x << " " << p1.y << endl;
        cout << "p2: " << p2.x << " " << p2.y << endl;
        cout << "get_local_tangent: " << result.x << " " << result.y << endl;
    }*/

    //MyVect2D result_re = regression(q_list);
    //cout << " regression: " << round(result_re.x*10.0)/10.0 << " " 
    //     << round(result_re.y*10.0)/10.0 << endl;

    return result;
}
///////

/// Helper Functions
line2D MyBugAlgorithm::cal_line2D(double x1, double y1, double x2, double y2){
    double a,b,c = 0;
    a = y1-y2;
    b = x2-x1;
    c = x1*y2 - x2*y1;
    line2D line = {a, b, c};
    return line;
}
MyRay2D MyBugAlgorithm::cal_ray2D(double x1, double y1, double x2, double y2){
    // init 
    MyRay2D ray;
    
    MyVect2D pos;
    pos.x = x1;
    pos.y = y1;
    ray.pos = pos;
    
    MyVect2D dir;
    dir.x = x2 - x1;
    dir.y = y2 - y1;
    dir = norm_vect(dir);
    ray.dir = dir;
    
    if( abs(x2-x1) < 0.01){
        ray.t = (y2-y1)/(dir.y);
    }
    else{
        ray.t = (x2-x1)/(dir.x);
    }
    return ray;
}
double MyBugAlgorithm::cal_distance(MyVect2D v1, MyVect2D v2={0.0,0.0}){
    return pow(pow(v1.x-v2.x,2)+pow(v1.y-v2.y,2),0.5);
}
MyVect2D MyBugAlgorithm::norm_vect(MyVect2D V){
    //init
    MyVect2D V_norm = {0.0, 0.0};
    
    //double d = pow(pow(V.x,2)+pow(V.y,2),0.5);
    double d = cal_distance(V);
    if(d > 0){
        V_norm.x = V.x/d;
        V_norm.y = V.y/d;
        return V_norm;
    }
    else{
        return V_norm;
    }
}
MyVect2D MyBugAlgorithm::rotate_theta(MyVect2D V, double angle){
    Matrix2D M;
    M.M11 = cos(angle); M.M12 = -sin(angle);
    M.M21 = sin(angle); M.M22 = cos(angle);
    MyVect2D V_new;
    V_new = norm_vect(matrix_mul(M,V));
    return V_new;
}
MyVect2D MyBugAlgorithm::matrix_mul(Matrix2D M, MyVect2D V){
    MyVect2D result;
    result.x = M.M11*V.x + M.M12*V.y;
    result.y = M.M21*V.x + M.M22*V.y;
    return result;
}
MyVect2D MyBugAlgorithm::get_two_line_intersect(line2D l1, line2D l2){
    MyVect2D ans;
    ans.x = (l1.b*l2.c-l2.b*l1.c)/(l1.a*l2.b-l2.a*l1.b);
    ans.y = (l2.a*l1.c-l1.a*l2.c)/(l1.a*l2.b-l2.a*l1.b);
    return ans;
}
MyVect2D MyBugAlgorithm::ray_cast_single_obs(const LineInfo l_info_ray, vector<LineInfo> l_info_obs_i){

    // init
    vector<MyVect2D> p_list;
    for(int i=0; i<l_info_obs_i.size(); i++){
        LineInfo l_info = l_info_obs_i[i];
        MyVect2D p_i = get_two_line_intersect(l_info_ray.l, l_info.l);

        double t_p_i = 999.0;
        if(abs(l_info.ray.dir.x) > 0.1){
            t_p_i = (p_i.x - l_info.ray.pos.x)/l_info.ray.dir.x;
        }
        else{
            t_p_i = (p_i.y - l_info.ray.pos.y)/l_info.ray.dir.y;
        }

        double t_p_r = 999.0;
        if(abs(l_info_ray.ray.dir.x) > 0.1){
            t_p_r = (p_i.x - l_info_ray.ray.pos.x)/l_info_ray.ray.dir.x;
        }
        else{
            t_p_r = (p_i.y - l_info_ray.ray.pos.y)/l_info_ray.ray.dir.y;
        }
        /*if(DEBUG_PRINT){
            print_myray2D(l_info_ray.ray);
            cout << i << " edge, tpi: " << t_p_i 
                 << " x " << p_i.x << " y " << p_i.y
                 << " posx: " << l_info.ray.pos.x 
                 << " dirx: " << l_info.ray.dir.x << endl;
        }*/
        // debug
        //cout << "edge: " << i << endl;
        //print_mylineinfo(l_info_obs_i[i]);
        //cout << "px: " << p_i.x << " py: " << p_i.y << endl;
        //cout << "ti: " << t_p_i << endl << endl;
        if(t_p_i <= l_info.ray.t && t_p_i >= 0 &&
           t_p_r <= l_info_ray.ray.t && t_p_r >= 0){
            p_list.push_back(p_i);
        }
    }

    /*if(DEBUG_PRINT){
        for(int i =0; i<p_list.size(); i++){
            cout << "plist: " << p_list[i].x << " " << p_list[i].y << endl;
        }
    }*/

    // reterive the point that is closest
    MyVect2D null;
    null.x = 999;
    null.y = 999;

    if(p_list.size()>0){
        // get the smallist distance
        double d_ref = 999.0;
        int index_ref = -1;
        for(int i=0; i<p_list.size(); i++){
            // debug
            double d = cal_distance(p_list[i], posdir.r);
            if(d<d_ref){
                d_ref = d;
                index_ref = i;
            }
        }
        if(index_ref > -1){
            return p_list[index_ref];
        }
        else{
            return null;
        }
    }
    else{
        return null;
    }
}
void MyBugAlgorithm::set_sense_point(const LineInfo l1_info, 
                                     const LineInfo l2_info,
                                     const vector<LineInfo> l_singleobs_info)
{   
    MyVect2D q_sense1 = ray_cast_single_obs(l1_info, l_singleobs_info);
    SensePoints.x1 = q_sense1.x;
    SensePoints.y1 = q_sense1.y;

    MyVect2D q_sense2 = ray_cast_single_obs(l2_info, l_singleobs_info);
    SensePoints.x2 = q_sense2.x;
    SensePoints.y2 = q_sense2.y;
}
MyVect2D MyBugAlgorithm::regression(const vector<MyVect2D> points){
    vector<double> x;
    vector<double> y;
    
    double sum_xy = 0;
    double sum_x = 0;
    double sum_y = 0;
    double sum_x_square = 0;
    double sum_y_square = 0;
    for(int i=0; i<points.size(); i++){
        double xi = points[i].x;
        double yi = points[i].y;
        sum_xy += xi * yi;
        sum_x += xi;
        sum_y += yi;
        sum_x_square += xi * xi;
        sum_y_square += yi * yi;
        x.push_back(xi);
        y.push_back(yi);
        //cout << "xi : " << xi << " yi: " << yi << endl;     
    }


    double N = x.size();
    double numerator
            = (N * sum_xy - sum_x * sum_y);
    double denominator
            = (N * sum_x_square - sum_x * sum_x);
    if(abs(denominator) < 0.01){
        //cout << "a=inf: " << y[N-1] << " " << y[0];
        return norm_vect(MyVect2D{0,y[N-1]-y[0]});
    }
    else{
        double coeff = numerator / denominator;
        return norm_vect(MyVect2D{1,coeff});
    }

}
void MyBugAlgorithm::update_dir_to_local_tangent(const MyVect2D t_v){
    posdir.tdir = t_v;
    posdir.ndir = rotate_theta(t_v, -90.0*3.14/180.0);
}
void MyBugAlgorithm::update_bug_mem(){
    BugMem.steps_follow++;
    double d = cal_distance(posdir.r, r_goal);
    if(d<BugMem.d_to_goal){
        BugMem.d_to_goal = d;
        BugMem.Q_Close = posdir.r;
        BugMem.steps_close = BugMem.steps_follow;
    }

    //check if point p is on the m line
    line2D l = BugMem.mline_info.l;
    MyRay2D ray = BugMem.mline_info.ray;
    double ans = l.a*posdir.r.x + l.b*posdir.r.y + l.c;
    if(abs(ans) < 0.4){
        //check if tp is on the m-ray
        double t = -1;
        if(abs(ray.dir.x) > 0.1){
            t = (posdir.r.x - ray.pos.x)/ray.dir.x;
        }
        else{
            t = (posdir.r.y - ray.pos.y)/ray.dir.y;
        }
        if(t>0){
            BugMem.t_intersect_mray = t;
            //cout << "Hit M Line that is Closest to Goal\n";
        }
    }
}
void MyBugAlgorithm::update_gap(){
    if(DEBUG_PRINT){
        print_posdir();
        cout << "GAP: " << BugMem.gap << endl;
    }

    double gap_min = 0.05;
    if(BugMem.gap < gap_min){
        posdir.r.x = posdir.r.x - (gap_min-BugMem.gap)*posdir.ndir.x;
        posdir.r.y = posdir.r.y - (gap_min-BugMem.gap)*posdir.ndir.y;
    }

    // double gap_max = 0.1;
    // if(BugMem.gap > gap_max){
    //     posdir.r.x = posdir.r.x + (BugMem.gap-gap_max)*posdir.ndir.x;
    //     posdir.r.y = posdir.r.y + (BugMem.gap-gap_max)*posdir.ndir.y;
    // }

    if(DEBUG_PRINT){
        cout << "UPDATE ";
        print_posdir();
    }
}
///////

/// Printer Functions
void MyBugAlgorithm::print_line2D(line2D l){
    cout << "line (a,b,c): " << l.a << " " << l.b << " " << l.c << "\n"; 
}
void MyBugAlgorithm::print_myray2D(const MyRay2D ray){
    cout << " ray: (";
    cout << ray.pos.x << " " << ray.pos.y << " ";
    cout << ray.dir.x << " " << ray.dir.y << " ";
    cout << ray.t << ")" << endl;
}
void MyBugAlgorithm::print_mylineinfo(const LineInfo l_info){
    cout << " Line Info: ";
    print_line2D(l_info.l);
    print_myray2D(l_info.ray);
}
void MyBugAlgorithm::print_obslineinfo(const vector<LineInfo> l_info_list){
    for(int i=0; i<l_info_list.size(); i++){
        print_mylineinfo(l_info_list[i]);
    }
}
void MyBugAlgorithm::print_posdir(){
    cout << "r: " << posdir.r.x << " " << posdir.r.y << endl;
    cout << "tdir: " << posdir.tdir.x << " " << posdir.tdir.y << endl;
    cout << "ndir: " << posdir.ndir.x << " " << posdir.ndir.y << endl;
}
void MyBugAlgorithm::print_bugmemory(){
    cout << "Bug Memory:";
    cout << " total steps: " << BugMem.steps_follow;
    cout << " steps close: " << BugMem.steps_close;
    cout << " q close: " << BugMem.Q_Close.x << " " << BugMem.Q_Close.y;
    cout << " d close: " << BugMem.d_to_goal << endl;
}
///////

/*
vector<line2D> MyBugAlgorithm::cal_singleobs_primitive(const amp::Polygon single_obs){
    //init
    vector<line2D> obs_primitive;

    for (int i=0; i < single_obs.verticesCCW().size(); i++){
        // init
        double x1,y1,x2,y2 = 0;
        //cout << "vertex: " << i << "\n";
        //std::cout << single_obs.verticesCCW()[i];
        if(i+1 != single_obs.verticesCCW().size()){
            x1 = single_obs.verticesCCW()[i][0];
            y1 = single_obs.verticesCCW()[i][1];
            x2 = single_obs.verticesCCW()[i+1][0];
            y2 = single_obs.verticesCCW()[i+1][1];
        }
        else{
            x1 = single_obs.verticesCCW()[i][0];
            y1 = single_obs.verticesCCW()[i][1];
            x2 = single_obs.verticesCCW()[0][0];
            y2 = single_obs.verticesCCW()[0][1];
        }
        //cout << x1 << y1 << "\t" << x2 << y2 << "\t";
        line2D line = cal_line2D(x1,y1,x2,y2);
        //LOG(" a: " << line.a << " b: " << line.b << " c: " << line.c << "\n");
        obs_primitive.push_back(line);
    }
    return obs_primitive;
}

void MyBugAlgorithm::init_primitives(const amp::Problem2D& problem){  
    for(int i=0; i<problem.obstacles.size(); i++){
        vector<line2D> single_obs_primitive = cal_singleobs_primitive(problem.obstacles[i]);
        Obs_Primitivies.push_back(single_obs_primitive);
    }
}

void MyBugAlgorithm::print_primitives(){
    cout << "\nprint obstacles primitives:\n";
    for(int i=0; i<Obs_Primitivies.size(); i++){
        cout << "Obs " << i << " :\n";
        for(int j=0; j<Obs_Primitivies[i].size(); j++){
            cout << " a: " << Obs_Primitivies[i][j].a << " " 
            << " b: " << Obs_Primitivies[i][j].b  << " "
            << " c: " << Obs_Primitivies[i][j].c;
            cout << "\n";
        }
        cout << "\n";
    }
}
vector<bool> MyBugAlgorithm::cal_singleobs_colbools(const amp::Polygon single_obs, vector<line2D> singleobs_primitive){
    vector<bool> test_bools;
    
    double x_sum = 0;
    double y_sum = 0;
    for (int i =0; i < single_obs.verticesCCW().size(); i++){
        x_sum += single_obs.verticesCCW()[i][0];
        y_sum += single_obs.verticesCCW()[i][1];
    }
    x_sum = x_sum/single_obs.verticesCCW().size();
    y_sum = y_sum/single_obs.verticesCCW().size();

    for (int i = 0; i < singleobs_primitive.size(); i++){
        line2D p = singleobs_primitive[i];
        double ans = p.a*x_sum + p.b*y_sum + p.c;
        if(ans > 0){
            test_bools.push_back(true);
        }
        else{
            test_bools.push_back(false);
        }
    }
    return test_bools;
}

void MyBugAlgorithm::print_singleobs_colbools(vector<bool> test_bools){
    for(int i=0; i<test_bools.size(); i++){
        cout << test_bools[i] << " ";
    }
}

void MyBugAlgorithm::init_colbools(const amp::Problem2D& problem){
    for(int i=0; i<problem.obstacles.size(); i++){
        vector<bool> test_bools = cal_singleobs_colbools(problem.obstacles[i], Obs_Primitivies[i]);
        //print_singleobs_colbools(test_bools);
        //cout << "\n";
        Col_Bools_Ref.push_back(test_bools);
    }
}

void MyBugAlgorithm::print_colbools(){
    for(int i=0; i<Col_Bools_Ref.size(); i++){
        print_singleobs_colbools(Col_Bools_Ref[i]);
        cout << "\n";
    }
}
*/



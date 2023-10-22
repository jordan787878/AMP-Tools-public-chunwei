#include "MyGDSalgo.h"

amp::Path2D MyGDSalgo::plan(const amp::Problem2D& problem){
    // debug parameter
    steps = 0;

    // Problem1: d_star_goal = 1, Q_star = 1, xi = 1, eta = 1, stepsize = 0.1
    // Problem2: d_star_goal = 2, Q_star = 1, xi = 1, eta = 0.1, stepsize = 0.1
    // Q_star_list = {2,0.3,0.5,0.5,0.5};
    // Problem3: 

    // Set Hyperparameters: xi, d_star_goal
    xi = 1;
    eta = 0.5;
    
    //d_star_goal = 1; 
    d_star_goal = 1;
    
    stepsize = 0.2;
    
    Q_star = 1;
    //Q_star_list = {2,0.3,0.5,0.5,1.5};
    //Q_star_list = {0.1,0.3,0.5,0.5,0.5};
    Q_star_list = {5,5,5,5,5,5,5,5,5};

    // init path
    amp::Path2D path;
    path.waypoints.push_back(problem.q_init);
    // init pos
    pos = problem.q_init;

    while(true){
        steps = steps + 1;

        double d = cal_distance(pos, problem.q_goal);
        if(steps > 10000){
            std::cout << "cannot find path\n";
            break;
        }
        if(d < 0.25){
            path.waypoints.push_back(problem.q_goal);
            std::cout << "Path Length: " << (double)steps * stepsize << "\n";
            break;
        }

        // get gradient from potential
        Eigen::Vector2d grad_k = cal_gradient(pos, problem.q_goal, problem);
        //std::cout << "grad: " << grad_k[0] << " " << grad_k[1] << "\n";
        //std::cout << "pos: " << pos[0] << " " << pos[1] << "\n";

        // handling local min. case
        if(abs(grad_k[0]) < 1e-8 && abs(grad_k[1]) < 1e-8 ){
            std::cout << "gradient vanish\n";
            break;
        }
        
        // update pos
        Eigen::Vector2d new_pos = pos - stepsize*grad_k;
        pos = new_pos;
        //std::cout << "pos: " << pos[0] << " " << pos[1] << "\n";
        path.waypoints.push_back(pos);

        // //[debug]
        // if(steps%100 == 0){
        //     int io;
        //     std::cin >> io;
        //     if(io == 0){
        //         std::cout << "";
        //     }
        //     else{
        //         break;
        //     }
        // }
    }

    return path;
}

double MyGDSalgo::potential(const Eigen::Vector2d q, const Eigen::Vector2d q_goal, const amp::Problem2D& problem){
    return MyGDSalgo::potential_att(q, q_goal) + MyGDSalgo::potential_rep(q, problem);
}

Eigen::Vector2d MyGDSalgo::cal_gradient(
    const Eigen::Vector2d q, 
    const Eigen::Vector2d q_goal,
    const amp::Problem2D& problem){
    // init
    double h = 1e-5;
    double x = q[0];
    double y = q[1];

    // [temp] only attrative potential now
    double fx = MyGDSalgo::potential(q, q_goal, problem);
    
    Eigen::Vector2d q1 ={q[0]+h, q[1]};
    double fx_plus_h_x = MyGDSalgo::potential(q1, q_goal, problem);
    Eigen::Vector2d q2 ={q[0]-h, q[1]};
    double fx_minus_h_x = MyGDSalgo::potential(q2, q_goal, problem);
    Eigen::Vector2d q3 ={q[0], q[1]+h};
    double fx_plus_h_y = MyGDSalgo::potential(q3, q_goal, problem);
    Eigen::Vector2d q4 ={q[0], q[1]-h};
    double fx_minus_h_y = MyGDSalgo::potential(q4, q_goal, problem);

    double gradX = (fx_plus_h_x - fx_minus_h_x) / (2.0 * h);
    double gradY = (fx_plus_h_y - fx_minus_h_y) / (2.0 * h);
    Eigen::Vector2d grad = {gradX, gradY};

    return grad;
}

double MyGDSalgo::potential_att(const Eigen::Vector2d q, const Eigen::Vector2d q_goal){
    double d = MyGDSalgo::cal_distance(q, q_goal);
    if (d < d_star_goal){
        return 0.5*xi*d*d;
    }
    else{
        return d_star_goal*xi*d - 
               0.5*xi*d_star_goal*d_star_goal;
    }
}

double MyGDSalgo::potential_rep(const Eigen::Vector2d q, const amp::Problem2D& problem){
    // init
    double U_rep = 0;

    // loop over each obstacle
    for(int i=0; i<problem.obstacles.size();i++){
        //std::cout << i << " ";
        amp::Obstacle2D obs = problem.obstacles[i];
        // init mindistance
        double minDistance = std::numeric_limits<double>::max();

        // loop over each edge of the obstacle
        for(int j=0; j<obs.verticesCCW().size(); j++){
            Eigen::Vector2d v1;
            Eigen::Vector2d v2;
            if(j == obs.verticesCCW().size() - 1 ){
                v1 = obs.verticesCCW()[j];
                v2 = obs.verticesCCW()[0];
            }
            else{
                v1 = obs.verticesCCW()[j];
                v2 = obs.verticesCCW()[j+1];
            }

            double dist = pointToLineDistance(q,v1,v2);
            if(dist < minDistance){
                minDistance = dist;
            }

            // debug
            // if(j==0){
            //     std::cout << i << " " << v1[0] << " " << v1[1] << "\n";
            // }
        }

        // calculate U_rep_i
        //std::cout << "minDistance: " << minDistance << "\n";
        double U_rep_i;
        //double q_star = Q_star; // For Problem 1
        double q_star = Q_star_list[i]; // For Problem 2 and 3
        if(minDistance < q_star){
            U_rep_i = 0.5*eta*((1/minDistance) - (1/q_star))*((1/minDistance) - (1/q_star));
        }
        else{
            U_rep_i = 0;
        }
        //std::cout << "Urep(" << i << "): " << minDistance << " " << U_rep_i << "\n";
        U_rep = U_rep + U_rep_i;
    }

    // [temp]
    return U_rep;
}

double MyGDSalgo::cal_distance(const Eigen::Vector2d v1, const Eigen::Vector2d v2){
    double dx = v1[0] - v2[0];
    double dy = v1[1] - v2[1];
    return pow(dx*dx+dy*dy,0.5);
}

// Function to calculate the distance from a point to a line segment defined by two points
double MyGDSalgo::pointToLineDistance(Eigen::Vector2d point, 
                           Eigen::Vector2d lineStart, Eigen::Vector2d lineEnd) {
    Eigen::Vector2d lineDir = lineEnd - lineStart;
    double lineLength = lineDir.norm();
    if (lineLength == 0.0) {
        // If the line segment degenerates to a point, return the distance to that point
        return cal_distance(point, lineStart);
    }

    // Calculate the projection of the point onto the line
    double t = (point - lineStart).dot(lineDir) / (lineLength * lineLength);

    if (t < 0.0) {
        // Closest point is the start of the line segment
        return cal_distance(point, lineStart);
    } else if (t > 1.0) {
        // Closest point is the end of the line segment
        return cal_distance(point, lineEnd);
    } else {
        // Closest point is along the line segment
        Eigen::Vector2d projection = lineStart + t * lineDir;
        return cal_distance(point, projection);
    }
}